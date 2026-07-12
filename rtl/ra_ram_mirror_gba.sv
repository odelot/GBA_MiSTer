// RetroAchievements RAM Mirror for GBA — Option C v2 (rtquery mailbox)
//
// Each VBlank, reads address list from DDRAM (written by ARM),
// fetches byte values from IWRAM (BRAM) or EWRAM/Cart RAM (SDRAM/DDRAM),
// writes results back to DDRAM for the ARM.
//
// v2 additions:
//  - Realtime query mailbox (RA_QUERY_* protocol, same as SNES v2+):
//    ARM writes query addresses, FPGA answers within ~µs. Used by the
//    smart cache to resolve AddAddress pointer targets on cache miss.
//  - Game-load clear covers both storage modes:
//      DDR3 mode  : flash region filled with 0xFF, EWRAM zeroed (qword writes)
//      SDRAM mode : flash region filled with 0xFF, EWRAM zeroed (ch4 writes)
//    (v1 cleared the wrong DDR3 region — dword base used as qword address —
//     which corrupted 256KB of ROM instead of clearing EWRAM.)
//  - VBlank and clear triggers are latched (no lost pulses while the FSM
//    is busy polling the query mailbox or clearing).
//  - DDRAM handshake watchdog: a stuck ack can no longer hold the GBA in
//    reset forever via ra_clear_busy.
//
// GBA RA Memory Map (rcheevos virtual addresses):
//   0x00000-0x07FFF (32KB)  → IWRAM — BRAM Port B
//   0x08000-0x47FFF (256KB) → EWRAM — SDRAM ch4 (sdram_en=1) or DDRAM (sdram_en=0)
//   0x48000-0x57FFF (64KB)  → Cart RAM — SDRAM ch4 (sdram_en=1) or DDRAM (sdram_en=0)

module ra_ram_mirror_gba #(
parameter [28:0] DDRAM_BASE       = 29'h07A00000,
parameter [24:0] EWRAM_BASE_DWORD = 131072,
parameter [24:0] FLASH_BASE_DWORD = 0
)(
input             clk,
input             reset,
input             vblank,
input             sdram_en,     // 1=EWRAM/Cart in SDRAM ch4, 0=in DDRAM

// SDRAM ch4 (EWRAM/Cart when sdram_en=1; reads for scan, writes for clear)
output reg [26:1] sdram_addr,
output reg [15:0] sdram_din,
output reg        sdram_rnw,    // 1=read, 0=write (clear path)
output reg        sdram_req,
input      [31:0] sdram_dout,
input             sdram_ready,

// IWRAM BRAM Port B read (byte-addressable, 32KB)
output reg [14:0] iwram_addr,
input       [7:0] iwram_dout,
input       [3:0] iwram_we,   // 1 = IWRAM write in progress (Port B busy)

// IWRAM clear on game load (cart_download-triggered)
input             cart_download,    // ROM loading active; falling edge triggers clear
input             save_loading,     // bk_loading: save file being written to the flash region
output reg        ra_clear_busy,    // holds reset=1 while clearing IWRAM
output reg [12:0] iwram_clear_addr, // BRAM word address to clear (0..8191)
output reg  [3:0] iwram_clear_we,

// DDRAM combined interface (toggle req/ack, lowest priority)
output reg [24:0] ddram_addr,
output reg [63:0] ddram_din,
output reg  [7:0] ddram_be,
output reg        ddram_we,
output reg        ddram_req,
input             ddram_ack,
input      [63:0] ddram_dout,

// Status
output reg        active,
output reg [31:0] dbg_frame_counter
);

// ======================================================================
// Constants
// ======================================================================
localparam [28:0] ADDRLIST_BASE = DDRAM_BASE + 29'h8000;  // byte offset 0x40000 / 8
localparam [28:0] VALCACHE_BASE = DDRAM_BASE + 29'h9000;  // byte offset 0x48000 / 8
localparam [12:0] MAX_ADDRS     = 13'd4096;

// Realtime query mailbox (byte offsets 0x50000/0x50008/0x50088, in qwords)
localparam [28:0] QUERY_CTRL_ADDR = DDRAM_BASE + 29'hA000;
localparam [28:0] QUERY_REQ_BASE  = DDRAM_BASE + 29'hA001;
localparam [28:0] QUERY_RESP_BASE = DDRAM_BASE + 29'hA011;
localparam [7:0]  MAX_RT_QUERIES  = 8'd16;

localparam [31:0] IWRAM_END = 32'h08000;
localparam [31:0] EWRAM_END = 32'h48000;
localparam [31:0] CART_END  = 32'h58000;

// Clear geometry.
// DDR3 (qword writes):  flash 128KB = 16384 qwords @ qword(FLASH_BASE_DWORD/2)
//                       EWRAM 256KB = 32768 qwords @ qword(EWRAM_BASE_DWORD/2)
// SDRAM (16-bit writes): flash 128KB = 65536 halfwords @ hw 0
//                        EWRAM 256KB = 131072 halfwords @ hw 0x40000
localparam [17:0] CLR_DDR_FLASH_LAST  = 18'd16383;
localparam [17:0] CLR_DDR_EWRAM_LAST  = 18'd32767;
localparam [17:0] CLR_SDR_FLASH_LAST  = 18'd65535;
localparam [17:0] CLR_SDR_EWRAM_LAST  = 18'd131071;
localparam [25:0] EWRAM_HW_BASE       = {EWRAM_BASE_DWORD, 1'b0};
localparam [25:0] FLASH_HW_BASE       = {FLASH_BASE_DWORD, 1'b0};

// ======================================================================
// CDC synchronizers for DDRAM toggle ack signals
// ======================================================================
reg dd_ack_s1, dd_ack_s2;
always @(posedge clk) begin
dd_ack_s1 <= ddram_ack; dd_ack_s2 <= dd_ack_s1;
end

// ======================================================================
// VBlank edge detection
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

// ======================================================================
// State machine (6-bit)
// ======================================================================
localparam S_IDLE          = 6'd0;
localparam S_DD_WR_WAIT    = 6'd1;
localparam S_DD_RD_WAIT    = 6'd2;
localparam S_READ_HDR      = 6'd3;
localparam S_PARSE_HDR     = 6'd4;
localparam S_READ_PAIR     = 6'd5;
localparam S_PARSE_ADDR    = 6'd6;
localparam S_DISPATCH      = 6'd7;
localparam S_FETCH_IWRAM   = 6'd8;
localparam S_IWRAM_WAIT    = 6'd9;
localparam S_DDR_DATA_WAIT = 6'd10;
localparam S_STORE_VAL     = 6'd11;
localparam S_FLUSH_BUF     = 6'd12;
localparam S_WRITE_RESP    = 6'd13;
localparam S_WR_HDR0       = 6'd14;
localparam S_WR_HDR1       = 6'd15;
localparam S_WR_DBG        = 6'd16;
localparam S_WR_DBG2       = 6'd17;
localparam S_FETCH_SDR     = 6'd18;  // SDRAM read request
localparam S_SDR_WAIT      = 6'd19;  // Wait for SDRAM ready
localparam S_READ_CFG      = 6'd20;  // Read ARM config byte from DDRAM
localparam S_PARSE_CFG     = 6'd21;  // Extract clear_en_r/rtq_en_r from config word
localparam S_CLEAR_IWRAM   = 6'd22;  // BRAM clear loop (8192 word addresses)
localparam S_CLEAR_DDR     = 6'd23;  // DDRAM clear loop (flash 0xFF, then EWRAM 0x00)
localparam S_CLEAR_DONE    = 6'd24;  // Deassert ra_clear_busy, return to IDLE
localparam S_CLEAR_SDR     = 6'd25;  // SDRAM clear write request
localparam S_CLEAR_SDR_WAIT= 6'd26;  // Wait for SDRAM write done
localparam S_RTQ_PARSE     = 6'd27;  // Parse query control word
localparam S_RTQ_RD_REQ    = 6'd28;  // Read request slot
localparam S_RTQ_REQ_PARSE = 6'd29;  // Latch address/num_bytes, start fetch
localparam S_RTQ_STORE     = 6'd30;  // Accumulate fetched byte into value
localparam S_RTQ_WR_RESP   = 6'd31;  // Write response slot
localparam S_RTQ_WR_CTRL   = 6'd32;  // Write response_seq (commit)

reg [5:0]  state;
reg [5:0]  return_state;
reg [5:0]  fetch_ret;     // state to enter once fetch_byte is valid

reg [31:0] frame_counter;
reg [63:0] rd_data;
reg [31:0] req_count;
reg [31:0] req_id;
reg [12:0] addr_idx;
reg [63:0] addr_word;
reg [31:0] cur_addr;
reg [63:0] collect_buf;
reg  [3:0] collect_cnt;
reg [12:0] val_word_idx;
reg  [7:0] fetch_byte;
reg [15:0] timeout;
reg [19:0] dd_wait_cnt;   // watchdog for DDRAM handshake waits
reg  [7:0] dbg_dispatch_cnt;
reg [15:0] dbg_iwram_cnt;
reg [15:0] dbg_ewram_cnt;
reg [15:0] dbg_cart_cnt;
reg [15:0] dbg_ok_cnt;
reg [15:0] dbg_timeout_cnt;
reg [15:0] dbg_first_addr;
reg  [2:0] byte_sel;      // byte position within 64-bit DDRAM qword

// Latched event flags (pulses survive a busy FSM)
reg        vbl_pending;
reg        clear_pending;

// IWRAM/EWRAM clear control
reg        clear_en_r;         // latched from DDRAM ARM config bit 1
reg        cart_dl_prev;       // previous cart_download for edge detection
reg        clear_trigger;      // one-cycle pulse triggering the clear sequence
reg        clear_phase;        // 0 = flash fill (0xFF), 1 = EWRAM zero
reg [17:0] clear_cnt;          // qword (DDR3) or halfword (SDRAM) counter

// Realtime query mailbox
reg        rtq_en_r;           // latched from DDRAM ARM config bit 0
reg  [7:0] qry_request_seq;
reg  [7:0] qry_last_seen_seq;
reg  [7:0] qry_num;
reg  [3:0] qry_idx;
reg  [2:0] qry_num_bytes;
reg [31:0] qry_value;
reg  [2:0] qry_byte_idx;
reg [10:0] qry_poll_timer;

// ======================================================================
// Address computation helpers
// ======================================================================
wire [31:0] ewram_offset = cur_addr - 32'h8000;
wire [31:0] cart_offset  = cur_addr - 32'h48000;

wire [24:0] ewram_dword = EWRAM_BASE_DWORD + {7'd0, ewram_offset[17:2]};
wire [24:0] cart_dword  = FLASH_BASE_DWORD  + {9'd0, cart_offset[15:2]};

// DDRAM 25-bit address: {1'b0, dword_addr[24:1]} (ddram.sv adds 4'b0011 prefix)
// byte_sel = offset[2:0] (works because base dword addrs are even)

// ======================================================================
// Main state machine
// NOTE: reset is ignored while ra_clear_busy=1 so a reset pulse (e.g.
// bk_loading rising right after the ROM download) cannot abort the clear
// half-way and leave stale bytes behind. The watchdog on the DDRAM waits
// guarantees ra_clear_busy is always eventually released.
// ======================================================================
always @(posedge clk) begin
if (reset && !ra_clear_busy) begin
state             <= S_IDLE;
active            <= 1'b0;
frame_counter     <= 32'd0;
dbg_frame_counter <= 32'd0;
ddram_req         <= dd_ack_s2;
sdram_req         <= 1'b0;
sdram_rnw         <= 1'b1;
clear_en_r        <= 1'b0;
rtq_en_r          <= 1'b0;
ra_clear_busy     <= 1'b0;
iwram_clear_addr  <= 13'd0;
iwram_clear_we    <= 4'b0000;
clear_phase       <= 1'b0;
clear_cnt         <= 18'd0;
vbl_pending       <= 1'b0;
clear_pending     <= 1'b0;
qry_last_seen_seq <= 8'd0;
qry_poll_timer    <= 11'd0;
dd_wait_cnt       <= 20'd0;
fetch_ret         <= S_STORE_VAL;
end
else begin
sdram_req <= 1'b0;  // default: deassert each cycle

// Latch trigger pulses; consumed in S_IDLE (consume wins on same-cycle set)
if (vblank_rising)  vbl_pending   <= 1'b1;
if (clear_trigger)  clear_pending <= 1'b1;

case (state)

// =============================================================
S_IDLE: begin
active <= 1'b0;
if (clear_pending) begin
    // ROM load complete: clear IWRAM, fill flash 0xFF, zero EWRAM
    clear_pending    <= 1'b0;
    ra_clear_busy    <= 1'b1;
    iwram_clear_addr <= 13'd0;
    iwram_clear_we   <= 4'b1111;
    state            <= S_CLEAR_IWRAM;
end else if (vbl_pending) begin
vbl_pending <= 1'b0;
active <= 1'b1;
qry_poll_timer <= 11'd0;
// Reset debug counters
dbg_ok_cnt <= 16'd0;
dbg_timeout_cnt <= 16'd0;
dbg_dispatch_cnt <= 8'd0;
dbg_iwram_cnt <= 16'd0;
dbg_ewram_cnt <= 16'd0;
dbg_cart_cnt <= 16'd0;
dbg_first_addr <= 16'd0;
// Write header with busy=1 (core_version 2.0)
ddram_addr <= DDRAM_BASE[24:0];
ddram_din  <= {16'h0200, 8'h01, 8'd0, 32'h52414348};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
return_state  <= S_READ_CFG;
state         <= S_DD_WR_WAIT;
end
else if (qry_poll_timer < 11'd2000) begin
qry_poll_timer <= qry_poll_timer + 11'd1;
end
else if (rtq_en_r) begin
// Poll realtime query mailbox
qry_poll_timer <= 11'd0;
ddram_addr <= QUERY_CTRL_ADDR[24:0];
ddram_we   <= 1'b0;
ddram_req  <= ~ddram_req;
return_state  <= S_RTQ_PARSE;
state         <= S_DD_RD_WAIT;
end
else begin
qry_poll_timer <= 11'd0;  // no rtquery active, skip poll
end
end

// =============================================================
S_DD_WR_WAIT: begin
dd_wait_cnt <= dd_wait_cnt + 20'd1;
if (ddram_req == dd_ack_s2) begin
dd_wait_cnt <= 20'd0;
state <= return_state;
end else if (&dd_wait_cnt) begin
// Watchdog: DDRAM ack never arrived. Abort, never wedge the GBA.
dd_wait_cnt   <= 20'd0;
ra_clear_busy <= 1'b0;
state         <= S_IDLE;
end
end

// =============================================================
S_DD_RD_WAIT: begin
dd_wait_cnt <= dd_wait_cnt + 20'd1;
if (ddram_req == dd_ack_s2) begin
dd_wait_cnt <= 20'd0;
rd_data <= ddram_dout;
state   <= return_state;
end else if (&dd_wait_cnt) begin
dd_wait_cnt   <= 20'd0;
ra_clear_busy <= 1'b0;
state         <= S_IDLE;
end
end

// =============================================================
// CLEAR: Step 1 — iterate IWRAM BRAM (8192 words = 32KB)
S_CLEAR_IWRAM: begin
    if (iwram_clear_addr == 13'd8191) begin
        iwram_clear_we  <= 4'b0000;  // done: disable BRAM writes
        clear_phase     <= 1'b0;     // flash fill first (shortest race vs save load)
        clear_cnt       <= 18'd0;
        state           <= sdram_en ? S_CLEAR_SDR : S_CLEAR_DDR;
    end else begin
        iwram_clear_addr <= iwram_clear_addr + 13'd1;
    end
end

// =============================================================
// CLEAR: Step 2 (DDR3 mode) — flash fill 0xFF, then EWRAM zeros.
// ddram_addr is a QWORD address; the base parameters are DWORD
// addresses, hence the >>1 (v1 used the dword base directly and
// clobbered ROM at byte 0x100000 instead of clearing EWRAM).
S_CLEAR_DDR: begin
    if (save_loading && !clear_phase) begin
        // A save file started loading: it owns the flash region now.
        // Abandon the 0xFF fill and move on to the EWRAM phase.
        clear_phase <= 1'b1;
        clear_cnt   <= 18'd0;
    end else begin
        ddram_addr <= (clear_phase ? {1'b0, EWRAM_BASE_DWORD[24:1]}
                                   : {1'b0, FLASH_BASE_DWORD[24:1]}) + {7'd0, clear_cnt};
        ddram_din  <= clear_phase ? 64'd0 : {64{1'b1}};
        ddram_be   <= 8'hFF;
        ddram_we   <= 1'b1;
        ddram_req  <= ~ddram_req;
        if (clear_cnt == (clear_phase ? CLR_DDR_EWRAM_LAST : CLR_DDR_FLASH_LAST)) begin
            if (clear_phase) begin
                return_state <= S_CLEAR_DONE;
            end else begin
                clear_phase  <= 1'b1;
                clear_cnt    <= 18'd0;
                return_state <= S_CLEAR_DDR;
            end
        end else begin
            clear_cnt    <= clear_cnt + 18'd1;
            return_state <= S_CLEAR_DDR;
        end
        state <= S_DD_WR_WAIT;
    end
end

// =============================================================
// CLEAR: Step 2 (SDRAM mode) — same regions via 16-bit ch4 writes.
// v1 skipped this ("rely on BIOS clear") but neither the BIOS boot
// nor the core reset clears SDRAM, so stale bytes from the previous
// game persisted — the main source of false achievement triggers.
S_CLEAR_SDR: begin
    if (save_loading && !clear_phase) begin
        // A save file started loading: it owns the flash region now.
        clear_phase <= 1'b1;
        clear_cnt   <= 18'd0;
    end else begin
        sdram_addr <= (clear_phase ? EWRAM_HW_BASE : FLASH_HW_BASE) + {8'd0, clear_cnt};
        sdram_din  <= clear_phase ? 16'h0000 : 16'hFFFF;
        sdram_rnw  <= 1'b0;
        sdram_req  <= 1'b1;
        timeout    <= 16'd0;
        state      <= S_CLEAR_SDR_WAIT;
    end
end

S_CLEAR_SDR_WAIT: begin
    timeout <= timeout + 16'd1;
    if (sdram_ready || timeout >= 16'hFFFF) begin
        sdram_rnw <= 1'b1;
        if (clear_cnt == (clear_phase ? CLR_SDR_EWRAM_LAST : CLR_SDR_FLASH_LAST)) begin
            if (clear_phase) begin
                state <= S_CLEAR_DONE;
            end else begin
                clear_phase <= 1'b1;
                clear_cnt   <= 18'd0;
                state       <= S_CLEAR_SDR;
            end
        end else begin
            clear_cnt <= clear_cnt + 18'd1;
            state     <= S_CLEAR_SDR;
        end
    end
end

// =============================================================
// CLEAR: Step 3 — release GBA from extended reset
S_CLEAR_DONE: begin
    ra_clear_busy <= 1'b0;
    state         <= S_IDLE;
end

// =============================================================
S_READ_CFG: begin
    // Read ARM config word: RA_ARM_CONFIG_OFFSET=0x40 = DDRAM word 8
    ddram_addr <= DDRAM_BASE[24:0] + 25'd8;
    ddram_we   <= 1'b0;
    ddram_req  <= ~ddram_req;
    return_state  <= S_PARSE_CFG;
    state         <= S_DD_RD_WAIT;
end

// =============================================================
S_PARSE_CFG: begin
    // rd_data[0] = RA_ARM_CFG_RTQUERY, rd_data[1] = RA_ARM_CFG_CLEAR_EN
    rtq_en_r   <= rd_data[0];
    clear_en_r <= rd_data[1];
    state      <= S_READ_HDR;
end

// =============================================================
S_READ_HDR: begin
ddram_addr <= ADDRLIST_BASE[24:0];
ddram_we   <= 1'b0;
ddram_req  <= ~ddram_req;
return_state  <= S_PARSE_HDR;
state         <= S_DD_RD_WAIT;
end

// =============================================================
S_PARSE_HDR: begin
req_id    <= rd_data[63:32];
fetch_ret <= S_STORE_VAL;
if (rd_data[31:0] == 32'd0) begin
req_count <= 32'd0;
state     <= S_WRITE_RESP;
end else begin
req_count    <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
                {19'd0, MAX_ADDRS} : rd_data[31:0];
addr_idx     <= 13'd0;
collect_cnt  <= 4'd0;
collect_buf  <= 64'd0;
val_word_idx <= 13'd0;
state        <= S_READ_PAIR;
end
end

// =============================================================
S_READ_PAIR: begin
ddram_addr <= ADDRLIST_BASE[24:0] + 25'd1 + {12'd0, addr_idx[12:1]};
ddram_we   <= 1'b0;
ddram_req  <= ~ddram_req;
return_state  <= S_PARSE_ADDR;
state         <= S_DD_RD_WAIT;
end

// =============================================================
S_PARSE_ADDR: begin
if (!addr_idx[0]) begin
addr_word <= rd_data;
cur_addr  <= rd_data[31:0];
end else begin
cur_addr <= addr_word[63:32];
end
state <= S_DISPATCH;
end

// =============================================================
S_DISPATCH: begin
dbg_dispatch_cnt <= dbg_dispatch_cnt + 8'd1;
if (!dbg_dispatch_cnt)
dbg_first_addr <= cur_addr[15:0];
if (cur_addr < IWRAM_END) begin
dbg_iwram_cnt <= dbg_iwram_cnt + 16'd1;
iwram_addr <= cur_addr[14:0];  // Set BRAM addr early (registered read needs 2 cycles)
timeout   <= 16'd0;            // Reset write-collision watchdog
state <= S_FETCH_IWRAM;
end
else if (cur_addr < EWRAM_END) begin
dbg_ewram_cnt <= dbg_ewram_cnt + 16'd1;
if (sdram_en) begin
    byte_sel  <= {1'b0, ewram_offset[1:0]};
    sdram_addr <= {ewram_dword, 1'b0};
    sdram_req  <= 1'b1;
    state      <= S_FETCH_SDR;
end else begin
    byte_sel <= ewram_offset[2:0];
    ddram_addr <= {1'b0, ewram_dword[24:1]};
    ddram_we   <= 1'b0;
    ddram_req  <= ~ddram_req;
    return_state  <= S_DDR_DATA_WAIT;
    state         <= S_DD_RD_WAIT;
end
end
else if (cur_addr < CART_END) begin
dbg_cart_cnt <= dbg_cart_cnt + 16'd1;
if (sdram_en) begin
    byte_sel  <= {1'b0, cart_offset[1:0]};
    sdram_addr <= {cart_dword, 1'b0};
    sdram_req  <= 1'b1;
    state      <= S_FETCH_SDR;
end else begin
    byte_sel <= cart_offset[2:0];
    ddram_addr <= {1'b0, cart_dword[24:1]};
    ddram_we   <= 1'b0;
    ddram_req  <= ~ddram_req;
    return_state  <= S_DDR_DATA_WAIT;
    state         <= S_DD_RD_WAIT;
end
end
else begin
fetch_byte <= 8'd0;
state <= fetch_ret;
end
end

// =============================================================
S_FETCH_IWRAM: begin
// Wait until Port B is free (no CPU IWRAM write), then data is valid next cycle.
// If timeout (>255 cycles), proceed anyway (rare stale read is better than hang).
timeout <= timeout + 16'd1;
if (iwram_we == 4'b0000 || timeout >= 16'd255)
    state <= S_IWRAM_WAIT;
end

S_IWRAM_WAIT: begin
// BRAM registered output now valid (2 cycles after address set)
fetch_byte <= iwram_dout;
dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
state <= fetch_ret;
end

// =============================================================
// SDRAM ch4 read (sdram_en=1 path for EWRAM/Cart)
S_FETCH_SDR: begin
timeout <= 16'd0;
state   <= S_SDR_WAIT;
end

S_SDR_WAIT: begin
timeout <= timeout + 16'd1;
if (sdram_ready) begin
    case (byte_sel[1:0])
        2'd0: fetch_byte <= sdram_dout[ 7: 0];
        2'd1: fetch_byte <= sdram_dout[15: 8];
        2'd2: fetch_byte <= sdram_dout[23:16];
        2'd3: fetch_byte <= sdram_dout[31:24];
    endcase
    dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
    state <= fetch_ret;
end else if (timeout >= 16'hFFFF) begin
    fetch_byte <= 8'd0;
    dbg_timeout_cnt <= dbg_timeout_cnt + 16'd1;
    state <= fetch_ret;
end
end

// =============================================================
S_DDR_DATA_WAIT: begin
case (byte_sel)
3'd0: fetch_byte <= rd_data[ 7: 0];
3'd1: fetch_byte <= rd_data[15: 8];
3'd2: fetch_byte <= rd_data[23:16];
3'd3: fetch_byte <= rd_data[31:24];
3'd4: fetch_byte <= rd_data[39:32];
3'd5: fetch_byte <= rd_data[47:40];
3'd6: fetch_byte <= rd_data[55:48];
3'd7: fetch_byte <= rd_data[63:56];
endcase
state <= fetch_ret;
end

// =============================================================
S_STORE_VAL: begin
case (collect_cnt[2:0])
3'd0: collect_buf[ 7: 0] <= fetch_byte;
3'd1: collect_buf[15: 8] <= fetch_byte;
3'd2: collect_buf[23:16] <= fetch_byte;
3'd3: collect_buf[31:24] <= fetch_byte;
3'd4: collect_buf[39:32] <= fetch_byte;
3'd5: collect_buf[47:40] <= fetch_byte;
3'd6: collect_buf[55:48] <= fetch_byte;
3'd7: collect_buf[63:56] <= fetch_byte;
endcase
collect_cnt <= collect_cnt + 4'd1;
addr_idx    <= addr_idx + 13'd1;

if (collect_cnt == 4'd7 || (addr_idx + 13'd1 >= req_count[12:0])) begin
state <= S_FLUSH_BUF;
end
else if (addr_idx[0]) begin
// Old addr_idx was odd → next is even → need new pair
state <= S_READ_PAIR;
end else begin
// Old addr_idx was even → next is odd → second addr in addr_word
state <= S_PARSE_ADDR;
end
end

// =============================================================
S_FLUSH_BUF: begin
ddram_addr <= VALCACHE_BASE[24:0] + 25'd1 + {12'd0, val_word_idx};
ddram_din  <= collect_buf;
ddram_be   <= (collect_cnt == 4'd8) ? 8'hFF
              : ((8'd1 << collect_cnt[2:0]) - 8'd1);
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
val_word_idx  <= val_word_idx + 13'd1;
collect_cnt   <= 4'd0;
collect_buf   <= 64'd0;

if (addr_idx >= req_count[12:0]) begin
return_state <= S_WRITE_RESP;
end else if (!addr_idx[0]) begin
// New addr_idx is even → need new pair
return_state <= S_READ_PAIR;
end else begin
// New addr_idx is odd → second addr in addr_word
return_state <= S_PARSE_ADDR;
end
state <= S_DD_WR_WAIT;
end

// =============================================================
S_WRITE_RESP: begin
ddram_addr <= VALCACHE_BASE[24:0];
ddram_din  <= {frame_counter + 32'd1, req_id};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
return_state  <= S_WR_HDR0;
state         <= S_DD_WR_WAIT;
end

// =============================================================
S_WR_HDR0: begin
ddram_addr <= DDRAM_BASE[24:0];
ddram_din  <= {16'h0200, 8'h00, 8'd0, 32'h52414348};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
return_state  <= S_WR_HDR1;
state         <= S_DD_WR_WAIT;
end

// =============================================================
S_WR_HDR1: begin
ddram_addr <= DDRAM_BASE[24:0] + 25'd1;
ddram_din  <= {32'd0, frame_counter + 32'd1};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
frame_counter <= frame_counter + 32'd1;
dbg_frame_counter <= frame_counter + 32'd1;
return_state  <= S_WR_DBG;
state         <= S_DD_WR_WAIT;
end


// =============================================================
S_WR_DBG: begin
ddram_addr <= DDRAM_BASE[24:0] + 25'd2;
ddram_din  <= {8'h02, dbg_dispatch_cnt, 16'd0, dbg_timeout_cnt, dbg_ok_cnt};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
return_state  <= S_WR_DBG2;
state         <= S_DD_WR_WAIT;
end

// =============================================================
S_WR_DBG2: begin
ddram_addr <= DDRAM_BASE[24:0] + 25'd3;
ddram_din  <= {dbg_first_addr, dbg_iwram_cnt, dbg_ewram_cnt, dbg_cart_cnt};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
return_state  <= S_IDLE;
state         <= S_DD_WR_WAIT;
end

// =============================================================
// Realtime Query States (v2)
// =============================================================
S_RTQ_PARSE: begin
// ctrl word: [7:0]=request_seq [15:8]=num_queries [39:32]=response_seq
if (rd_data[7:0] != qry_last_seen_seq && rd_data[15:8] != 8'd0) begin
    qry_request_seq <= rd_data[7:0];
    qry_num         <= (rd_data[15:8] > MAX_RT_QUERIES) ? MAX_RT_QUERIES : rd_data[15:8];
    qry_idx         <= 4'd0;
    state           <= S_RTQ_RD_REQ;
end else begin
    state <= S_IDLE;
end
end

S_RTQ_RD_REQ: begin
ddram_addr <= QUERY_REQ_BASE[24:0] + {21'd0, qry_idx};
ddram_we   <= 1'b0;
ddram_req  <= ~ddram_req;
return_state  <= S_RTQ_REQ_PARSE;
state         <= S_DD_RD_WAIT;
end

S_RTQ_REQ_PARSE: begin
// req slot: [31:0]=address [39:32]=num_bytes (1-4)
cur_addr      <= rd_data[31:0];
qry_num_bytes <= (rd_data[39:32] == 8'd0 || rd_data[39:32] > 8'd4) ? 3'd1 : rd_data[34:32];
qry_value     <= 32'd0;
qry_byte_idx  <= 3'd0;
fetch_ret     <= S_RTQ_STORE;
state         <= S_DISPATCH;
end

S_RTQ_STORE: begin
qry_value    <= qry_value | ({24'd0, fetch_byte} << {qry_byte_idx, 3'b000});
qry_byte_idx <= qry_byte_idx + 3'd1;
if (qry_byte_idx + 3'd1 >= qry_num_bytes) begin
    state <= S_RTQ_WR_RESP;
end else begin
    cur_addr <= cur_addr + 32'd1;
    state    <= S_DISPATCH;
end
end

S_RTQ_WR_RESP: begin
ddram_addr <= QUERY_RESP_BASE[24:0] + {21'd0, qry_idx};
ddram_din  <= {32'd0, qry_value};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
qry_idx    <= qry_idx + 4'd1;
if ({1'b0, qry_idx} + 5'd1 >= qry_num[4:0]) begin
    return_state <= S_RTQ_WR_CTRL;
end else begin
    return_state <= S_RTQ_RD_REQ;
end
state <= S_DD_WR_WAIT;
end

S_RTQ_WR_CTRL: begin
// Commit: response_seq (byte 4) = request_seq. Full-qword write also
// restores request_seq/num so the ARM's next read sees a coherent word.
qry_last_seen_seq <= qry_request_seq;
ddram_addr <= QUERY_CTRL_ADDR[24:0];
ddram_din  <= {24'd0, qry_request_seq, 16'd0, qry_num, qry_request_seq};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
return_state  <= S_IDLE;
state         <= S_DD_WR_WAIT;
end

default: state <= S_IDLE;
endcase
end
end

// ======================================================================
// Edge detection: triggers IWRAM+EWRAM/flash clear on cart_download fall
// cart_dl_prev always tracks cart_download (not gated by reset) so the
// falling edge is detected even after reset deasserts.
// clear_trigger is a 1-cycle pulse latched into clear_pending above.
// NOTE: ra_mirror reset does NOT include ra_clear_busy (see GBA.sv).
//       GBA_on uses gba_clear_reset = reset | ra_clear_busy instead.
// ======================================================================
always @(posedge clk) begin
    cart_dl_prev <= cart_download;  // Track regardless of reset
    if (reset && !ra_clear_busy) begin
        clear_trigger <= 1'b0;
    end else begin
        // One-cycle pulse: cart_download 1→0 AND clear enabled
        clear_trigger <= cart_dl_prev && !cart_download && clear_en_r;
    end
end

endmodule
