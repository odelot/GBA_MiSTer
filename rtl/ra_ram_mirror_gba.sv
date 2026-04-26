// RetroAchievements RAM Mirror for GBA — Option C (Optimized for routing)
//
// Each VBlank, reads address list from DDRAM (written by ARM),
// fetches byte values from IWRAM (BRAM) or EWRAM/Cart RAM (DDRAM),
// writes results back to DDRAM for the ARM.
//
// GBA RA Memory Map (rcheevos virtual addresses):
//   0x00000-0x07FFF (32KB)  → IWRAM — BRAM Port B
//   0x08000-0x47FFF (256KB) → EWRAM — DDRAM
//   0x48000-0x57FFF (64KB)  → Cart RAM — DDRAM

module ra_ram_mirror_gba #(
parameter [28:0] DDRAM_BASE       = 29'h07A00000,
parameter [24:0] EWRAM_BASE_DWORD = 131072,
parameter [24:0] FLASH_BASE_DWORD = 0
)(
input             clk,
input             reset,
input             vblank,

// IWRAM BRAM Port B read (byte-addressable, 32KB)
output reg [14:0] iwram_addr,
input       [7:0] iwram_dout,

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

localparam [31:0] IWRAM_END = 32'h08000;
localparam [31:0] EWRAM_END = 32'h48000;
localparam [31:0] CART_END  = 32'h58000;

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
// State machine (5-bit, 18 states)
// ======================================================================
localparam S_IDLE = 5'd0;
localparam S_DD_WR_WAIT = 5'd1;
localparam S_DD_RD_WAIT = 5'd2;
localparam S_READ_HDR = 5'd3;
localparam S_PARSE_HDR = 5'd4;
localparam S_READ_PAIR = 5'd5;
localparam S_PARSE_ADDR = 5'd6;
localparam S_DISPATCH = 5'd7;
localparam S_FETCH_IWRAM = 5'd8;
localparam S_IWRAM_WAIT = 5'd9;
localparam S_DDR_DATA_WAIT = 5'd10;
localparam S_STORE_VAL = 5'd11;
localparam S_FLUSH_BUF = 5'd12;
localparam S_WRITE_RESP = 5'd13;
localparam S_WR_HDR0 = 5'd14;
localparam S_WR_HDR1 = 5'd15;
localparam S_IWRAM_VERIFY = 5'd16;
localparam S_IWRAM_CHECK  = 5'd17;
localparam S_WR_DBG = 5'd18;
localparam S_WR_DBG2 = 5'd19;

reg [4:0]  state;
reg [4:0]  return_state;

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
reg  [7:0] dbg_dispatch_cnt;
reg [15:0] dbg_iwram_cnt;
reg [15:0] dbg_ewram_cnt;
reg [15:0] dbg_cart_cnt;
reg [15:0] dbg_ok_cnt;
reg [15:0] dbg_timeout_cnt;
reg [15:0] dbg_first_addr;
reg  [2:0] byte_sel;      // byte position within 64-bit DDRAM qword
reg  [7:0] iwram_first_val; // First BRAM read for collision detection
reg  [2:0] iwram_retry;     // Retry counter for BRAM collision

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
// ======================================================================
always @(posedge clk) begin
if (reset) begin
state             <= S_IDLE;
active            <= 1'b0;
frame_counter     <= 32'd0;
dbg_frame_counter <= 32'd0;
ddram_req         <= dd_ack_s2;
end
else begin
case (state)

// =============================================================
S_IDLE: begin
active <= 1'b0;
if (vblank_rising) begin
active <= 1'b1;
// Reset debug counters
dbg_ok_cnt <= 16'd0;
dbg_timeout_cnt <= 16'd0;
dbg_dispatch_cnt <= 8'd0;
dbg_iwram_cnt <= 16'd0;
dbg_ewram_cnt <= 16'd0;
dbg_cart_cnt <= 16'd0;
dbg_first_addr <= 16'd0;
// Write header with busy=1
ddram_addr <= DDRAM_BASE[24:0];
ddram_din  <= {16'h0100, 8'h01, 8'd0, 32'h52414348};
ddram_be   <= 8'hFF;
ddram_we   <= 1'b1;
ddram_req  <= ~ddram_req;
return_state  <= S_READ_HDR;
state         <= S_DD_WR_WAIT;
end
end

// =============================================================
S_DD_WR_WAIT: begin
if (ddram_req == dd_ack_s2)
state <= return_state;
end

// =============================================================
S_DD_RD_WAIT: begin
if (ddram_req == dd_ack_s2) begin
rd_data <= ddram_dout;
state   <= return_state;
end
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
req_id <= rd_data[63:32];
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
iwram_retry <= 3'd0;
state <= S_FETCH_IWRAM;
end
else if (cur_addr < EWRAM_END) begin
dbg_ewram_cnt <= dbg_ewram_cnt + 16'd1;
byte_sel <= ewram_offset[2:0];
ddram_addr <= {1'b0, ewram_dword[24:1]};
ddram_we   <= 1'b0;
ddram_req  <= ~ddram_req;
return_state  <= S_DDR_DATA_WAIT;
state         <= S_DD_RD_WAIT;
end
else if (cur_addr < CART_END) begin
dbg_cart_cnt <= dbg_cart_cnt + 16'd1;
byte_sel <= cart_offset[2:0];
ddram_addr <= {1'b0, cart_dword[24:1]};
ddram_we   <= 1'b0;
ddram_req  <= ~ddram_req;
return_state  <= S_DDR_DATA_WAIT;
state         <= S_DD_RD_WAIT;
end
else begin
fetch_byte <= 8'd0;
state <= S_STORE_VAL;
end
end

// =============================================================
S_FETCH_IWRAM: begin
// Wait cycle 1: BRAM latches address set in S_DISPATCH
state <= S_IWRAM_WAIT;
end

S_IWRAM_WAIT: begin
// First BRAM read complete. Save value, re-drive address for verification read.
iwram_first_val <= iwram_dout;
iwram_addr <= cur_addr[14:0];  // re-assert for 2nd read
state <= S_IWRAM_VERIFY;
end

S_IWRAM_VERIFY: begin
// Wait cycle 2: BRAM latches address for verification read
state <= S_IWRAM_CHECK;
end

S_IWRAM_CHECK: begin
// Compare 1st and 2nd reads to detect BRAM port B collision
if (iwram_dout == iwram_first_val || iwram_retry >= 3'd4) begin
// Match (or max retries exhausted) — value is reliable
fetch_byte <= iwram_dout;
dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
state <= S_STORE_VAL;
end else begin
// Mismatch — BRAM port B was hijacked by CPU write. Retry.
iwram_first_val <= iwram_dout;
iwram_addr <= cur_addr[14:0];  // re-drive address
iwram_retry <= iwram_retry + 3'd1;
dbg_timeout_cnt <= dbg_timeout_cnt + 16'd1;  // track retries
state <= S_IWRAM_VERIFY;  // back to wait for next BRAM latch
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
state <= S_STORE_VAL;
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
ddram_din  <= {16'h0100, 8'h00, 8'd0, 32'h52414348};
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
ddram_din  <= {8'h01, dbg_dispatch_cnt, 16'd0, dbg_timeout_cnt, dbg_ok_cnt};
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

default: state <= S_IDLE;
endcase
end
end

endmodule
