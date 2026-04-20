# GBA_MiSTer — RetroAchievements Fork

This is a fork of the official [Game Boy Advance core for MiSTer](https://github.com/MiSTer-devel/GBA_MiSTer) with modifications to support **RetroAchievements** on MiSTer FPGA.

> **Status:** Experimental / Proof of Concept — works together with the [modified Main_MiSTer binary](https://github.com/odelot/Main_MiSTer).

## What's Different from the Original

The upstream GBA core is an FPGA Game Boy Advance implementation. This fork adds one new module and modifies several existing files so the ARM side (Main_MiSTer) can read emulated GBA RAM for achievement evaluation. **No emulation logic was changed** — the core plays games identically to the original.

### Added Files

| File | Purpose |
|------|--------|
| `rtl/ra_ram_mirror_gba.sv` | Option C selective-address mirror — reads IWRAM from BRAM Port B, EWRAM and Cart RAM/Flash from DDRAM, writes cached values back to DDRAM for the ARM CPU |

### Modified Files

| File | Change |
|------|--------|
| `GBA.sv` | Instantiates `ra_ram_mirror_gba`, wires IWRAM BRAM Port B read interface and DDRAM RA channel |
| `rtl/ddram.sv` | Adds a lowest-priority RA channel (read + write) to the existing DDRAM controller — no separate arbiter needed |
| `rtl/gba_top.vhd` | Passes new `ra_iwram_addr` / `ra_iwram_data` ports through to `gba_memorymux` |
| `rtl/gba_memorymux.vhd` | Exposes IWRAM via BRAM Port B — converts `SyncRam` Port B from unused to active, routes byte address to 32-bit word address with byte-select mux |
| `files.qip` | Adds `ra_ram_mirror_gba.sv` to the Quartus project |

### How the RAM Mirror Works

The GBA has three distinct memory regions relevant for achievements. The core uses the **selective address protocol (Option C)**, the same approach used for SNES, Genesis, and other cores:

1. The **ARM binary** writes a list of RAM addresses it needs to DDRAM offset `0x40000` (up to 4096 addresses per frame).
2. On each **VBlank**, the FPGA module dispatches each address to the correct memory backend:
   - **IWRAM** (0x00000–0x07FFF): Read from BRAM Port B of the `gba_memorymux` small RAM, with a double-read collision-detection mechanism (up to 4 retries if the CPU writes Port A at the same cycle).
   - **EWRAM** (0x08000–0x47FFF): Read from DDRAM at `Softmap_GBA_WRam_ADDR`.
   - **Cart RAM / Flash** (0x48000–0x57FFF): Read from DDRAM at `Softmap_GBA_FLASH_ADDR`.
3. Values are packed into 8-byte chunks and written to DDRAM offset `0x48000`, with a response counter so the ARM knows the data is ready.
4. The ARM binary reads the values and feeds them to the rcheevos achievement engine.

**Memory regions exposed:**

| Region | Address Range | Size | Source | Description |
|--------|-------------|------|--------|-------------|
| IWRAM | $00000–$07FFF | 32 KB | BRAM Port B | Internal Work RAM (fast, on-chip) |
| EWRAM | $08000–$47FFF | 256 KB | DDRAM | External Work RAM |
| Cart RAM / Flash | $48000–$57FFF | 64 KB | DDRAM | Save data (Flash/SRAM/EEPROM) |

**Total exposed: 352 KB**

### Key Differences from Other Cores

| Aspect | Gameboy | GBA |
|--------|---------|-----|
| Console ID | 4 | 5 (RC_CONSOLE_GAME_BOY_ADVANCE) |
| RAM size | up to 160 KB | 352 KB |
| IWRAM access | Dual-port BRAM | BRAM Port B with collision retry |
| EWRAM/Cart access | SDRAM ch2 | DDRAM (shared controller, lowest priority) |
| DDRAM integration | Dedicated arbiter | Integrated into existing `ddram.sv` |
| Flash init | Not needed | ARM pre-fills 128 KB with 0xFF (no-save sentinel) |

### DDRAM Layout

```
0x00000   Header:   magic ("RACH") + flags + frame counter
0x40000   AddrReq:  ARM → FPGA address request list (count + request_id + addresses)
0x48000   ValResp:  FPGA → ARM value response cache (response_id + response_frame + values)
```

All data flows through shared DDRAM at ARM physical address **0x3D000000**.

### Flash DDRAM Initialization

On real GBA hardware, erased/unwritten Flash reads `0xFF`. The FPGA core does not pre-fill the DDRAM Flash region, so when no save file is loaded the area stays at `0x00` (DDR3 default). Since many RetroAchievements conditions check for `0xFF` to detect "no save present", the Main_MiSTer ARM binary initializes the 128 KB Flash region at `0x30000000` with `0xFF` on game load when the area is all-zero.

### Architecture Diagram

```
┌───────────────────────────────────────┐
│       GBA FPGA Core                   │
│                                       │
│  IWRAM (32KB)   in BRAM               │
│  EWRAM (256KB)  in DDRAM              │
│  Cart RAM (64KB) in DDRAM             │
└─────────────┬─────────────────────────┘
              │  VBlank
              ▼
┌───────────────────────────────────────┐
│     ra_ram_mirror_gba.sv             │
│  IWRAM: BRAM Port B (collision retry) │
│  EWRAM/Cart: DDRAM lowest-priority ch │
│  Writes header + values to DDRAM      │
└─────────────┬─────────────────────────┘
              │  DDRAM @ 0x3D000000
              ▼
┌───────────────────────────────────────┐
│     Main_MiSTer (ARM binary)          │
│  mmap /dev/mem → reads mirror         │
│  Writes address list → reads values   │
│  Flash init (0xFF fill if no save)    │
│  rcheevos hashes ROM + evaluates      │
└───────────────────────────────────────┘
```

## How to Try It

1. Download the latest GBA core binary (`GBA_*.rbf`) from the [Releases](https://github.com/odelot/GBA_MiSTer/releases) page.
2. Copy the `.rbf` file to `/media/fat/_Console/` on your MiSTer SD card (replacing or alongside the stock GBA core).
3. You will also need the **modified Main_MiSTer binary** from [odelot/Main_MiSTer](https://github.com/odelot/Main_MiSTer) — follow the setup instructions there to configure your RetroAchievements credentials.
4. Reboot your MiSTer, load the GBA core, and open a game that has achievements on [retroachievements.org](https://retroachievements.org/).

## Building from Source

Open the project in Quartus Prime (use the same version as the upstream MiSTer GBA core) and compile. The `ra_ram_mirror_gba.sv` file is already included in `files.qip`.

## Links

- Original GBA core: [MiSTer-devel/GBA_MiSTer](https://github.com/MiSTer-devel/GBA_MiSTer)
- Modified Main binary (required): [odelot/Main_MiSTer](https://github.com/odelot/Main_MiSTer)
- RetroAchievements: [retroachievements.org](https://retroachievements.org/)

---

# Original GBA Core Documentation

*Everything below is from the upstream [GBA_MiSTer](https://github.com/MiSTer-devel/GBA_MiSTer) README and applies unchanged to this fork.*

## [Gameboy Advance](https://en.wikipedia.org/wiki/Game_Boy_Advance) for [MiSTer Platform](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

## HW Requirements/Features
The games can run from a naked DE10-Nano with the built-in DDR-RAM.
However, using SDRAM is highly recommended, as some games may slowdown or lose sync when using DDR-RAM.

When using SDRAM, it requires 32MB SDRAM for games less than 32MB. 32MB games require either 64MB or 128MB module.
SDRAM will be automatically used when available and size is sufficient.

## Bios
Opensource Bios from Normmatt is included, however it has issues with some games.
Original GBA BIOS can be placed to GBA folder with name boot.rom

PLEASE do not report errors without testing with the original BIOS

Homebrew games are sometimes not supported by the official BIOS, 
because the BIOS checks for Nintendo Logo included in the ROM, which is protected by copyright.
To use these ROMs without renaming or removing the boot.rom, 
you can activate the "Homebrew BIOS" settings in OSD.
As the BIOS is already replaced at boot time, you must save this setting and hard reset/reload the GBA core.

## Status
~1600 games tested until ingame.
There is no known official game that doesn't work.
Exceptions are games that require rare extra hardware (mostly japanese).
Some small video glitches remain, see issue list.

## Features
- Saving as in GBA
- Savestates
- FastForward - speed up game by factor 2-4
- CPU Turbomode - give games additional CPU power
- Flickerblend - set to blend or 30Hz mode for games like F-Zero, Mario Kart or NES Classics to prevent flickering effects
- Spritelimit - turn on to prevent wrong sprites for games that rely on the limit (opt-in)
- Cheats
- Color optimizations: shader colors and desaturate
- Rewind: go back up to 60 seconds in time
- Tilt: use analog stick (map stick in Mister Main before)
- Solar Sensor: Set brightness in OSD
- Gyro: use analog stick (map stick in Mister Main before)
- RTC: automatically used, works with RTC board or internet connection
- Rumble: for Drill Dozer, Wario Ware Twisted and some romhacks
- 2x Resolution: game is rendered at 480x320 instead of 240x160 pixels

## Savestates
Core provides 4 slots to save and restore the state. 
Those can be saved to SD Card or reside only in memory for temporary use(OSD Option). 
Usage with either Keyboard, Gamepad mappable button or OSD.

Keyboard Hotkeys for save states:
- <kbd>ALT</kbd>+<kbd>F1</kbd>/<kbd>F2</kbd>/<kbd>F3</kbd>/<kbd>F4</kbd> – save state  
- <kbd>F1</kbd>/<kbd>F2</kbd>/<kbd>F3</kbd>/<kbd>F4</kbd> – restore state

Gamepad:
- <kbd>SAVESTATEBUTTON</kbd>+<kbd>LEFT</kbd>/<kbd>RIGHT</kbd> prev/next savestate slot
- <kbd>SAVESTATEBUTTON</kbd>+<kbd>START</kbd>+<kbd>DOWN</kbd> saves to the selected slot
- <kbd>SAVESTATEBUTTON</kbd>+<kbd>START</kbd>+<kbd>UP</kbd> loads from the selected slot

## Rewind
To use rewind, turn on the OSD Option "Rewind Capture" and map the rewind button.
You may have to restart the game for the function to work properly.
Attention: Rewind capture will slow down your game by about 0.5% and may lead to light audio stutter.
Rewind capture is not compatible with "Pause when OSD is open", so pause is disabled when Rewind capture is on.

## Spritelimit
There are only very few games known that produce glitches without sprite pixel limit.
Those games use the sprite pixel limit automatically.
You can optionally also turn this on if you notice problems.

## 2x Resolution
Only works over HDMI, Analog output is not changed in 2x Resolution mode. 

Improved rendering resolution for:
- Affine background: "Mode7" games, typically racing games like Mario Kart
- Affine sprites: games that scale or rotate sprites

This rendering is experimental and can cause glitches, as not all game behavior can be supported.
Those glitches can not be fixed without gamespecific hacks and therefore will not be fixed. 
Please don't add bugs in such cases.

## Cartridge Hardware supported games
- RTC: Pokemon Sapphire+Ruby+Emerald, Boktai 1+2+3, Sennen Kazoku, Rockman EXE 4.5
- Solar Sensor: Boktai 1+2+3
- Gyro: Wario Ware Twisted
- Tilt: Yoshi Topsy Turvy/Universal Gravitation, Koro Koro Puzzle - Happy Panechu!
- Rumble: Wario Ware Twisted, Drill Dozer

If there is a game you want to play that also uses one of these features, but is not listed, please open a bug request.

For romhacks you can activate the option "GPIO HACK(RTC+Rumble)". Make sure to deactivate it for other games, otherwise you will experience crashes.

## Not included
- Multiplayer features like serial communication
- E-Reader support
- Gameboy Player features

## Information for developers

How to simulate:
https://github.com/MiSTer-devel/GBA_MiSTer/tree/master/sim

How to implement a GPIO module:
https://github.com/MiSTer-devel/GBA_MiSTer/blob/master/gpio_readme.md
