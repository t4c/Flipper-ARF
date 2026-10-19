/* Pandora ARM - emulator of the REAL Pandora D-605 keyfob firmware (EFM32
 * Cortex-M3) for Flipper Zero.
 *
 * The keyfob firmware runs UNMODIFIED on the Thumb-2 interpreter
 * (lib/thumbcore). This app emulates the hardware the firmware expects:
 *   - OLED UC16xx/ST7528 (framebuffer 92col x 16pag = 1472B): the firmware
 *     writes it over SPI; the framebuffer lives in RAM (0x20001152 pandora /
 *     0x200005DA pmax), we read it and paint it on the Flipper 128x64 screen.
 *   - 6 buttons (GPIO DIN, pull-up, 0=pressed). BLACK screen until the
 *     unlock knock-code (blind button sequence).
 *   - Si4432 (SPI model), timers, CMU, ADC: minimal stubs so the fw does not
 *     hang. Minimal NVIC (timer tick) to wake up the WFI.
 *
 * Faithfully ported from the reference emulator emu/pandora_tui.py (verified).
 * FlipperGB-style frontend (direct-draw, poll GPIO, safe_malloc, yield).
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_subghz.h>
#include <gui/gui.h>
#include <gui/view_dispatcher.h>
#include <gui/modules/submenu.h>
#include <gui/modules/widget.h>
#include <input/input.h>
#include <dialogs/dialogs.h>
#include <storage/storage.h>

#include <stdlib.h>
#include <string.h>

#include "lib/thumbcore/thumb_core.h"

#define TAG "PandoraARM"

#define SCREEN_W 128
#define SCREEN_H 64
#define FB_SIZE (SCREEN_W * SCREEN_H / 8) /* 1024 bytes */

/* Keyfob OLED: the firmware writes a 92 columns x 16 pages = 1472-byte
 * framebuffer (2bpp). But the PHYSICAL glass is only 88 columns wide (the D-605
 * status screen blits with width=0x58=88; columns 88..91 are off-glass margin,
 * see RE_display_D605_auto.md). So we capture 92 but only RENDER the 88 visible
 * columns, centered on the Flipper's 128-wide screen. */
#define OLED_COLS 92
#define OLED_VISIBLE_COLS 88 /* physical glass width (cols 88..91 = off-glass) */
#define OLED_PAGES 16
#define OLED_FB_SIZE (OLED_COLS * OLED_PAGES)

/* Memory map of the emulated EFM32 (NOT the Flipper; PA_ prefix to avoid
 * clashing with the PA_FLASH_BASE/PA_FLASH_SIZE from the Flipper's own STM32WB CMSIS). */
#define PA_FLASH_BASE 0x00000000u
#define PA_FLASH_SIZE 0x00042000u /* 264 KB (covers pandora 0x3FFFF + margin) */
#define PA_RAM_BASE 0x20000000u
#define PA_RAM_SIZE 0x00008000u /* 32 KB */

#define ALLOC_MARGIN 1024u

/* ---------------------------------------------------------------- FLASH CACHE
 * The emulated firmware's flash is loaded PAGED, not as one contiguous block.
 *
 * ROOT CAUSE of the "Out of memory" at Launch: load_firmware used to
 * safe_malloc(flash_used) = ONE contiguous block of 77KB (pandora) / 84KB
 * (pmax). Measured on hardware (Bluetooth OFF) the heap has ~71KB free but the
 * largest contiguous block is only ~67KB -> the single big malloc never fits ->
 * OOM. (See PandoraARM_mem_fix.md.)
 *
 * FIX (same pattern as the sister app FlipperGB's RomCache): split the flash
 * into 4KB PAGES, each its own small safe_malloc (small blocks fit a fragmented
 * heap), held in an LRU cache. The core reads flash through a CALLBACK (we pass
 * flash_ptr=NULL to thumb_set_regions, leaving only the 32KB RAM as a direct
 * region) and each flash byte is resolved via the cache.
 *
 * Why this stays fast: the EFM32 firmware is RAM-RESIDENT. At boot it copies
 * ~25KB of flash into RAM (0x20000000) and EXECUTES FROM RAM. Instruction fetch
 * post-boot therefore hits the DIRECT RAM region (never the cache). Only the few
 * CONSTANT loads that still address flash (< ~11 distinct 4KB pages post-boot)
 * go through the cache. So the cache is cold-path.
 *
 * Source of the pages: at load time we normalise the firmware (.hex parsed once
 * in streaming, or .bin copied) into a canonical FLAT temp .bin on the SD
 * (flash_used bytes, gaps = 0xFF). Paging is then a trivial seek+read of 4KB
 * from that flat file -- robust and format-agnostic (no re-parsing of Intel HEX
 * on a miss). */
#define FLASH_PAGE_SIZE 0x1000u /* 4 KB cache granularity (matches FlipperGB) */
/* Heap kept free for the GUI direct-draw takeover and system services while the
 * emulator runs. RX has its own early reservation so this reserve is available
 * to the live UI instead of being consumed by the flash cache. */
#define FLASH_HEAP_RESERVE (14u * 1024u)
/* One slot can thrash when XIP instructions, vector tables, and firmware data
 * alternate between flash pages. Require a small working set for launch. */
#define FLASH_MIN_SLOTS 3u

#define FRAME_US 30000u /* ~33 Hz UI refresh */
#define SCREEN_REFRESH_FALLBACK_MS 100u /* repaint static firmware screens too */
#define EMU_INSN_PER_RX_FRAME 3000u /* prioritize receiver injection while in RX */
/* Firmware instructions executed per UI frame. Caps the emulation rate so the
 * firmware's inactivity timeout (~280k insn) fires at a human scale (sub-mode
 * stays ~a couple seconds before auto-returning), like the real keyfob. On the
 * Flipper the Thumb-2 interpreter is slower than on a PC, so this is a ceiling;
 * tune on hardware if needed.
 *
 * CALIBRATION @ ~70 k-insn/s (MEASURED on Cortex-M4 @64MHz, heartbeat log
 * "rate=70 k-insn/s" -- the interpreter is ~33x slower than a PC's 2.3 M-insn/s):
 *   - old 40000 -> 40000/70000 = ~570ms of CPU per main-loop iteration. Input is
 *     only re-polled once per iteration, so buttons were polled ~1.75x/s -> laggy.
 *   - 6000 -> 6000/70000 = ~86ms per loop -> input is polled about 12x/s.
 * OLED conversion and canvas commits happen immediately when firmware commits a
 * new frame, with a slower fallback repaint for static screens and direct-draw
 * surfaces that need periodic refresh. */
#define EMU_INSN_PER_FRAME 6000u

/* Window (in instructions) where the knock scanner is active after boot.
 * The keyfob enters low power afterwards; that is why the unlock is done at
 * startup (same as the Python TUI fix). */
/* @ ~70 k-insn/s: 250000/70000 = ~3.5s settle at boot (runs ONCE, acceptable).
 * Left unchanged -- it is the window the scanner must be active for the unlock. */
#define KNOCK_SETTLE_INSN 250000ull
/* Per-knock-step SAFETY CAP (the step loop exits early when unlocked, so this is
 * rarely reached). @ 70 k-insn/s: old 4M = ~57s/step worst case; new 1.5M =
 * ~21s/step worst case. The unlock normally completes well before this, so this
 * only trims the pathological "never unlocks" timeout; it does NOT reduce the
 * budget available for a successful unlock within a step. */
#define KNOCK_STEP_BUDGET 1500000ull
#define KNOCK_RELEASE_AFTER 4

/* POST-unlock navigation mirrors pandora_tui.py's read-counted short/long
 * presses. Small interpreter slices bound the time between dispatch and redraw. */
#define NAV_HOLD_READS 40u
#define NAV_LONG_READS 120u
/* --- Recalibrated @ ~70 k-insn/s (times below assume the worst case of the cap
 * being reached; in practice the loops cut much earlier). ---
 *
 * NAV_BUDGET_INSN: SAFETY CAP for press+hold+release. The loop exits as soon as
 * nav_phase==2 (release dispatched), which normally happens after the firmware
 * has re-read our pin NAV_HOLD_READS(40)/NAV_LONG_READS(120) times. The press
 * loop advances in short instruction bursts, so a long press needs on the order
 * of a few hundred k insn to register. @ 70 k-insn/s: old 4M = ~57s cap; new 1.0M =
 * ~14s cap. 1.0M still leaves a generous ~20x margin over the insn a long press
 * actually needs to register, so the press is never clipped -- it only trims the
 * "firmware never released" pathological timeout. */
#define NAV_BUDGET_INSN 1000000ull
/* Briefly advance after release; the main loop continues the redraw afterward,
 * so a slow screen update must not keep input blocked. */
#define NAV_SETTLE_INSN 30000ull
/* Require two short settle slices before accepting a framebuffer change. */
#define NAV_SETTLE_MIN_INSN 10000ull
/* Small navigation slices keep the synchronous press/settle path from running
 * hundreds of milliseconds past the firmware's actual button dispatch. */
#define NAV_PRESS_BURST 5000u
#define NAV_SETTLE_BURST 5000u

/* Physical hold detection (Flipper keys). Hold > ~500ms = long press of the
 * corresponding keyfob button. Flipper UP held > 1s = EXIT the app (Button 1
 * has no hold function on the real keyfob, so it is safe to use it for exit).
 * Flipper BACK held > 1s = cycle the RF bridge (OFF/TX/RX) without stealing a
 * keyfob button. */
#define HOLD_MS 500u
#define EXIT_HOLD_MS 1000u
#define RF_HOLD_MS 1000u

static void* safe_malloc(size_t size) {
    if(memmgr_heap_get_max_free_block() < size + ALLOC_MARGIN) return NULL;
    return malloc(size);
}

/* ------------------------------------------------------------ HW profile ----
 *
 * AGNOSTIC HARDWARE PROFILE (ported from the ESP32 port's PROFILE_PANDORA_HW and
 * pandora_tui.py's `HW` dict).
 *
 * This struct now describes ONLY the PHYSICAL hardware of the D650 keyfob, which
 * is IDENTICAL for every firmware of this MCU. It contains NO firmware-internal
 * code/RAM address: no scan_head / spin_heads / unlock_pc / nav_reader /
 * demod_isr / rx_decoder_dispatch / rx_* RAM offsets / ui_* / fb_addr / knock /
 * flash_used. All I/O is done through GPIO pins and the SPI bus as on the real
 * hardware, so the SAME profile runs ANY .hex for this MCU.
 *
 * What used to be per-firmware is handled agnostically:
 *   - flash size is derived from the loaded firmware image length at load time
 *     (fc.flash_used), NOT a hardcoded ArmProfile.flash_used.
 *   - the knock-code is FIRMWARE CONFIG (not hardware): it lives in AppState
 *     (knock_user, set by the host/UI) and is pressed PURELY over GPIO by
 *     auto_unlock, exactly like the ESP32 emu_set_knock / pandora_tui.py.
 *   - unlock is an OBSERVABLE signal (framebuffer has lit pixels), not a code
 *     address.
 *   - the LED/buzzer are detected generically from GPIO activity (no led_port).
 */
typedef struct {
    const char* name;
    /* 6 physical buttons -> (port, pin) GPIO DIN (pull-up, active-low).
     * Identical in every firmware of this keyfob (verified by RE).
     * NOTE: BACK=(0,15) for ALL firmwares. The old 'pandora' profile had
     * BACK=(2,7) which was a BUG: PC7=(2,7) is a vestigial OUTPUT, not a
     * button. Fixed for all (matches PANDORAMAX and the ESP32 port). */
    uint8_t up_port, up_pin;
    uint8_t down_port, down_pin;
    uint8_t ok_port, ok_pin;
    uint8_t back_port, back_pin;
    uint8_t b5_port, b5_pin;
    uint8_t b6_port, b6_pin;
    uint8_t gate; /* button of the final held unlock gate (BTN_* index) */
    /* RF pins (Si4432 direct mode, bit-bang OOK/FSK). Identical on both fw. */
    uint8_t data_port, data_pin;       /* RX-data pin (PB0 = port1.0) */
    uint8_t tx_data_port, tx_data_pin; /* TX-data pin (PB3 = port1.3) */
    uint8_t rf_cs_port, rf_cs_pin;     /* Si4432 CS (PE9 = port4.9, active-high) */
    uint8_t rf_ant_rx_port, rf_ant_rx_pin; /* RX antenna (PE14 = port4.14) */
    uint8_t rf_ant_tx_port, rf_ant_tx_pin; /* TX antenna (PE15 = port4.15) */
    uint16_t rf_tx_cell_us; /* default OOK cell width (us), order-of-magnitude */
} ArmProfile;

/* logical button indices */
enum { B_UP = 0, B_DOWN, B_OK, B_BACK, B5, B6, BTN_COUNT };

/* The ONE hardware profile of the D650 keyfob. AGNOSTIC: no firmware code/RAM
 * address, no per-firmware knock, no flash_used. The SAME profile runs ANY
 * .hex for this MCU. (Ported from the ESP32 PROFILE_PANDORA_HW.) */
static const ArmProfile PROFILE_PANDORA_HW = {
    .name = "pandora-d650-hw",
    .up_port = 5, .up_pin = 7,
    .down_port = 5, .down_pin = 5,
    .ok_port = 2, .ok_pin = 9,
    .back_port = 0, .back_pin = 15, /* FIX: BACK=(0,15) for ALL firmwares */
    .b5_port = 0, .b5_pin = 1,
    .b6_port = 0, .b6_pin = 6,
    .gate = B5,
    /* radio Si4432 (direct mode) pins */
    .data_port = 1, .data_pin = 0,       /* PB0 = RX-data */
    .tx_data_port = 1, .tx_data_pin = 3, /* PB3 = TX-data */
    .rf_cs_port = 4, .rf_cs_pin = 9,     /* PE9 = Si4432 CS (active-high) */
    .rf_ant_rx_port = 4, .rf_ant_rx_pin = 14, /* PE14 = RX antenna/LNA */
    .rf_ant_tx_port = 4, .rf_ant_tx_pin = 15, /* PE15 = TX antenna/PA */
    .rf_tx_cell_us = 400,
};

/* COMPAT: legacy names still referenced by the code. Both alias the SINGLE
 * hardware profile above (there is no per-firmware profile any more), exactly
 * as the ESP32 port did. */
#define PROFILE_PANDORA PROFILE_PANDORA_HW
#define PROFILE_PANDORAMAX PROFILE_PANDORA_HW

/* Default knock-codes for the two known keyfob builds. This is FIRMWARE CONFIG
 * (model-specific), NOT hardware -- the agnostic core neither knows nor needs
 * it. The host/UI supplies it; auto_unlock presses it PURELY over GPIO. If a
 * firmware is unknown we press no default (the user enters the PIN in the UI).
 * Ported from pandora_tui.py _LEGACY_KNOCK. */
static const uint8_t KNOCK_PANDORA[] = {B_UP, B_OK, B_OK, B_UP, B_UP, B_DOWN};
static const uint8_t KNOCK_PMAX[] = {B_UP, B_OK, B_DOWN, B_OK, B_UP, B_DOWN};

/* ------------------------------------------------------------ Si4432 model */

/* DIRECT MODE ONLY (DYN_HW_D605.md §5/§9): both firmwares (pandora, pandoramax)
 * drive the Si4432 in DIRECT mode (bit-bang OOK/FSK over a GPIO pin), NOT the
 * FIFO/packet mode. Dynamic analysis recorded 0 accesses to the FIFO register
 * (0x7F) during a full boot+unlock+RX run of EACH firmware. The old
 * PACKET/FIFO bridge (si_tx_fifo / si_rx_fifo / fifo_tx_transmit / fifo_rx_poll
 * / REG[0x7F] special handling) was therefore REMOVED: it was an incorrect path
 * that never fires on real firmware and wasted 128 B of AppState RAM (good for
 * the paged flash cache). The bridge is now a single DIRECT path: RX injects the
 * demodulated OOK stream into PB0 and the firmware's demod ISR decodes it; TX
 * captures the bit-banged waveform on PB3 and replays it through the CC1101. */

typedef struct {
    uint8_t regs[0x80];
    uint8_t mode; /* 0 idle, 1 ready, 2 rx, 3 tx (derived from REG[0x07]) */
} Si4432;

static void si4432_init(Si4432* s) {
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 0x08;
    s->regs[0x01] = 0x06;
    s->regs[0x26] = 0x60;
    s->mode = 0;
}
/* Direct-mode register read. REG[0x7F] (FIFO Access) is NOT special-cased: the
 * firmware never touches it in direct mode (DYN_HW_D605.md: 0 FIFO accesses). */
static uint8_t si4432_read(Si4432* s, uint8_t reg) {
    reg &= 0x7F;
    uint8_t v = s->regs[reg];
    /* REG 0x03/0x04 are the interrupt status registers: reading them clears the
     * latched flags on the real part. */
    if(reg == 0x03 || reg == 0x04) s->regs[reg] = 0;
    return v;
}
/* Returns the mode if it changed (so the frontend detects rf_tx_on/rf_rx_on),
 * -1 if it did not change. REG[0x07] (Operating Mode): bit3=TX, bit2=RX, bit0=READY. */
static int si4432_write(Si4432* s, uint8_t reg, uint8_t val) {
    reg &= 0x7F;
    s->regs[reg] = val;
    if(reg == 0x07) {
        uint8_t nm = s->mode;
        if(val & 0x08)
            nm = 3; /* TX */
        else if(val & 0x04)
            nm = 2; /* RX */
        else if(val & 0x01)
            nm = 1; /* READY */
        else
            nm = 0; /* IDLE */
        if(nm != s->mode) {
            /* DIAG: the firmware changed the radio operating mode. nm 3=TX 2=RX
             * 1=READY 0=IDLE. This is the key signal that the firmware wants to
             * transmit a frame (nm==3). If you press a button and this never logs
             * TX, the firmware is not driving the radio for that action. */
            FURI_LOG_I(TAG, "si4432: REG[0x07]=0x%02X -> mode %u (%s)", val, nm,
                       nm == 3 ? "TX" : nm == 2 ? "RX" : nm == 1 ? "READY" : "IDLE");
            s->mode = nm;
            return (int)nm;
        }
    }
    return -1;
}

/* ------------------------------------------------------------ app state */

enum {
    KBIT_UP = 1 << 0,
    KBIT_DOWN = 1 << 1,
    KBIT_LEFT = 1 << 2,
    KBIT_RIGHT = 1 << 3,
    KBIT_OK = 1 << 4,
    KBIT_BACK = 1 << 5,
};

/* Physical Flipper keys we track for hold-duration. Index into the key-tracking
 * arrays.
 *
 * BUTTON LAYOUT (Flipper D-pad -> keyfob button number), as requested so the
 * physical D-pad matches the keyfob visually:
 *   Flipper UP    -> keyfob button 6 (B6)   trunk        (hold: back to menu)
 *   Flipper LEFT  -> keyfob button 2 (OK)   confirm/mode (hold: erase cell)
 *   Flipper OK    -> keyfob button 5 (B5)   AM+FM / open car
 *   Flipper RIGHT -> keyfob button 1 (UP)   On / menu / cell +
 *   Flipper DOWN  -> keyfob button 4 (BACK) RX frequency / close car
 *   Flipper BACK  -> keyfob button 3 (DOWN) menu / cell - (hold: auto)
 */
enum {
    FK_UP = 0, /* Flipper UP    -> keyfob button 6 (B6) */
    FK_OK, /* Flipper OK    -> keyfob button 5 (B5) */
    FK_DOWN, /* Flipper DOWN  -> keyfob button 4 (BACK) */
    FK_LEFT, /* Flipper LEFT  -> keyfob button 2 (OK) */
    FK_RIGHT, /* Flipper RIGHT -> keyfob button 1 (UP) */
    FK_BACK, /* Flipper BACK  -> keyfob button 3 (DOWN) */
    FK_COUNT,
};

/* Maps a tracked Flipper key (FK_*) to the keyfob logical button (B_*). */
static const uint8_t FK_TO_BTN[FK_COUNT] = {
    [FK_UP] = B6, /* Flipper UP    -> keyfob 6 */
    [FK_OK] = B5, /* Flipper OK    -> keyfob 5 */
    [FK_DOWN] = B_BACK, /* Flipper DOWN  -> keyfob 4 */
    [FK_LEFT] = B_OK, /* Flipper LEFT  -> keyfob 2 */
    [FK_RIGHT] = B_UP, /* Flipper RIGHT -> keyfob 1 */
    [FK_BACK] = B_DOWN, /* Flipper BACK  -> keyfob 3 */
};

/* Maps a tracked Flipper key (FK_*) to its KBIT_* mask bit. */
static const uint8_t FK_TO_KBIT[FK_COUNT] = {
    [FK_UP] = KBIT_UP,
    [FK_OK] = KBIT_OK,
    [FK_DOWN] = KBIT_DOWN,
    [FK_LEFT] = KBIT_LEFT,
    [FK_RIGHT] = KBIT_RIGHT,
    [FK_BACK] = KBIT_BACK,
};

#define SPI_RX_QUEUE_LEN 64

/* --- CC1101 bridge (adapted from PandoraPIC) --- */
/* TX: the ARM firmware enables rf_tx_on (Si4432 REG[0x07]=0x08) and bit-bangs
 * the DATA pin (GPIO DOUT port B bit0). We capture each edge with its duration
 * (measured by ninsn between toggles) in a LevelDuration ring and replay it
 * through furi_hal_subghz_start_async_tx.
 *
 * MEMORY: 512 edges (about 4KB) are reserved for a captured frame while keeping
 * the flash cache from losing several pages to an oversized TX buffer. */
#define TX_RING_LEN 512u
/* RX: the CC1101 capture ISR writes edges to a single-producer/single-consumer
 * ring; the emulator thread feeds them to the firmware demodulator. */
#define RX_RING_MAX_LEN 2048u
/* The replayed duration is still passed unchanged to the emulated TIMER1/ISR.
 * This is only a host-side scheduling batch: a decoded frame breaks the batch
 * immediately so the firmware foreground handler gets to consume it. Eight
 * edges per UI iteration made an SD spool take minutes to replay. */
#define RX_PLAY_MAX_EDGES_PER_PUMP 256u
#define RX_ISR_SAMPLES_PER_EDGE 6u /* conservative fallback when taps are unknown */
#define RX_PREROLL_LEN 256u /* retain roughly 60-120 ms before RSSI trigger */
#define RX_SPOOL_SLOTS 2u
#define RX_SPOOL_BATCH 512u
#define RX_SPOOL_MAX_BYTES (256u * 1024u)
#define RX_POSTROLL_MS 100u
#define RX_RSSI_POLL_MS 20u
#define RX_RSSI_MIN_OPEN_DBM (-95.0f)
#define RX_RSSI_NOISE_MARGIN_DB 6.0f
#define RX_RSSI_HYSTERESIS_DB 3.0f
#define RX_RSSI_HANG_MS 300u
#define RX_HEARTBEAT_MS 5000u

/* ninsn->us conversion for the bit-bang TX. The keyfob is not real-time under
 * the interpreter; the scale is approximate and only affects the replayed
 * cadence, not the logical content of the frame. 1 insn ~= 1 us as an
 * adjustable base. */
#define ARM_INSN_US_NUM 1u
#define ARM_INSN_US_DEN 1u

/* --- buzzer detection / tentative beep tuning ---------------------------------
 * These are heuristics: the buzzer mechanism is [DESC] (unknown from the RE), so
 * the goal of the first version is to DETECT + LOG a candidate and emit a safe,
 * non-blocking tentative beep. Tune once hardware logs confirm the real pin. */
#define BUZZ_MIN_EDGES 4u /* toggles before we treat a pin as a buzzer candidate */
#define BUZZ_GAP_INSN 20000u /* if a candidate is idle this many insn, reset it */
#define BUZZ_TONE_MIN_HZ 100.0f /* Flipper speaker lower bound (coin buzzer) */
#define BUZZ_TONE_MAX_HZ 2500.0f /* Flipper speaker upper bound */
#define BUZZ_TONE_DEFAULT_HZ 2000.0f /* fallback when the period is unreliable */
#define BUZZ_VOLUME 0.5f /* tentative volume (0..1) */
#define BUZZ_HOLD_MS 60u /* keep the tone up this long after the last edge */
#define SPK_ACQUIRE_TIMEOUT 0u /* non-blocking speaker acquire (do not stall) */

/* RF bridge modes of the overlay (Left cycles). */
typedef enum {
    RfModeOff = 0,
    RfModeTx,
    RfModeRx,
} RfMode;

/* Which CC1101 modulation preset is currently loaded. Tracked so we only reload
 * a custom preset when the modulation actually changes (reloading is a bus
 * transaction and must happen on an idle radio). */
typedef enum {
    CcPresetOok = 0, /* OOK/ASK 650kHz BW (Pandora common / OOK remotes) */
    CcPreset2Fsk, /* 2-FSK, moderate deviation (Pandora FSK/GFSK firmwares) */
} CcPreset;

/* Si4432 data-source mode (REG[0x71] dtmod, bits[7:6]). Tracked for DIAGNOSTIC
 * logging only. DYN_HW_D605.md proved both firmwares use DIRECT mode (dtmod=0,
 * 0 FIFO accesses), so the runtime bridge path is ALWAYS direct bit-bang; the
 * FIFO path was removed. We still decode dtmod so the log reveals if a future
 * firmware ever programs FIFO (dtmod=2) -- it would print a warning rather than
 * silently taking a wrong path.
 *   00 direct GPIO  -> bit-bang on the DATA pin (both firmwares)
 *   01 direct SDI   -> direct (bit-bang)
 *   10 FIFO         -> packet mode (NOT used by any known firmware)
 *   11 PN9          -> test pattern */
typedef enum {
    DtModDirectGpio = 0,
    DtModDirectSdi = 1,
    DtModFifo = 2,
    DtModPn9 = 3,
} SiDtMod;

typedef struct {
    bool level;
    uint32_t duration; /* us */
} RfEdge;

/* RX durations are ultimately loaded into a 16-bit timer (duration * 6).
 * Keeping the low 16 bits is exact for that peripheral and halves ring memory. */
typedef struct {
    uint16_t duration;
    uint8_t level;
    uint8_t gate_open; /* snapshot at capture time; padding-free on 32-bit ABI */
} RxEdge;

typedef enum {
    RxSpoolFree = 0,
    RxSpoolWriting,
    RxSpoolReady,
    RxSpoolPlaying,
    RxSpoolFailed,
} RxSpoolState;

typedef struct {
    File* file;
    RxSpoolState state;
    uint32_t bytes;
    uint32_t play_pos;
    uint32_t order;
    uint32_t write_errors;
    uint32_t drop_start;
    char path[48];
} RxSpool;

/* ---------------------------------------------------------------- FlashCache
 * Paged LRU cache of the emulated firmware flash. Each slot is an independent
 * 4KB safe_malloc (small blocks fit a fragmented heap; no single big block).
 * Pages are streamed from a canonical flat temp .bin on the SD. Faithful port
 * of FlipperGB's RomCache (slots/slot_page/slot_use + rc_fill LRU). */
typedef struct {
    File* file; /* flat .bin on SD (flash_used bytes, gaps=0xFF); kept open */
    uint32_t flash_used; /* logical flash size (bytes of real code) */
    uint32_t num_pages; /* ceil(flash_used / FLASH_PAGE_SIZE) */
    uint8_t** slots; /* num_slots pointers to 4KB blocks */
    uint32_t* slot_page; /* which flash page each slot holds (0xFFFFFFFF=empty) */
    uint32_t* slot_use; /* LRU stamps */
    uint32_t use_counter;
    uint32_t miss_count; /* diagnostics: SD page misses */
    uint16_t num_slots;
    bool fully_resident; /* every page has a slot -> O(1) index, no eviction */
} FlashCache;

typedef struct {
    ThumbCore* cpu;
    const ArmProfile* prof;
    FlashCache fc; /* paged flash cache (replaces the old contiguous buffer) */
    uint8_t* ram; /* RAM buffer 32KB */

    Si4432 si;
    /* SPI RX queue (Si4432 responses) */
    uint8_t spi_rx[SPI_RX_QUEUE_LEN];
    uint8_t spi_rx_head, spi_rx_tail;
    int spi_phase; /* -1 idle, 0 pending-after-addr */
    uint8_t spi_kind; /* 'w'/'r' */
    uint8_t spi_reg;

    /* GPIO */
    uint16_t gpio_din[16]; /* din per port (bit=1 released) */
    uint16_t gpio_dout[16];

    /* --- indicator LED (keyfob LED -> Flipper red LED) ---
     * The keyfob drives an indicator LED on a GPIO pin (pandora: port3 pin14,
     * bit 0x4000; see RE_DYN_pandora.md). On every GPIO DOUT write we read the
     * FINAL state of that pin and mirror it to the Flipper's red LED via
     * furi_hal_light_set (cheap, low latency). led_on caches the last state so we
     * only call the HAL on a real change (no spam in the blink loop). */
    bool led_on; /* last LED state pushed to the Flipper (true = lit) */
    bool led_logged; /* 1 after the first LED toggle has been logged (throttle) */

    /* --- buzzer detection / tentative beep ---
     * The keyfob's buzzer pin/register is [DESC] (unknown from the RE). We do two
     * things here, both best-effort:
     *   (A) DETECT: log candidate mechanisms so a human can identify the real
     *       buzzer on hardware. Two patterns are watched:
     *         - a GPIO pin (NOT the LED / OLED-DC / Si4432-DATA / buttons) that
     *           toggles repeatedly in a short window (square-wave => audible tone);
     *         - TIMER writes to compare/CC or output-enable sub-registers that are
     *           outside our delay/CNT/TOP model (a PWM tone generator).
     *   (B) TENTATIVE BEEP: when a square-wave GPIO toggle burst is detected we
     *       estimate its frequency from the toggle period (ninsn -> us) and request
     *       a short non-blocking beep on the Flipper speaker. If we cannot derive a
     *       reliable frequency we fall back to a fixed tone. The speaker is a shared
     *       resource: we acquire it lazily, start the tone, and stop/release it
     *       from the hot loop once the activity ceases (buzz_deadline_tick). We
     *       NEVER block the emulator loop while the speaker is held. */
    uint8_t buzz_cand_port; /* port of the current candidate buzzer pin (0xFF=none) */
    uint8_t buzz_cand_pin; /* pin of the current candidate buzzer pin */
    uint32_t buzz_edges; /* toggle count seen for the current candidate */
    uint64_t buzz_last_edge_insn; /* ninsn at the previous candidate toggle */
    uint64_t buzz_period_insn; /* latest measured toggle half-period (insn) */
    bool buzz_logged; /* 1 after the first "BUZZER? gpio..." log (throttle) */
    bool buzz_timer_logged; /* 1 after the first "BUZZER? timer..." log (throttle) */

    /* Flipper speaker ownership + tentative-beep pacing (driven by the hot loop) */
    bool spk_owned; /* true while we hold furi_hal_speaker (acquired) */
    bool spk_playing; /* true while a tone is actively started */
    float spk_freq; /* latest REQUESTED tone frequency (Hz, set by mmio_write) */
    float spk_freq_playing; /* tone frequency currently started on the speaker */
    uint32_t buzz_active_tick; /* furi_get_tick() of the last detected buzzer edge */
    bool buzz_request; /* set by mmio_write when buzzer activity is detected;
                        * consumed by the hot loop to start/refresh the beep */

    uint16_t timer_cnt[4]; /* legacy (kept for the RX feed path) */
    /* Faithful EFM32 timer model (CORE fix: eliminates UI "stutter"). CNT is a
     * free-running counter that advances with TIME (modeled by instructions
     * executed), NOT per read: CNT = ((ninsn - t0) * rate) mod (TOP+1). The old
     * "+0x2000 per read" made delay() loops take a bimodal 1-or-7 reads ->
     * irregular pacing. This makes each delay() consume a CONSISTENT number of
     * instructions -> smooth pacing. */
    uint64_t timer_t0[4]; /* ninsn at the last CNT reset (write) */
    uint32_t timer_top[4]; /* TOP configured by the firmware (wrap) */
    uint32_t timer_rate[4]; /* CNT advance per instruction (HFPERCLK proxy) */

    /* --- AGNOSTIC NVIC (driven by the firmware's REAL ISER/ICER) --- */
    uint8_t in_irq;
    uint8_t irq_rr;
    /* Pending UI tick request. The reference (Unicorn pandora_tui) raises a
     * _pending_tick after EVERY run_burst from its button/boot drivers, forcing
     * one timer IRQ tick at the start of the next burst IN ADDITION to the
     * periodic one. That extra per-burst tick gives the firmware's UI TIMER IRQ
     * (IRQ13) its correct cadence; without it this core fired IRQ13 ~30% less
     * often than Unicorn, so post-unlock the firmware's screen state machine
     * stayed in the 3-scanner UI loop and navigation did nothing. */
    uint8_t pending_tick;
    uint32_t tick_count;
    uint64_t last_tick_insn;
    uint32_t wfi_pc_hint; /* first WFI addr (compat / fast stuck-WFI check) */
    /* Mask of IRQs the firmware has ENABLED right now (NVIC ISER set / ICER
     * clear). 64 bits so IRQ numbers up to ~63 are covered (this MCU uses up to
     * ~IRQ36). Bit N = IRQ N. Ported from pandora_tui.py _nvic_enabled and the
     * ESP32 port. The old NVIC ignored the ISER entirely (bug). */
    uint64_t nvic_enabled;
    /* SCB->VTOR (0xE000ED08) the firmware installs at runtime. Stored here
     * because the PPB has no backing memory; cur_vtor reads it. Agnostic. */
    uint32_t ppb_vtor;
    /* WFI/WFE skip set: absolute addresses of WFI(0xBF30)/WFE(0xBF20) opcodes,
     * scanned agnostically from flash (from 0x40) and -- post-boot -- RAM. Not a
     * per-firmware address. Ported from the ESP32 wfi_set / _wfi_set. */
    uint32_t* wfi_set;
    uint32_t wfi_count;
    uint32_t wfi_cap;
    uint8_t wfi_ram_scanned; /* RAM rescanned once after boot (app-in-RAM case) */

    uint8_t unlocked; /* OBSERVABLE: set when the framebuffer has lit pixels */
    int max_step; /* inert, kept for struct/wire compat */

    /* knock-code (FIRMWARE CONFIG, set by the host/UI). Pressed PURELY over GPIO
     * as an autotest at reset (no PC-sync, no scanner-address). Agnostic.
     * Ported from the ESP32 emu_set_knock. */
    const uint8_t* knock_seq; /* active sequence (points to knock_user or a default) */
    uint8_t knock_len;        /* length of the active sequence */
    int knock_gate;           /* B_* index of the final held gate, or -1 */

    /* buttons (logical mask) */
    uint8_t btn_pressed[BTN_COUNT];

    /* --- OLED double buffering (fix: flicker / overwrite on screen) ---
     * Faithful port of emu/pandora_tui.py (_oled_cmd/_oled_data/_oled_commit,
     * _oled_work/_oled_fb, the 0xAF handling and the blank-frame skip).
     *
     * ROOT CAUSE (DISPLAY_DIAG.md / OLED_FIDELITY.md): render_oled used to read
     * the firmware framebuffer DIRECTLY from RAM every UI frame. The firmware
     * composes that framebuffer byte-by-byte during oled_flush @0xa550, which
     * FIRST calls an internal CLEAR (sub_9fc8 @0xa554: 1472 bytes 0x00 = blank
     * screen) and THEN paints the real content. A per-frame RAM read could catch
     * a PARTIAL frame (mid-write) or a fully BLANK frame (just after the clear)
     * -> flicker / overwrite / blink-to-black.
     *
     * FIX (Option A, same as the Python reference): intercept the OLED SPI bus in
     * mmio_write -> spi_byte. The DC pin (port0 bit9, A9) selects command vs
     * pixel data (oled_write_cmd @0xa5c8 sets A9 HIGH for a command, LOW for the
     * 92 pixel bytes). We rebuild the frame in a WORK buffer and COMMIT it
     * atomically to a VISIBLE buffer ONLY on the CONTENT 0xAF (display ON =
     * end-of-flush), SKIPPING the commit if the work buffer is all-zero (that is
     * the internal clear's 0xAF -> would be a black frame). render_oled reads the
     * VISIBLE buffer, so it never sees a partial or black frame. */
    uint8_t oled_fb[OLED_FB_SIZE]; /* VISIBLE buffer (what render_oled reads) */
    uint8_t oled_work[OLED_FB_SIZE]; /* WORK buffer (written byte-by-byte on flush) */
    uint8_t oled_page; /* current page pointer (set-page command) */
    uint8_t oled_col; /* current column pointer (auto-increments on data) */
    uint8_t oled_work_dirty; /* 1 if data was written in this pass */
    uint32_t oled_frames; /* visible framebuffer changes committed (diagnostic) */

    /* Flipper framebuffer */
    uint8_t screen[FB_SIZE];
    FuriMutex* fb_mutex;
    Gui* gui;
    Canvas* canvas;

    bool radio_on;
    uint32_t frequency;

    /* --- CC1101 RF bridge (adapted from PandoraPIC) --- */
    RfMode rf_mode; /* OFF / TX / RX */

    /* TX bridge: ring of edges captured from the firmware DATA pin */
    RfEdge tx_ring[TX_RING_LEN];
    volatile uint32_t tx_head; /* producer (main loop / mmio_write) */
    volatile uint32_t tx_tail; /* consumer (async_tx callback) */
    bool tx_capturing; /* the firmware is in TX mode (rf_tx_on) */
    bool tx_last_level; /* last DATA pin level */
    uint64_t tx_last_insn; /* ninsn at the last edge */
    bool tx_active; /* async_tx in progress */
    uint32_t tx_frames; /* counter of logged TX frames */
    uint32_t tx_edges_frame; /* accumulated edges of the current frame */
    uint32_t tx_edges_dropped; /* waveform intervals lost to a full ring */

    /* RX bridge: single-producer (CC1101 capture ISR), single-consumer ring. */
    uint8_t* rx_ring_storage;
    uint16_t* rx_ring_duration;
    uint8_t* rx_ring_flags; /* 2 packed bits per edge: level, RSSI gate */
    uint16_t rx_ring_capacity; /* may degrade under heap pressure; spool retains bursts */
    volatile uint32_t rx_head; /* ISR producer sequence */
    volatile uint32_t rx_tail; /* emulator consumer sequence */
    bool rx_running; /* async_rx active */
    bool rx_primed; /* receiver RAM state prepared (prime_rx) */
    bool rx_reprime_pending; /* frame-ready flag awaits foreground firmware handling */
    bool rx_prev_level; /* previous level for the transition feed */
    bool rx_feeding; /* 1 while the demod ISR runs (TIMER1=pulse_ticks) */
    uint32_t rx_pulse_ticks; /* TIMER1_CNT to present during the feed (dur*6) */
    uint32_t rx_events; /* raw edges delivered by the CC1101 callback */
    uint32_t rx_injected; /* edges actually delivered to the emulated demod ISR */
    uint32_t rx_dropped; /* edges rejected because the RX queue was full */
    uint32_t rx_gate_drop_snapshot; /* rx_dropped when the RSSI gate opened */
    uint32_t rx_frames; /* frames observed (OLED redrew after an RX inject) */
    bool rx_last_frame_valid; /* suppress repeated identical frame log lines */
    uint8_t rx_last_frame_id;
    /* NOTE: the per-firmware RX trace fields (rx_last_frame[12] raw buffer,
     * rx_addr_trace_*, rx_drawn_text[48]) were REMOVED with the agnostic RX port
     * -- they read firmware code/RAM addresses (draw_char @0xA060, dispatch
     * @0xD16C, rx_result RAM) which no longer exist. Saves ~62 B of AppState. */
    RxEdge rx_preroll[RX_PREROLL_LEN]; /* recent raw edges before RSSI gate opens */
    volatile uint16_t rx_preroll_head;
    volatile uint16_t rx_preroll_count;
    volatile uint8_t rx_preroll_frozen;
    volatile uint32_t rx_postroll_isr_us;
    uint32_t rx_preroll_overwritten;
    uint32_t rx_idle_edges;
    uint32_t rx_gate_open_count;
    uint32_t rx_gate_close_count;
    RxSpool rx_spool[RX_SPOOL_SLOTS];
    /* Writer and player share this 2 KiB buffer. Before a new capture writes,
     * playback is rewound to its first undelivered edge and paused. */
    RxEdge rx_spool_buffer[RX_SPOOL_BATCH];
    uint16_t rx_spool_batch_count;
    uint16_t rx_spool_play_count;
    uint16_t rx_spool_play_index;
    int8_t rx_spool_writer;
    int8_t rx_spool_player;
    uint32_t rx_spool_order;
    uint32_t rx_spool_bursts;
    uint32_t rx_spool_replayed;
    uint32_t rx_spool_lost;
    uint16_t rx_gate_sample_duration[8];
    uint8_t rx_gate_sample_level[8];
    uint8_t rx_gate_sample_count;
    uint32_t rx_capture_post_remaining_us;
    bool rx_capture_active;
    bool rx_capture_blocked;
    volatile uint8_t rx_gate_open_isr; /* sampled by capture IRQ */
    float rx_rssi_dbm;
    float rx_noise_floor_dbm;
    float rx_gate_open_dbm;
    bool rx_rssi_initialized;
    uint32_t rx_rssi_sample_tick;
    uint32_t rx_gate_below_tick;

    /* --- RF "follow firmware" bridge (Si4432 -> CC1101 auto mapping) ---
     * When rf_follow is true (the default), the CC1101 mode/frequency track what
     * the firmware programs into its Si4432: REG[0x07] drives TX/RX/IDLE and
     * REG[0x75..0x77] drive the carrier frequency. The manual hold-BACK cycling
     * still works as a diagnostic override (it flips rf_follow off). */
    bool rf_follow; /* true: CC1101 mode/freq follow the firmware's Si4432 */
    uint32_t rx_events_hb; /* rx_events snapshot for the RX heartbeat delta */
    uint32_t rx_dropped_hb; /* rx_dropped snapshot for the RX heartbeat delta */
    uint32_t rx_frames_hb; /* rx_frames snapshot for the RX heartbeat delta */
    uint32_t rx_gate_open_hb;
    uint32_t rx_gate_close_hb;
    uint8_t si_modtyp; /* last REG[0x71] modtyp[1:0] seen (0=unmod 1=OOK 2=FSK 3=GFSK) */
    uint8_t si_dtmod; /* last REG[0x71] dtmod[7:6] (SiDtMod); DIAGNOSTIC only */
    uint32_t si_devi_hz; /* FSK frequency deviation derived from REG[0x71]/0x72 */
    CcPreset cc_preset; /* which CC1101 custom preset is loaded right now */
    uint8_t cc_fsk_deviation_reg; /* selected CC1101 DEVIATN value */

    /* NOTE: the old Si4432 PACKET/FIFO bridge (si_tx_fifo / si_rx_fifo and the
     * fifo_tx_transmit / fifo_rx_poll functions) was REMOVED. DYN_HW_D605.md
     * proved both firmwares use DIRECT mode with 0 accesses to REG[0x7F], so the
     * FIFO path never fired on real firmware and only cost RAM. The bridge is now
     * DIRECT-ONLY: RX injects into PB0 (demod ISR), TX captures PB3 (bit-bang). */

    volatile bool exit_requested;
    volatile bool menu_requested;
    bool menu_active;

    uint64_t ninsn;
    uint32_t pc;
    uint32_t next_pc;

    /* AGNOSTIC: effective flash size, DERIVED from the loaded firmware image
     * (highest address written / file size, page-aligned), NOT a per-firmware
     * ArmProfile.flash_used constant. Set by flatten_firmware, consumed by
     * fc_init. Mirrors the ESP32 port's emu_init flash sizing. */
    uint32_t flash_used;

    /* --- user-defined knock-code (menu) --- */
    uint8_t knock_user[16]; /* button sequence (B_* indices) */
    uint8_t knock_user_len; /* 0 = use the profile default */

    /* --- phase 1 menu (view_dispatcher) --- */
    ViewDispatcher* view_dispatcher;
    Submenu* submenu;
    Widget* about_widget; /* About screen (scrollable text) */
    FuriString* fw_path; /* chosen firmware path */
    char fw_name[32]; /* short name for the label */

    Storage* storage; /* kept for the flash cache (streams pages at runtime) */

    /* --- GPIO EXTI (external interrupt) delivery ---------------------------
     * Both firmwares read the nav buttons via the EFM32 GPIO external interrupt
     * (GPIO_EVEN=IRQ1 even pins, GPIO_ODD=IRQ11 odd pins), NOT by polling ports
     * 2/5. The post-unlock UI loop only polls port 0 directly; the OK/UP/DOWN
     * edges reach the firmware through its GPIO ISR. The core must raise the
     * GPIO IRQ on a real button pin edge, or those presses are lost (erratic
     * navigation). gpio_if = latched EXTI IF (read at 0x40006114); pending_gpio_*
     * ask the scheduler to enter the GPIO ISR. Agnostic: driven by the observed
     * button pin edge + the firmware's own IEN/ISER. */
    uint16_t gpio_if;
    uint8_t pending_gpio_even;
    uint8_t pending_gpio_odd;
} AppState;

/* ThumbCore counts each completed instruction. Treat the instruction currently
 * executing as +1 so MMIO callbacks see a consistent virtual timestamp whether
 * the per-instruction hook is enabled or bypassed. */
static uint64_t emu_insn_now(const AppState* app) {
    if(!app->cpu) return app->ninsn;
    return app->cpu->cycles + 1u;
}

static volatile uint32_t s_fb_cb_inflight = 0;

static void wait_inflight_zero(volatile uint32_t* counter) {
    while(__atomic_load_n(counter, __ATOMIC_ACQUIRE) != 0) {
        furi_delay_ms(1);
    }
}

/* RF bridge fwd decls (used from the MMIO callbacks) */
static void tx_on_si_mode(AppState* app, int new_mode);
static void tx_capture_data_edge(AppState* app, bool level);
/* Si4432 -> CC1101 "follow firmware" bridge (used from spi_byte). */
static void rf_follow_apply_mode(AppState* app, int new_mode);
static void rf_follow_apply_freq(AppState* app);
static void rx_bridge_start(AppState* app);
static void rx_bridge_stop(AppState* app);
static void radio_rx_callback(bool level, uint32_t duration, void* context);

/* Flash cache fwd decl (used from the MMIO read callback; defined with the rest
 * of the FlashCache code in the load+boot section). */
static uint32_t fc_read(FlashCache* fc, uint32_t addr, int size);
static const uint8_t* fc_page_ptr(FlashCache* fc, uint32_t page);

/* ------------------------------------------------------------ buttons */

static void btn_port_pin(const ArmProfile* p, int btn, uint8_t* port, uint8_t* pin) {
    switch(btn) {
    case B_UP: *port = p->up_port; *pin = p->up_pin; break;
    case B_DOWN: *port = p->down_port; *pin = p->down_pin; break;
    case B_OK: *port = p->ok_port; *pin = p->ok_pin; break;
    case B_BACK: *port = p->back_port; *pin = p->back_pin; break;
    case B5: *port = p->b5_port; *pin = p->b5_pin; break;
    case B6: *port = p->b6_port; *pin = p->b6_pin; break;
    default: *port = 0; *pin = 0; break;
    }
}

static void set_button(AppState* app, int btn, bool pressed) {
    if(btn < 0 || btn >= BTN_COUNT) return;
    uint8_t port, pin;
    btn_port_pin(app->prof, btn, &port, &pin);
    bool was = ((app->gpio_din[port] >> pin) & 1u) == 0u; /* prev pressed? */
    if(pressed)
        app->gpio_din[port] &= (uint16_t)~(1u << pin); /* 0 = pressed */
    else
        app->gpio_din[port] |= (uint16_t)(1u << pin); /* 1 = released */
    app->btn_pressed[btn] = pressed ? 1 : 0;

    /* GPIO button-edge -> GPIO IRQ, delivered ONLY on the RELEASE edge.
     *
     * MEASURED FACT (root-cause fix validated on the G82 against both firmwares
     * and the Unicorn oracle): NEITHER Pandora firmware uses the EFM32 GPIO EXTI
     * path for the nav buttons -- it NEVER writes GPIO_IEN (0x40006110) and NEVER
     * reads the EXTI IF register (0x40006114); it scans the button DIN ports by
     * polling. Raising the GPIO IRQ on BOTH edges made the firmware process the
     * key while the pin was still held (press edge), double/mis-detecting and
     * giving erratic navigation (pandora 4/10). A real keyfob commits a short
     * click on RELEASE, not on first contact. Nudging the scanner on the RELEASE
     * edge ONLY reproduces the oracle exactly (pandora 7/10, pmax 6/10,
     * deterministic 3/3) -- usable, hardware-faithful navigation. The latch +
     * parity IRQ are kept (gated by the fw's own ISER at dispatch) so a firmware
     * that DID enable EXTI would still be served. */
    if(was != pressed && !pressed) { /* RELEASE edge only */
        app->gpio_if |= (uint16_t)(1u << pin);
        if(pin & 1u) app->pending_gpio_odd = 1;
        else         app->pending_gpio_even = 1;
    }
}

static void init_buttons_released(AppState* app) {
    for(int i = 0; i < 16; i++) app->gpio_din[i] = 0xFFFF; /* pull-up everything released */
    for(int b = 0; b < BTN_COUNT; b++) app->btn_pressed[b] = 0;
}

/* ------------------------------------------------------------ MMIO (callbacks) */

static uint32_t mmio_read(void* ctx, uint32_t addr, int size) {
    AppState* app = (AppState*)ctx;
    uint32_t base = addr & 0xFFFFF000u;
    uint32_t off = addr & 0xFFFu;
    uint32_t v = 0;
    /* EMULATED FLASH (0x00000000..0x00042000). The flash is NOT a direct region
     * anymore (flash_ptr=NULL), so EVERY flash read lands here and is resolved
     * through the paged LRU cache. In practice this is cold-path: the firmware is
     * RAM-resident (fetch hits the direct RAM region), only the few flash CONSTANT
     * loads reach this. Bytes at/above flash_used are open-bus 0xFF (handled
     * inside fc_read, without polluting the cache). */
    if(addr < 0x42000u) {
        return fc_read(&app->fc, addr, size);
    }
    if(base == 0x4000C000u) { /* USART0 (SPI Si4432) */
        if(off == 0x10) {
            v = (1u << 6) | (1u << 5); /* TXBL + RXDATAV ready */
        } else if(off == 0x1C) { /* RXDATA */
            if(app->spi_rx_head != app->spi_rx_tail) {
                v = app->spi_rx[app->spi_rx_head];
                app->spi_rx_head = (uint8_t)((app->spi_rx_head + 1) % SPI_RX_QUEUE_LEN);
            }
        }
    } else if(base == 0x400C4000u) { /* 2nd USART (EEPROM/config) */
        if(off == 0x10) v = (1u << 6) | (1u << 5);
    } else if(addr == 0x40006114u) { /* GPIO EXTI IF (interrupt flags) */
        v = app->gpio_if;
    } else if(addr >= 0x40006000u && addr < 0x40006000u + 36u * 16u) { /* GPIO */
        uint32_t port = (addr - 0x40006000u) / 36u;
        uint32_t poff = (addr - 0x40006000u) - port * 36u;
        if(poff == 0x1C && port < 16) v = app->gpio_din[port]; /* DIN */
    } else if(base == 0x40002000u) { /* ADC0 (pmax reads the battery) */
        if(off == 0x08)
            v = (1u << 16); /* SINGLEDV ready */
        else if(off == 0x24)
            v = 0x0C00; /* data ~3.9V */
    } else if(base == 0x400C8000u) { /* CMU #2 */
        /* FIX (CORE_DIAG P1): firmware busy-waits on bits 3/7/9 of 0x400C802C
         * (clock-ready). If we return 0 it spins forever (~30% CPU). Return only
         * those bits set (0x288) so the wait exits without flipping other bits
         * (0xFFFFFFFF broke pmax's boot). */
        v = (off == 0x2Cu) ? ((1u << 3) | (1u << 7) | (1u << 9)) : 0u;
    } else if(base == 0x40080000u) { /* CMU #1 */
        v = 0;
    } else if(base == 0x40010000u) { /* TIMER0..3 */
        uint32_t tn = off >> 10;
        uint32_t sub = off & 0x3FFu;
        if(sub == 0x24 && tn < 4) {
            /* During the RX feed the demod ISR reads TIMER1.CNT as the WIDTH of
             * the half-period (see arm_pandora.py: TIMER1_CNT = dur_us*6). */
            if(tn == 1 && app->rx_feeding) {
                v = app->rx_pulse_ticks & 0xFFFFu;
            } else {
                /* Faithful CNT: advances with time (instructions), wraps at TOP.
                 * CONSISTENT per-delay cost -> smooth pacing (no stutter). */
                uint32_t top = app->timer_top[tn] ? app->timer_top[tn] : 0xFFFFu;
                uint64_t elapsed =
                    (emu_insn_now(app) - app->timer_t0[tn]) * (uint64_t)app->timer_rate[tn];
                v = (uint32_t)(elapsed % ((uint64_t)top + 1u));
            }
        } else if(sub == 0x3C && tn < 4) { /* TOP reached? (IF overflow proxy) */
            uint32_t top = app->timer_top[tn] ? app->timer_top[tn] : 0xFFFFu;
            uint64_t elapsed =
                (emu_insn_now(app) - app->timer_t0[tn]) * (uint64_t)app->timer_rate[tn];
            v = (elapsed >= top) ? 1u : 0u;
        }
    } else if(addr == 0xE000ED08u) {
        /* SCB->VTOR read-back: the PPB has no backing memory in the core, so we
         * return the value the firmware last installed (stored in ppb_vtor).
         * Agnostic (ported from the ESP32 port). */
        v = app->ppb_vtor;
    }
    (void)size;
    return v;
}

/* ------------------------------------------------------------ OLED capture (SPI)
 * Double-buffered capture of the keyfob OLED (UC16xx/ST7528) off the SPI bus.
 * Faithful port of emu/pandora_tui.py (_dc_level/_oled_cmd/_oled_data/
 * _oled_commit). The DC pin is port0 bit9 (A9); the flush addresses by raw page
 * number (0x00..0x0F), resets the column to 0 with 0x60/0x70, then streams 92
 * data bytes/page, ending each pass with 0xAF (display ON). */

/* OLED DC pin: port0 bit9 (A9). HIGH=command, LOW=pixel data. */
#define OLED_DC_PORT 0u
#define OLED_DC_PIN 9u

static int oled_dc_level(AppState* app) {
    return (app->gpio_dout[OLED_DC_PORT] >> OLED_DC_PIN) & 1u;
}

/* Atomic commit: copy the WORK buffer (a full freshly-flushed frame) to the
 * VISIBLE buffer. Faithful port of _oled_commit. Skips the commit if:
 *   - nothing was written this pass (not dirty), or
 *   - the frame is entirely blank (all 0x00). The internal clear (sub_9fc8)
 *     leaves the work buffer at zero and emits its OWN 0xAF; committing that
 *     would make a BLACK frame visible (flicker). The real content of the flush
 *     fills the SAME work buffer immediately after and commits on ITS 0xAF.
 * Everything runs on the single emulator thread (run_emulator), so no lock is
 * needed between this commit and render_oled (both main-thread); the Flipper GUI
 * callback reads app->screen, not these buffers. */
static void oled_commit(AppState* app) {
    if(!app->oled_work_dirty) return;
    bool any = false;
    uint16_t nonzero = 0;
    for(int i = 0; i < OLED_FB_SIZE; i++) {
        if(app->oled_work[i]) {
            any = true;
            nonzero++;
        }
    }
    if(!any) {
        /* blank frame = internal clear of the flush; do not make it visible.
         * (keep dirty: the real body reuses the same work buffer and will commit
         *  the content on its own 0xAF.) */
        if(app->rx_reprime_pending)
            FURI_LOG_D(TAG, "RX OLED blank flush skipped (#%lu)",
                       (unsigned long)app->oled_frames);
        return;
    }
    /* Firmware often flushes an unchanged status frame on periodic ticks.
     * Keep the currently displayed image and avoid a full Flipper-side
     * conversion/GUI commit when the 2bpp source did not actually change. */
    if(memcmp(app->oled_fb, app->oled_work, OLED_FB_SIZE) == 0) {
        app->oled_work_dirty = 0;
        return;
    }
    memcpy(app->oled_fb, app->oled_work, OLED_FB_SIZE);
    app->oled_work_dirty = 0;
    app->oled_frames++;
    /* AGNOSTIC RX commit diagnostic: an observable framebuffer bbox + nonzero,
     * with NO firmware RAM address (the old code read profile rx_valid/rx_brand/
     * ui_* offsets). When a frame was just RX-decoded (rx_reprime_pending) log
     * the visible change so a human can confirm the firmware redrew. */
    if(app->rx_reprime_pending) {
        uint16_t gray[4] = {0, 0, 0, 0};
        int min_x = OLED_VISIBLE_COLS, min_y = OLED_PAGES * 4;
        int max_x = -1, max_y = -1;
        for(int page = 0; page < OLED_PAGES; page++) {
            for(int col = 0; col < OLED_VISIBLE_COLS; col++) {
                uint8_t b = app->oled_work[page * OLED_COLS + col];
                for(int k = 0; k < 4; k++) {
                    uint8_t level = (uint8_t)(((b >> (2 * k)) & 1u) |
                                              (((b >> (2 * k + 1)) & 1u) << 1));
                    gray[level]++;
                    if(level >= 2) {
                        int y = page * 4 + k;
                        if(col < min_x) min_x = col;
                        if(col > max_x) max_x = col;
                        if(y < min_y) min_y = y;
                        if(y > max_y) max_y = y;
                    }
                }
            }
        }
        FURI_LOG_I(TAG,
                   "RX OLED commit #%lu nonzero=%u gray=%u/%u/%u/%u bbox=%d,%d-%d,%d",
                   (unsigned long)app->oled_frames, (unsigned)nonzero,
                   (unsigned)gray[0], (unsigned)gray[1], (unsigned)gray[2],
                   (unsigned)gray[3], min_x, min_y, max_x, max_y);
    }
}

/* Processes one OLED controller command (set page / column / display-on).
 * Faithful port of _oled_cmd. */
static void oled_cmd(AppState* app, uint8_t b) {
    uint8_t c = b;
    if(c < OLED_PAGES) { /* 0x00..0x0F = raw page number */
        app->oled_page = c;
        app->oled_col = 0;
    } else if(c == 0x60 || c == 0x70) { /* set column address -> column 0 */
        app->oled_col = 0;
    } else if(c == 0xAF) { /* display ON = end of a flush pass -> commit */
        oled_commit(app);
    }
    /* other commands (init table 0xD2.., 0x60/0x70 handled, 0x81, etc.): ignored
     * for the page/col pointer. */
}

/* Writes one pixel byte into the WORK buffer (not the visible buffer). Faithful
 * port of _oled_data. Layout: fb[page*cols + col], byte = 8 vertical pixels,
 * bit0 = top row. The column auto-increments; the flush emits an explicit
 * set-page per page, so we clamp to avoid overflow. */
static void oled_data(AppState* app, uint8_t b) {
    if(app->oled_page < OLED_PAGES && app->oled_col < OLED_COLS) {
        uint32_t idx = (uint32_t)app->oled_page * OLED_COLS + app->oled_col;
        if(idx < (uint32_t)OLED_FB_SIZE) {
            app->oled_work[idx] = b;
            app->oled_work_dirty = 1;
        }
    }
    if(app->oled_col < 0xFF) app->oled_col++;
    /* if it exceeds, clamp until the next set-page/col */
}

static void spi_byte(AppState* app, uint8_t b) {
    /* SPI bus routing by CHIP SELECT (DYN_HW_D605.md §0/§1): the OLED and the
     * Si4432 SHARE USART0. The Si4432 CS is PE9 (port4.9, ACTIVE-HIGH); when PE9
     * is LOW the byte belongs to the OLED (its command vs pixel-data chosen by the
     * DC pin A9 = port0.9). The old code fed EVERY SPI byte (incl. ~55000 OLED
     * pixel bytes) through the Si4432 model, corrupting the radio register state.
     * Now we gate strictly on PE9: CS HIGH -> Si4432 transaction, CS LOW -> OLED.
     * Faithful port of emu/pandora_tui.py _spi_byte (si_cs = (gpio_dout[4]>>9)&1). */
    uint8_t si_cs = (uint8_t)((app->gpio_dout[app->prof->rf_cs_port] >>
                               app->prof->rf_cs_pin) & 1u);

    if(!si_cs) {
        /* --- OLED byte (Si4432 CS inactive) --- */
        /* Abort any partial Si4432 transaction that was in flight when CS fell. */
        app->spi_phase = -1;
        if(oled_dc_level(app))
            oled_cmd(app, b); /* DC HIGH = command */
        else
            oled_data(app, b); /* DC LOW  = pixel data */
        return;
    }

    /* --- Si4432 transaction (CS = PE9 HIGH) --- */
    if(app->spi_phase < 0) {
        if(b & 0x80) {
            app->spi_kind = 'w';
            app->spi_reg = b & 0x7F;
        } else {
            app->spi_kind = 'r';
            app->spi_reg = b & 0x7F;
        }
        app->spi_phase = 0;
        /* junk echo when sending the address */
        app->spi_rx[app->spi_rx_tail] = 0;
        app->spi_rx_tail = (uint8_t)((app->spi_rx_tail + 1) % SPI_RX_QUEUE_LEN);
    } else {
        if(app->spi_kind == 'w') {
            uint8_t wreg = app->spi_reg;
            /* DIRECT MODE ONLY: REG[0x7F] (FIFO Access) is NOT special-cased. The
             * firmware never writes the FIFO in direct mode (DYN_HW_D605.md: 0
             * accesses). It is treated as a plain register write below. */
            int nm = si4432_write(&app->si, wreg, b);
            /* Si4432 -> CC1101 "follow firmware" bridge: when the firmware
             * reprograms the carrier (REG[0x75..0x77]) re-map the CC1101
             * frequency; when it changes modulation / data-source (REG[0x71])
             * track modtyp + dtmod and compute the FSK deviation. */
            if(app->rf_follow) {
                if(wreg == 0x75 || wreg == 0x76 || wreg == 0x77) {
                    rf_follow_apply_freq(app);
                } else if(wreg == 0x71 || wreg == 0x72) {
                    uint8_t mt = app->si.regs[0x71] & 0x03;
                    uint8_t dt = (app->si.regs[0x71] >> 6) & 0x03;
                    /* Si4432 FSK deviation: fd[8:0] with fd[8]=REG71 bit2,
                     * fd[7:0]=REG72. deviation = 625 Hz * fd (datasheet approx). */
                    uint32_t fd = (uint32_t)app->si.regs[0x72] |
                                  (((uint32_t)(app->si.regs[0x71] >> 2) & 1u) << 8);
                    uint32_t devi = fd * 625u;
                    if(mt != app->si_modtyp || dt != app->si_dtmod ||
                       devi != app->si_devi_hz) {
                        app->si_modtyp = mt;
                        app->si_dtmod = dt;
                        app->si_devi_hz = devi;
                        static const char* const MODN[4] = {"unmod", "OOK", "FSK",
                                                            "GFSK"};
                        static const char* const SRC[4] = {"direct-GPIO", "direct-SDI",
                                                           "FIFO", "PN9"};
                        FURI_LOG_I(TAG,
                                   "RF: firmware modulation = %s src = %s devi = %lu Hz "
                                   "(REG71=0x%02X REG72=0x%02X)",
                                   MODN[mt], SRC[dt], (unsigned long)devi,
                                   app->si.regs[0x71], app->si.regs[0x72]);
                        /* The actual CC1101 preset reload happens on the idle
                         * radio right before TX/RX (rf_reapply_preset), so we do
                         * NOT touch the radio here (it may be mid RX/TX). */
                    }
                }
            }
            if(nm >= 0) {
                /* rf_tx_on / rf_rx_on detected. Manual overlay capture first
                 * (unchanged), then the automatic follow bridge. */
                tx_on_si_mode(app, nm);
                if(app->rf_follow) rf_follow_apply_mode(app, nm);
            }
        } else {
            /* DIRECT MODE ONLY: plain register read (incl. 0x03/0x04 INT_STATUS,
             * cleared on read). REG[0x7F] is NOT special-cased -- the firmware
             * never reads the FIFO in direct mode (DYN_HW_D605.md: 0 accesses). */
            uint8_t rv = si4432_read(&app->si, app->spi_reg);
            app->spi_rx[app->spi_rx_tail] = rv;
            app->spi_rx_tail = (uint8_t)((app->spi_rx_tail + 1) % SPI_RX_QUEUE_LEN);
        }
        app->spi_phase = -1;
    }
}

/* Keyfob indicator LED pin. This is PHYSICAL HARDWARE (GPIO port3=D pin14, bit
 * 0x4000; confirmed by the RE: "DOUT(port3) ^= 0x4000 ; blink LED"), identical
 * for every firmware of this keyfob -- so it is a HW constant, NOT a per-firmware
 * profile field. Agnostic. */
#define KEYFOB_LED_PORT 3u
#define KEYFOB_LED_PIN 14u

/* Mirror the keyfob indicator LED (HW pin port3/pin14) onto the Flipper's red
 * LED. Called after every GPIO DOUT write on that port, reading the FINAL pin
 * state (so DOUTSET/DOUTCLR/DOUT are all handled uniformly). We only touch the
 * HAL on a real change (led_on cache) to avoid spamming furi_hal_light_set in the
 * firmware's fast blink loop. The first toggle is logged (throttled). */
static void led_mirror(AppState* app, uint32_t port) {
    if(port != KEYFOB_LED_PORT) return;
    bool on = (app->gpio_dout[port] >> KEYFOB_LED_PIN) & 1u;
    if(on == app->led_on) return;
    app->led_on = on;
    furi_hal_light_set(LightRed, on ? 0xFF : 0x00);
    if(!app->led_logged) {
        app->led_logged = true;
        FURI_LOG_D(TAG, "LED: port%u pin%u -> %s (first toggle; subsequent are silent)",
                   (unsigned)KEYFOB_LED_PORT, (unsigned)KEYFOB_LED_PIN,
                   on ? "ON" : "OFF");
    }
}

/* Buzzer GPIO detection. The buzzer pin is [DESC]: we watch for ANY GPIO pin
 * that is NOT a known function (LED / OLED-DC / Si4432-DATA / buttons) and that
 * toggles repeatedly in a short window (a square wave => audible tone). When such
 * a burst is seen we (a) log the candidate once for hardware confirmation and
 * (b) measure the toggle half-period to request a tentative beep (started
 * non-blockingly by the hot loop). pin_prev/pin_now are the OLD/NEW pin levels. */
static bool buzz_pin_is_reserved(AppState* app, uint32_t port, uint32_t pin) {
    const ArmProfile* p = app->prof;
    if(port == KEYFOB_LED_PORT && pin == KEYFOB_LED_PIN) return true; /* LED */
    if(port == OLED_DC_PORT && pin == OLED_DC_PIN) return true; /* OLED DC (A9) */
    if(port == p->data_port && pin == p->data_pin) return true; /* Si4432 RX-data (PB0) */
    /* RF control pins are NOT a buzzer (they toggle during radio activity):
     * CS=PE9, TX-data=PB3, antenna PE14/PE15. PE9 (CS) was a false positive. */
    if(port == p->tx_data_port && pin == p->tx_data_pin) return true; /* Si4432 TX-data (PB3) */
    if(port == p->rf_cs_port && pin == p->rf_cs_pin) return true; /* Si4432 CS (PE9) */
    if(port == p->rf_ant_rx_port && pin == p->rf_ant_rx_pin) return true; /* RX antenna (PE14) */
    if(port == p->rf_ant_tx_port && pin == p->rf_ant_tx_pin) return true; /* TX antenna (PE15) */
    /* the 6 keyfob buttons are DIN (inputs), but guard anyway in case a profile
     * reuses a port for both; a button pin as DOUT is not a buzzer. */
    if((port == p->up_port && pin == p->up_pin) ||
       (port == p->down_port && pin == p->down_pin) ||
       (port == p->ok_port && pin == p->ok_pin) ||
       (port == p->back_port && pin == p->back_pin) ||
       (port == p->b5_port && pin == p->b5_pin) ||
       (port == p->b6_port && pin == p->b6_pin))
        return true;
    return false;
}

/* Request a tentative beep from the detected toggle period. Converts the
 * half-period in insn -> us -> Hz (full period = 2 * half-period). Clamps to the
 * Flipper speaker range; falls back to a fixed tone if the period is unreliable.
 * This only SETS state (buzz_request/spk_freq/buzz_active_tick); the hot loop
 * owns the speaker so mmio_write never blocks or touches the HAL. */
static void buzz_request_tone(AppState* app) {
    float freq = BUZZ_TONE_DEFAULT_HZ;
    if(app->buzz_period_insn > 0) {
        /* full square-wave period = 2 half-periods; period_us = periods*insn_us */
        uint64_t half_us = (app->buzz_period_insn * ARM_INSN_US_NUM) / ARM_INSN_US_DEN;
        uint64_t full_us = half_us * 2u;
        if(full_us > 0) {
            float f = 1000000.0f / (float)full_us;
            if(f >= BUZZ_TONE_MIN_HZ && f <= BUZZ_TONE_MAX_HZ) freq = f;
        }
    }
    app->spk_freq = freq;
    app->buzz_active_tick = furi_get_tick();
    app->buzz_request = true;
}

static void buzz_detect_gpio(AppState* app, uint32_t port, uint32_t pin_mask, uint16_t before) {
    /* only consider pins that actually CHANGED and are not reserved functions */
    uint16_t after = app->gpio_dout[port];
    uint16_t changed = (uint16_t)((before ^ after) & pin_mask);
    if(!changed) return;
    for(uint32_t pin = 0; pin < 16; pin++) {
        if(!((changed >> pin) & 1u)) continue;
        if(buzz_pin_is_reserved(app, port, pin)) continue;
        /* a non-reserved DOUT pin toggled. Track it as a buzzer candidate. */
        if(app->buzz_cand_port != (uint8_t)port || app->buzz_cand_pin != (uint8_t)pin) {
            /* new / different candidate: (re)start the measurement */
            app->buzz_cand_port = (uint8_t)port;
            app->buzz_cand_pin = (uint8_t)pin;
            app->buzz_edges = 1;
            app->buzz_last_edge_insn = emu_insn_now(app);
            app->buzz_period_insn = 0;
            continue;
        }
        /* same candidate toggled again: measure the half-period */
        uint64_t now_insn = emu_insn_now(app);
        uint64_t dinsn = now_insn - app->buzz_last_edge_insn;
        app->buzz_last_edge_insn = now_insn;
        if(dinsn == 0 || dinsn > BUZZ_GAP_INSN) {
            /* too slow / stale -> restart the burst (not a tone) */
            app->buzz_edges = 1;
            app->buzz_period_insn = 0;
            continue;
        }
        app->buzz_period_insn = dinsn;
        app->buzz_edges++;
        if(app->buzz_edges >= BUZZ_MIN_EDGES) {
            if(!app->buzz_logged) {
                app->buzz_logged = true;
                FURI_LOG_D(TAG,
                           "BUZZER? gpio port%u pin%u toggling (%lu edges, ~%lu insn/half) "
                           "-- CONFIRM on hardware",
                           (unsigned)port, (unsigned)pin,
                           (unsigned long)app->buzz_edges,
                           (unsigned long)app->buzz_period_insn);
            }
            /* square-wave burst detected -> request a tentative beep */
            buzz_request_tone(app);
        }
    }
}

static void mmio_write(void* ctx, uint32_t addr, uint32_t val, int size) {
    AppState* app = (AppState*)ctx;
    uint32_t base = addr & 0xFFFFF000u;
    uint32_t off = addr & 0xFFFu;
    if(base == 0x4000C000u && off == 0x34) { /* USART0 TXDATA -> Si4432 */
        spi_byte(app, (uint8_t)(val & 0xFF));
    } else if(addr == 0x4000611Cu) { /* GPIO EXTI IFC: clear interrupt flags */
        app->gpio_if &= (uint16_t)~(val & 0xFFFFu);
    } else if(addr == 0x40006118u) { /* GPIO EXTI IFS: set interrupt flags (SW) */
        app->gpio_if |= (uint16_t)(val & 0xFFFFu);
    } else if(addr >= 0x40006000u && addr < 0x40006000u + 36u * 16u) { /* GPIO DOUT* */
        uint32_t port = (addr - 0x40006000u) / 36u;
        uint32_t poff = (addr - 0x40006000u) - port * 36u;
        if(port < 16) {
            uint16_t before = app->gpio_dout[port]; /* for LED/buzzer edge detect */
            uint16_t touched = 0; /* which pins this write may have changed */
            if(poff == 0x10) {
                app->gpio_dout[port] &= (uint16_t)~val; /* DOUTCLR */
                touched = (uint16_t)val;
            } else if(poff == 0x14) {
                app->gpio_dout[port] |= (uint16_t)val; /* DOUTSET */
                touched = (uint16_t)val;
            } else if(poff == 0x0C) {
                app->gpio_dout[port] = (uint16_t)val; /* DOUT */
                touched = 0xFFFFu;
            }
            /* TX capture: in direct mode the firmware bit-bangs the TX-DATA pin
             * PB3 (tx_data_port/tx_data_pin), NOT the RX-data pin PB0. See
             * DYN_HW_D605.md §9.B (TX waveform pin = port1.3/PB3). */
            if(app->tx_capturing && port == app->prof->tx_data_port) {
                bool level = (app->gpio_dout[port] >> app->prof->tx_data_pin) & 1u;
                tx_capture_data_edge(app, level);
            }
            /* mirror the indicator LED (reads the FINAL pin state) */
            led_mirror(app, port);
            /* detect a buzzer square-wave on any non-reserved pin that changed */
            if(touched) buzz_detect_gpio(app, port, touched, before);
        }
    } else if(base == 0x40010000u) { /* TIMER0..3: track CNT reset + TOP */
        uint32_t tn = off >> 10;
        uint32_t sub = off & 0x3FFu;
        if(tn < 4) {
            if(sub == 0x24) {
                /* firmware writes CNT (usually 0): re-anchor t0 so CNT restarts
                 * counting from here (faithful free-running model). */
                app->timer_t0[tn] = emu_insn_now(app);
            } else if(sub == 0x3C) {
                app->timer_top[tn] = val ? val : 0xFFFFu; /* TOP (wrap) */
            } else if(!app->buzz_timer_logged &&
                      (sub == 0x00 /* CTRL (mode/clk) */ ||
                       (sub >= 0x30 && sub <= 0x74) /* ROUTE + CCx_CTRL/CCV/CCVB */)) {
                /* BUZZER? candidate: the firmware programmed a TIMER sub-register
                 * that is NOT part of our CNT/TOP delay model -- typically a
                 * compare channel (CCx_CCV, PWM duty) or the output ROUTE/CTRL that
                 * turns a TIMER CC pin into a PWM tone generator for a piezo buzzer.
                 * Logged ONCE so a human can correlate it with an audible beep on
                 * real hardware and confirm the exact register. */
                app->buzz_timer_logged = true;
                FURI_LOG_D(TAG,
                           "BUZZER? timer%u write 0x%08lX=0x%lX (sub=0x%03lX, not CNT/TOP) "
                           "-- possible PWM tone; CONFIRM on hardware",
                           (unsigned)tn, (unsigned long)addr, (unsigned long)val,
                           (unsigned long)sub);
            }
        }
    } else if(addr == 0xE000ED08u) {
        /* SCB->VTOR: the firmware installs its active vector table base at
         * runtime (flash-XIP 0x8100 for both known fw, or RAM). Store it so
         * cur_vtor()/the NVIC dispatch honour the DYNAMIC table. The PPB has no
         * backing memory in the core, so we keep it here. Agnostic: we honour
         * whatever the firmware writes (ported from the ESP32 port). */
        app->ppb_vtor = val;
    } else if(addr == 0xE000E100u) {
        /* NVIC ISER0 (set-enable, IRQ0..31): writing 1 ENABLES IRQ N. Track the
         * live enabled mask so the scheduler only fires IRQs the firmware really
         * wants. Ported from pandora_tui.py _on_ppb_write / the ESP32 port. */
        app->nvic_enabled |= (uint64_t)(val & 0xFFFFFFFFu);
    } else if(addr == 0xE000E104u) {
        /* NVIC ISER1 (set-enable, IRQ32..63): this MCU uses up to ~IRQ36. */
        app->nvic_enabled |= (uint64_t)(val & 0xFFFFFFFFu) << 32;
    } else if(addr == 0xE000E180u) {
        /* NVIC ICER0 (clear-enable, IRQ0..31): writing 1 DISABLES IRQ N. */
        app->nvic_enabled &= ~((uint64_t)(val & 0xFFFFFFFFu));
    } else if(addr == 0xE000E184u) {
        /* NVIC ICER1 (clear-enable, IRQ32..63). */
        app->nvic_enabled &= ~((uint64_t)(val & 0xFFFFFFFFu) << 32);
    }
    (void)size;
}

/* ------------------------------------------------------------ AGNOSTIC NVIC --
 *
 * Driven by the firmware's REAL ISER/ICER (nvic_enabled) and dynamic VTOR
 * (ppb_vtor). The old port had bugs: it round-robined {12,13,14,1} blindly, read
 * VTOR via a callback that returned 0 (so IRQs rarely fired), and hardcoded the
 * default-handler address 0x82DB. All fixed here, matching the ESP32 port and
 * pandora_tui.py. */

/* Current VTOR: the firmware installs it at runtime; stored in ppb_vtor (the PPB
 * has no backing memory in the core). Agnostic. */
static uint32_t cur_vtor(AppState* app) {
    return app->ppb_vtor;
}

static uint32_t irq_handler_addr(AppState* app, int irqn) {
    uint32_t vt = cur_vtor(app);
    uint32_t off = vt + 0x40u + (uint32_t)irqn * 4u;
    return thumb_read_mem(app->cpu, off, 4);
}

/* EFM32 Gecko TIMER IRQ lines (TIMER0..2 = IRQ12/13/14). These are IRQ NUMBERS
 * (indices into the VTOR-pointed vector table), NOT firmware code addresses. The
 * UI refresh tick of both firmwares is IRQ13 (TIMER2). IRQ1 is intentionally
 * EXCLUDED now (firing an always-on helper on every tick floods the fw when it
 * masks IRQ13 in a critical section -> pmax UI livelock). Same as the ESP32 port
 * and the Python core. */
static const int TIMER_IRQS[] = {12, 13, 14};

/* Try to enter the exception for IRQ 'irqn'. Returns true if it actually entered
 * a VALID handler. Rejects: empty/0 vectors, handlers outside flash/RAM, and the
 * default/weak handler (a `b .` self-branch, opcode 0xE7FE) -- all detected
 * GENERICALLY (no firmware address hardcoded; the old 0x82DB hardcode is gone).
 * Ported from the ESP32 try_enter_irq / pandora_tui.py _enter_irq. */
static bool try_enter_irq(AppState* app, int irqn) {
    uint32_t handler = irq_handler_addr(app, irqn);
    uint32_t h = handler & ~1u;
    bool in_flash = (h < PA_FLASH_BASE + app->fc.flash_used);
    bool in_ram = (h >= PA_RAM_BASE && h < PA_RAM_BASE + PA_RAM_SIZE);
    if(!handler || !(in_flash || in_ram)) return false;
    /* skip a branch-to-self default handler (infinite loop) */
    uint16_t op = (uint16_t)thumb_read_mem(app->cpu, h, 2);
    if(op == 0xE7FEu) return false;
    app->tick_count++;
    app->in_irq = 1;
    thumb_enter_exception(app->cpu, handler, THUMB_EXC_RETURN_MSP_THREAD,
                          (uint32_t)(irqn + 16));
    app->next_pc = thumb_get_reg(app->cpu, 15);
    return true;
}

/* Fire the NEXT valid TIMER IRQ the firmware has ENABLED now in the real ISER.
 * Walk TIMER_IRQS round-robin, skipping IRQs not enabled in nvic_enabled and
 * ones try_enter_irq rejects. If none is valid this tick (fw masked the timers
 * in a critical section) fire NOTHING (no hang, no flood). If the fw has not
 * written any ISER yet (mask==0, very early boot) don't filter by ISER and let
 * try_enter_irq judge. Ported from the ESP32 nvic_tick / _irq_scheduler_tick. */
static void nvic_tick(AppState* app) {
    if(app->in_irq) return;
    uint32_t vtor = cur_vtor(app);
    if(vtor < 0x8000u) return; /* table not installed yet (pre-app) */
    int n = (int)(sizeof(TIMER_IRQS) / sizeof(TIMER_IRQS[0]));
    uint64_t mask = app->nvic_enabled;
    bool use_mask = (mask != 0);
    for(int k = 0; k < n; k++) {
        int irqn = TIMER_IRQS[app->irq_rr % n];
        app->irq_rr++;
        if(use_mask && !(mask & ((uint64_t)1u << irqn))) continue; /* not enabled now */
        if(try_enter_irq(app, irqn)) return;
    }
    /* no valid slot this tick */
}

/* GPIO EXTI delivery: if a button pin edge raised a pending GPIO interrupt and
 * the firmware has the matching GPIO IRQ ENABLED (IRQ1=GPIO_EVEN for even pins,
 * IRQ11=GPIO_ODD for odd pins), enter its ISR so the firmware processes the
 * button. The ISR reads the EXTI IF register (0x40006114) and clears it via IFC.
 * Returns true if an ISR was entered. AGNOSTIC: triggered by the observed button
 * pin edge, gated by the firmware's own ISER; no firmware code address used. */
static bool gpio_tick(AppState* app) {
    if(app->in_irq) return false;
    uint32_t vtor = cur_vtor(app);
    if(vtor < 0x8000u) return false;
    uint64_t mask = app->nvic_enabled;
    bool use_mask = (mask != 0);
    if(app->pending_gpio_odd) {
        if(!use_mask || (mask & ((uint64_t)1u << 11))) {
            if(try_enter_irq(app, 11)) { app->pending_gpio_odd = 0; return true; }
        }
    }
    if(app->pending_gpio_even) {
        if(!use_mask || (mask & ((uint64_t)1u << 1))) {
            if(try_enter_irq(app, 1)) { app->pending_gpio_even = 0; return true; }
        }
    }
    return false;
}

/* ------------------------------------------------------------ WFI scan ------
 * AGNOSTIC: scan the whole code space for WFI(0xBF30)/WFE(0xBF20). Flash is
 * scanned from 0x40 (past the initial vector table, so vector data is not
 * mistaken for opcodes) -- NOT the old hardcoded 0x8100. RAM is rescanned once
 * after boot (app-in-RAM loaders). The addresses are kept in a growable set for
 * membership tests. Ported from the ESP32 find_wfi / _find_wfi. */
static void wfi_add(AppState* app, uint32_t addr) {
    if(app->wfi_count >= app->wfi_cap) {
        uint32_t ncap = app->wfi_cap ? app->wfi_cap * 2u : 64u;
        uint32_t* nb = (uint32_t*)realloc(app->wfi_set, ncap * sizeof(uint32_t));
        if(!nb) return;
        app->wfi_set = nb;
        app->wfi_cap = ncap;
    }
    app->wfi_set[app->wfi_count++] = addr;
}

static bool wfi_contains(AppState* app, uint32_t addr) {
    for(uint32_t i = 0; i < app->wfi_count; i++)
        if(app->wfi_set[i] == addr) return true;
    return false;
}

static void scan_wfi_buf(AppState* app, const uint8_t* buf, uint32_t base,
                         uint32_t len, uint32_t start) {
    for(uint32_t off = start; off + 1u < len; off += 2) {
        uint16_t hw = (uint16_t)(buf[off] | (buf[off + 1] << 8));
        if(hw == 0xBF30u || hw == 0xBF20u) wfi_add(app, base + off);
    }
}

/* Scan flash for WFI/WFE from 0x40. Streams each 4KB page from the paged cache
 * as a byte array (no per-halfword fc_read -> no LRU thrash at launch). */
static void find_wfi(AppState* app) {
    FlashCache* fc = &app->fc;
    app->wfi_count = 0;
    uint32_t size = fc->flash_used;
    uint32_t lim = size < 0x20000u ? size : 0x20000u;
    for(uint32_t page_base = 0; page_base < lim; page_base += FLASH_PAGE_SIZE) {
        uint32_t page_end = page_base + FLASH_PAGE_SIZE;
        if(page_end > lim) page_end = lim;
        uint32_t first = page_base < 0x40u ? 0x40u : page_base;
        if(first >= page_end) continue;
        const uint8_t* page = fc_page_ptr(fc, page_base / FLASH_PAGE_SIZE);
        if(!page) continue;
        uint32_t off = first - page_base;
        if(off & 1u) off++;
        for(; off + 1u < page_end - page_base; off += 2) {
            uint16_t hw = (uint16_t)(page[off] | (page[off + 1] << 8));
            if(hw == 0xBF30u || hw == 0xBF20u) wfi_add(app, page_base + off);
        }
    }
    app->wfi_pc_hint = app->wfi_count ? app->wfi_set[0] : 0xFFFFFFFFu;
    app->wfi_ram_scanned = 0;
}

static void rescan_wfi_ram(AppState* app) {
    /* RAM holds the app (and its WFI) when a loader copied code there. */
    scan_wfi_buf(app, app->ram, PA_RAM_BASE, PA_RAM_SIZE, 0u);
}

/* ------------------------------------------------------------ unlock signal -
 * AGNOSTIC: a locked keyfob shows a BLACK screen; once unlocked the firmware
 * draws its UI. We derive 'unlocked' from the OBSERVABLE framebuffer having lit
 * pixels, NOT from a firmware code address (the old code hooked unlock_pc).
 * Ported from the ESP32 emu_is_unlocked / pandora_tui.py @property unlocked. */
static bool emu_is_unlocked(AppState* app) {
    for(int i = 0; i < OLED_FB_SIZE; i++)
        if(app->oled_fb[i]) return true;
    return false;
}

/* Per-instruction hook: counts, keeps PC, and performs the AGNOSTIC WFI skip.
 * The old hook also detected unlock by pc==unlock_pc and advanced the knock/nav
 * PC-sync FSMs -- all REMOVED (unlock is now observable via the framebuffer, and
 * knock/nav are pure-GPIO presses with no PC-sync). If the firmware parks in a
 * WFI/WFE, step over it so the foreground polling loop keeps spinning and the
 * NVIC tick can advance. Detected by membership in the scanned set (no
 * per-firmware address). Ported from the ESP32 code_hook / _on_code. */
static void code_hook(void* ctx, uint32_t pc) {
    AppState* app = (AppState*)ctx;
    app->ninsn = emu_insn_now(app);
    app->pc = pc;
    if(app->wfi_count && wfi_contains(app, pc & ~1u)) {
        /* advance PC past the 16-bit WFI/WFE so the core does not re-park. */
        thumb_set_reg(app->cpu, 15, (pc & ~1u) + 2u + 1u);
    }
}

/* ------------------------------------------------------------ execution ----
 *
 * FUSION of the Flipper fast-path with the AGNOSTIC ISER scheduler.
 *
 * The Flipper improvement (preserved) is the HOOKLESS fast-path: once unlocked
 * and with no PC-sync FSM active, the core runs its own tight thumb_run() loop
 * instead of crossing back into this wrapper per instruction. The old code ran
 * ONE nvic_tick at the TOP of the call and then let thumb_run(count) run the
 * WHOLE budget uninterrupted -- so with 6000-insn callers the IRQ13 UI tick
 * fired at the caller's cadence, but a single large call (auto_unlock uses
 * 50k-100k) diluted the tick ~N-fold and the firmware's button scanner missed
 * injected key edges (UI stayed black). This is the exact bug the ESP32 port
 * fixed by SUBDIVIDING.
 *
 * FUSION: we subdivide the budget into RUN_SUBBURST chunks and interleave the
 * nvic_tick BETWEEN chunks (so IRQ13 keeps its cadence regardless of the caller
 * burst size), while keeping the hookless fast-path INSIDE each chunk. The slow
 * instruction-at-a-time path (with EXC_RETURN detection) is used only while an
 * IRQ is in flight or the hook is needed (pre-unlock boot). Ported from the
 * ESP32 emu_run_burst + the Python run_burst. */
#define RUN_SUBBURST 15000u

static void run_burst(AppState* app, uint32_t count) {
    ThumbCore* cpu = app->cpu;
    ThumbHook saved_hook = cpu->hook;
    void* saved_hook_ctx = cpu->hook_ctx;

    uint32_t remaining = count;
    int guard = 0;
    while(remaining > 0 && guard < 1024) {
        guard++;
        bool app_running = cur_vtor(app) >= 0x8000u;
        /* Once the app is running, rescan RAM ONCE for WFI/WFE in case a loader
         * copied the app (and its WFI) to SRAM (app-in-RAM, e.g. pmax). */
        if(app_running && !app->wfi_ram_scanned) {
            rescan_wfi_ram(app);
            app->wfi_ram_scanned = 1;
        }
        /* INTERLEAVED TICK: fire the NVIC tick when stuck in a WFI or at the
         * periodic RUN_SUBBURST cadence, BEFORE running the next chunk. This is
         * what keeps IRQ13 (the UI tick) from being diluted inside a big call. */
        bool stuck_wfi =
            app_running && app->wfi_count && wfi_contains(app, app->next_pc & ~1u);
        uint64_t now_insn = emu_insn_now(app);
        bool periodic = (now_insn - app->last_tick_insn) > RUN_SUBBURST;
        /* GPIO EXTI first: deliver a pending button-edge interrupt BEFORE the
         * periodic timer tick so a press is processed promptly (not starved by
         * the round-robin timer scheduler). Only fires on a real button pin edge
         * when the firmware enabled the GPIO IRQ. */
        if(app_running && !app->in_irq &&
           (app->pending_gpio_even || app->pending_gpio_odd)) {
            gpio_tick(app);
        }
        /* Match Unicorn run_burst: fire a tick when a per-burst pending_tick was
         * raised by a driver (press/boot), when stuck in WFI, or periodically. */
        if(app_running && !app->in_irq && (app->pending_tick || stuck_wfi || periodic)) {
            app->pending_tick = 0;
            app->last_tick_insn = now_insn;
            nvic_tick(app);
        }

        uint32_t step = remaining < RUN_SUBBURST ? remaining : RUN_SUBBURST;
        uint64_t before = emu_insn_now(app);

        /* The hookless fast-path is valid only in steady state (unlocked, no IRQ
         * in flight). Otherwise the per-instruction hook (WFI skip) and the
         * EXC_RETURN detection are both required. */
        bool skip_hook = app->unlocked && !app->in_irq;
        thumb_set_reg(cpu, 15, app->next_pc);
        if(skip_hook) {
            thumb_set_hook(cpu, NULL, NULL);
            (void)thumb_run(cpu, step);
            thumb_set_hook(cpu, saved_hook, saved_hook_ctx);
            app->next_pc = cpu->r[15];
        } else {
            /* Run until 'step' EXECUTED instructions retire (emu_insn_now/cycles
             * delta), NOT 'step' thumb_step() calls -- an IT-skipped NOP must not
             * count, matching Unicorn's emu_start(count=step) and the hookless
             * fast-path (thumb_run, also cycles-counted). Counting by calls drifted
             * the sub-burst boundary by a few instructions per IT-skip, shifting the
             * NVIC tick phase off the oracle and breaking cumulative navigation.
             * Guard bounds the degenerate all-NOP case. */
            uint64_t slow_start = emu_insn_now(app);
            uint32_t guard2 = 0;
            uint32_t guard_lim = step * 2u + 16u;
            while((uint32_t)(emu_insn_now(app) - slow_start) < step && guard2 < guard_lim) {
                guard2++;
                int rc = thumb_step(cpu);
                if(rc < 0) break;
                uint32_t pc = thumb_get_reg(cpu, 15);
                if((pc & 0xFFFFFFF0u) == 0xFFFFFFF0u) { /* EXC_RETURN */
                    thumb_exc_return(cpu, pc);
                    app->in_irq = 0;
                    break;
                }
            }
            app->next_pc = thumb_get_reg(cpu, 15);
        }
        app->ninsn = emu_insn_now(app);
        app->pc = app->next_pc & ~1u;

        uint64_t consumed = app->ninsn - before;
        uint32_t dec = consumed > 0 ? (consumed < remaining ? (uint32_t)consumed : remaining) : 1u;
        remaining -= dec;
    }
    /* keep the observable unlock flag up to date (no code-address hook) */
    if(!app->unlocked && emu_is_unlocked(app)) app->unlocked = 1;

    /* Raise a UI tick request for the NEXT burst. The reference (Unicorn
     * pandora_tui) sets _pending_tick=True after EVERY run_burst its drivers make
     * (boot settle, knock presses, button presses, idle loop). Centralising it
     * here reproduces that per-burst tick for every caller (auto_unlock,
     * nav_button, the event loop) and keeps IRQ13's cadence in phase with
     * Unicorn so post-unlock navigation matches. */
    app->pending_tick = 1;
}

/* Pre-unlock fallback press (rare: auto_unlock normally already ran at boot).
 * AGNOSTIC: a single short button press purely over GPIO, no scanner PC-sync. */
static void tap(AppState* app, int btn) {
    if(btn < 0 || btn >= BTN_COUNT) return;
    set_button(app, btn, true);
    uint64_t done = 0;
    while(done < 300000ull) {
        run_burst(app, 50000);
        done += 50000;
    }
    set_button(app, btn, false);
    done = 0;
    while(done < 300000ull) {
        run_burst(app, 50000);
        done += 50000;
    }
}

/* Cheap hash of the keyfob framebuffer (92x16=1472B), used by nav_button's
 * adaptive settle to detect a UI redraw. Mirrors hash(bytes(self.read_fb())) in
 * pandora_tui.py, where read_fb() defaults to the SPI/double-buffered frame: we
 * hash the STABLE VISIBLE buffer (oled_fb) so the settle only fires on a real
 * committed frame (end-of-flush), not on mid-flush RAM churn. FNV-1a. */
static uint32_t fb_hash(AppState* app) {
    uint32_t h = 2166136261u;
    for(uint32_t i = 0; i < (uint32_t)OLED_FB_SIZE; i++) {
        h ^= app->oled_fb[i];
        h *= 16777619u;
    }
    return h;
}

/* --- AGNOSTIC button/knock timing (pure HARDWARE order-of-magnitude, insn as a
 * proxy for time; NOT a per-firmware code address). Mirrors the ESP32 port and
 * pandora_tui.py press_button / auto_unlock durations. --- */
#define BTN_PRESS_INSN 300000ull   /* short click: pin held low ~this long */
#define BTN_HOLD_INSN 1200000ull   /* long press (above the long-press window) */
#define BTN_SETTLE_INSN 500000ull  /* settle after release (adaptive early-out) */
#define BTN_BURST 50000u

#define KNOCK_BOOT_SETTLE_INSN 1200000ull /* boot settle before the knock */
#define KNOCK_CLICK_INSN 150000ull        /* short knock click (brief pulses) */
#define KNOCK_CLICK_SETTLE_INSN 200000ull
#define KNOCK_GATE_HOLD_INSN 900000ull
#define KNOCK_GATE_SETTLE_INSN 400000ull
#define KNOCK_FINAL_SETTLE_INSN 2000000ull

/* Press+release ONE button POST-unlock PURELY over GPIO with realistic timing,
 * exactly like the hardware / the ESP32 emu_press_button / pandora_tui.py
 * press_button. Drive the DIN pin low (pressed), run the firmware a while (its
 * own polling scan -- powered by the faithful TIMERs + agnostic NVIC -- sees the
 * pin), then drive it high (released). The firmware distinguishes short click vs
 * long press by the TIME the pin was low, like a real finger. AGNOSTIC: no
 * PC/code-address sync (no scan_head/nav_reader). The adaptive settle
 * (improvement 7) is preserved: run short bursts and STOP as soon as the
 * committed framebuffer changes, for ~0.2s feedback instead of ~1s. */
/* Parameterised press+release, mirroring Unicorn pandora_tui.press_button exactly
 * (down-phase burst = 50000, settle-phase burst = 60000, adaptive early-out when
 * the UI redraws after >=120000 settle insns). The knock uses the SAME mechanism
 * (just different durations), which keeps the IRQ-tick phase at nav time identical
 * to the oracle. */
static bool press_button_timed(AppState* app, int btn, uint64_t down_insn,
                               uint64_t settle_insn) {
    if(btn < 0 || btn >= BTN_COUNT) return false;

    /* 1) press (pin low) */
    set_button(app, btn, true);
    uint64_t done = 0;
    while(done < down_insn) {
        run_burst(app, BTN_BURST);
        done += BTN_BURST;
    }
    /* 2) release (pin high) -> firmware dispatches the action on the edge */
    set_button(app, btn, false);

    /* 3) adaptive settle: run until the UI redraws (fb changes), bounded. */
    uint32_t prev = fb_hash(app);
    done = 0;
    bool redrew = false;
    while(done < settle_insn) {
        run_burst(app, 60000u);
        done += 60000u;
        uint32_t cur = fb_hash(app);
        if(cur != prev && done >= 120000u) { redrew = true; break; } /* redrew: early-out */
        prev = cur;
    }
    FURI_LOG_D(TAG, "press_button_timed: done btn=%d redrew=%d settle_insn=%llu",
               btn, redrew, (unsigned long long)done);
    return true;
}

static bool nav_button(AppState* app, int btn, bool hold) {
    if(btn < 0 || btn >= BTN_COUNT) return false;
    FURI_LOG_D(TAG, "nav_button: btn=%d hold=%d unlocked=%d", btn, hold, app->unlocked);
    return press_button_timed(app, btn, hold ? BTN_HOLD_INSN : BTN_PRESS_INSN,
                              BTN_SETTLE_INSN);
}

/* AGNOSTIC auto-unlock: press the knock-code PURELY over GPIO as an autotest
 * (knock = FIRMWARE CONFIG, supplied by the host/UI, NOT baked into the agnostic
 * core). Mirrors the ESP32 auto_unlock / pandora_tui.py: settle the boot with
 * FINE bursts (so the keyboard scanner reaches a phase where it samples every
 * port, incl. the B5 gate port), press the sequence with brief clicks, then hold
 * the gate. 'unlocked' is the OBSERVABLE display signal (framebuffer has lit
 * pixels), NOT a code address. Returns true if the display drew content. */
static bool auto_unlock(AppState* app) {
    uint32_t phase_tick = furi_get_tick();
    FURI_LOG_I(TAG, "boot: starting MCU execution/unlock sequence");
    app->unlocked = 0;
    app->max_step = -1;

    /* Resolve the knock sequence: the user sequence from the UI takes priority;
     * otherwise a model default (firmware config, not hardware). knock_seq /
     * knock_len / knock_gate are set by the caller (emu_reset_and_unlock). */
    const uint8_t* seq = app->knock_seq;
    uint8_t len = app->knock_len;
    int gate = app->knock_gate;

    /* 1) settle the boot with FINE bursts so the scanner samples every port */
    uint64_t done = 0;
    while(done < KNOCK_BOOT_SETTLE_INSN) {
        run_burst(app, 20000u);
        done += 20000u;
    }
    FURI_LOG_I(TAG, "boot: scan loop reached (%lu ms, insn=%llu)",
               (unsigned long)(furi_get_tick() - phase_tick),
               (unsigned long long)app->ninsn);

    if(len == 0) {
        /* no knock configured: just report the observable after the settle. */
        app->unlocked = emu_is_unlocked(app) ? 1 : 0;
        FURI_LOG_I(TAG, "boot: no knock configured -> unlocked=%d (observable)",
                   app->unlocked);
        return app->unlocked != 0;
    }

    /* 2) press the knock sequence as short clicks. Use the SAME press mechanism
     * as a regular button press (press_button_timed), exactly like the reference
     * (Unicorn auto_unlock -> press_button(press_insn=150000, settle=200000)).
     * The old code hand-rolled the click with 20000-insn bursts, a different
     * burst granularity than the oracle, which shifted the IRQ-tick phase by
     * ~200000 insns by nav time -> post-unlock UP/OK/DOWN landed at the wrong
     * point in the tick cycle. */
    for(uint8_t i = 0; i < len; i++) {
        int btn = seq[i];
        if(btn < 0 || btn >= BTN_COUNT) continue;
        press_button_timed(app, btn, KNOCK_CLICK_INSN, KNOCK_CLICK_SETTLE_INSN);
    }
    FURI_LOG_I(TAG, "boot: knock sequence injected (len=%u, insn=%llu)", len,
               (unsigned long long)app->ninsn);

    /* 3) final gate: hold it (reference: press_button(gate, hold=True,
     * hold_insn=900000, settle=400000)). */
    if(gate >= 0 && gate < BTN_COUNT) {
        press_button_timed(app, gate, KNOCK_GATE_HOLD_INSN, KNOCK_GATE_SETTLE_INSN);
    }

    /* 4) final settle: let the firmware finish unlocking + draw the UI */
    done = 0;
    while(done < KNOCK_FINAL_SETTLE_INSN && !emu_is_unlocked(app)) {
        run_burst(app, 100000u);
        done += 100000u;
    }
    app->unlocked = emu_is_unlocked(app) ? 1 : 0;
    FURI_LOG_I(TAG, "boot: unlock %s (%lu ms, insn=%llu, observable)",
               app->unlocked ? "OK" : "FAILED",
               (unsigned long)(furi_get_tick() - phase_tick),
               (unsigned long long)app->ninsn);
    return app->unlocked != 0;
}

/* ------------------------------------------------------------ render OLED */

/* The keyfob framebuffer is 92col x 16pag = 1472B, page-addressed, bit0 top,
 * GRAYSCALE 2bpp: each page byte is 4 logical pixels (bits 2k,2k+1 = pixel k),
 * 16 pag x 4 = 64 real rows. Decode each pixel as the 2-bit value (not the
 * population count), then threshold at level >= 2 for the Flipper's 1bpp canvas.
 * Paint the visible 88x64 area centered on the Flipper 128x64 screen.
 * (See RE_DISPLAY_D605.md §1.3/§6.2.) */
static void render_oled(AppState* app) {
    furi_mutex_acquire(app->fb_mutex, FuriWaitForever);
    memset(app->screen, 0, FB_SIZE);
    /* Read the STABLE VISIBLE buffer (double buffering), NOT the firmware RAM.
     * oled_fb only changes in oled_commit() (on the CONTENT 0xAF of a flush), so
     * it is always a COHERENT, non-blank frame -> no partial/black frames ->
     * no flicker/overwrite. (See the OLED capture block above and the fix notes
     * ported from emu/pandora_tui.py.) */
    const uint8_t* fbv = app->oled_fb;
    /* center the 88 VISIBLE columns in the Flipper's 128 wide screen (=> 20) */
    const int x_off = (SCREEN_W - OLED_VISIBLE_COLS) / 2;
    for(int page = 0; page < OLED_PAGES; page++) {
        for(int col = 0; col < OLED_VISIBLE_COLS; col++) {
            uint8_t v = fbv[page * OLED_COLS + col];
            for(int k = 0; k < 4; k++) {
                /* pixel level = (hi << 1) | lo; 01 and 10 are distinct gray levels. */
                int lvl = ((v >> (2 * k + 1)) & 1) << 1;
                lvl |= (v >> (2 * k)) & 1;
                if(lvl < 2) continue; /* map levels 0/1 to off, 2/3 to on */
                int y = page * 4 + k; /* 0..63 */
                int x = x_off + col;
                if(y < 0 || y >= SCREEN_H || x < 0 || x >= SCREEN_W) continue;
                app->screen[(y >> 3) * SCREEN_W + x] |= (uint8_t)(1u << (y & 7));
            }
        }
    }
    furi_mutex_release(app->fb_mutex);
}

static void framebuffer_commit_callback(
    uint8_t* data,
    size_t size,
    CanvasOrientation orientation,
    void* context) {
    __atomic_fetch_add(&s_fb_cb_inflight, 1, __ATOMIC_RELAXED);
    AppState* app = (AppState*)context;
    (void)orientation;
    if(!app || !data || size < FB_SIZE || app->menu_active) {
        __atomic_fetch_sub(&s_fb_cb_inflight, 1, __ATOMIC_RELAXED);
        return;
    }
    if(furi_mutex_acquire(app->fb_mutex, 0) != FuriStatusOk) {
        __atomic_fetch_sub(&s_fb_cb_inflight, 1, __ATOMIC_RELAXED);
        return;
    }
    const uint8_t* src = app->screen;
    for(size_t i = 0; i < FB_SIZE; i++) {
        data[i] = (uint8_t)(src[i] ^ 0xFF); /* screen bit=1 light; display bit=1 dark */
    }
    furi_mutex_release(app->fb_mutex);
    __atomic_fetch_sub(&s_fb_cb_inflight, 1, __ATOMIC_RELAXED);
}

/* ------------------------------------------------------------ input */

static uint8_t poll_raw_keys(void) {
    uint8_t keys = 0;
    for(size_t i = 0; i < input_pins_count; i++) {
        const InputPin* p = &input_pins[i];
        if(furi_hal_gpio_read(p->gpio) == p->inverted) continue;
        switch(p->key) {
        case InputKeyUp: keys |= KBIT_UP; break;
        case InputKeyDown: keys |= KBIT_DOWN; break;
        case InputKeyLeft: keys |= KBIT_LEFT; break;
        case InputKeyRight: keys |= KBIT_RIGHT; break;
        case InputKeyOk: keys |= KBIT_OK; break;
        case InputKeyBack: keys |= KBIT_BACK; break;
        default: break;
        }
    }
    /* DIAG: log raw key state only on change, so we can confirm on hardware that
     * button presses are actually reaching the app (debug level). */
    static uint8_t s_last_keys = 0;
    if(keys != s_last_keys) {
        FURI_LOG_D(TAG, "poll_raw_keys: 0x%02X (was 0x%02X)", keys, s_last_keys);
        s_last_keys = keys;
    }
    return keys;
}

/* ------------------------------------------------------------ load + boot */

/* Canonical flat flash image on the SD (flash_used bytes, gaps=0xFF). The flash
 * cache pages 4KB blocks from here at runtime. Lives in the app data dir. */
#define PA_DATA_DIR "/ext/apps_data/pandora_arm"
#define PA_FLASH_BIN PA_DATA_DIR "/flash.bin"

typedef enum { LoadOk, LoadIoError, LoadNoMem, LoadBadFormat } LoadResult;

/* Decodes a hex byte (2 chars). Returns -1 if invalid. */
static int hexbyte(const char* s) {
    int hi = (unsigned char)s[0], lo = (unsigned char)s[1];
    if(hi >= '0' && hi <= '9') hi -= '0';
    else { hi |= 0x20; if(hi >= 'a' && hi <= 'f') hi = hi - 'a' + 10; else return -1; }
    if(lo >= '0' && lo <= '9') lo -= '0';
    else { lo |= 0x20; if(lo >= 'a' && lo <= 'f') lo = lo - 'a' + 10; else return -1; }
    return (hi << 4) | lo;
}

/* Processes ONE line of Intel HEX (without the leading ':') writing the decoded
 * data bytes into the open flat temp file `out` by absolute address (seek+write).
 * Updates *ext_lin. Returns 1 if it was EOF, 0 if next, -1 on error. The temp
 * file is pre-filled with 0xFF, so records can arrive out of order safely. */
static int ihex_line(const char* ln, size_t len, File* out, uint32_t flash_size,
                     uint32_t* ext_lin, uint32_t* highest) {
    if(len < 10) return 0; /* line too short: ignore (stray CR/LF) */
    int count = hexbyte(ln);
    int ah = hexbyte(ln + 2);
    int al = hexbyte(ln + 4);
    int rtype = hexbyte(ln + 6);
    if(count < 0 || ah < 0 || al < 0 || rtype < 0) return -1;
    uint32_t addr = ((uint32_t)ah << 8) | (uint32_t)al;
    const char* data = ln + 8;
    if(rtype == 0x00) { /* data */
        uint32_t base = *ext_lin + addr;
        if(base >= flash_size) return 0; /* entirely above the used flash: skip */
        /* decode into a small local buffer, then one seek + one write */
        uint8_t buf[256];
        int n = 0;
        for(int k = 0; k < count; k++) {
            int b = hexbyte(data + k * 2);
            if(b < 0) return -1;
            if(base + (uint32_t)k < flash_size && n < (int)sizeof(buf))
                buf[n++] = (uint8_t)b;
        }
        if(n > 0) {
            if(!storage_file_seek(out, base, true)) return -1;
            if(storage_file_write(out, buf, (size_t)n) != (size_t)n) return -1;
            /* AGNOSTIC: track the highest written address (effective flash size,
             * like the Python core's `top = max(code)+1`). */
            if(highest && base + (uint32_t)n > *highest) *highest = base + (uint32_t)n;
        }
    } else if(rtype == 0x01) { /* EOF */
        return 1;
    } else if(rtype == 0x02) { /* ext SEGMENT addr: base = (seg << 4) */
        /* pmax's .hex uses this form (:020000021000 -> 0x10000) to reach the
         * code above 0x10000. Without it the upper flash (pmax code up to
         * 0x150FF) would be written to the wrong, low addresses. */
        int b0 = hexbyte(data), b1 = hexbyte(data + 2);
        if(b0 < 0 || b1 < 0) return -1;
        *ext_lin = (((uint32_t)b0 << 8) | (uint32_t)b1) << 4;
    } else if(rtype == 0x04) { /* ext LINEAR addr: base = (hi << 16) */
        int b0 = hexbyte(data), b1 = hexbyte(data + 2);
        if(b0 < 0 || b1 < 0) return -1;
        *ext_lin = (((uint32_t)b0 << 8) | (uint32_t)b1) << 16;
    }
    /* other types (03/05): ignore */
    return 0;
}

/* Pre-fills the open temp file with `size` bytes of 0xFF (open-bus value of the
 * unused flash). Done in chunks; returns false on any write error. */
static bool fill_file_ff(File* out, uint32_t size) {
    static uint8_t ff[512];
    memset(ff, 0xFF, sizeof(ff));
    if(!storage_file_seek(out, 0, true)) return false;
    uint32_t left = size;
    while(left) {
        size_t n = left < sizeof(ff) ? left : sizeof(ff);
        if(storage_file_write(out, ff, n) != n) return false;
        left -= (uint32_t)n;
    }
    return true;
}

/* Normalises the chosen firmware into the canonical FLAT temp .bin on the SD
 * (flash_used bytes, gaps=0xFF). STREAMS from the source (the .hex can be 600KB+;
 * the Flipper has little heap) and writes the flat image by absolute address.
 * This is done ONCE at load; the flash cache then pages 4KB blocks from the flat
 * file, never re-parsing Intel HEX. Robust and format-agnostic. */
static LoadResult flatten_firmware(AppState* app, Storage* storage, const char* path) {
    /* AGNOSTIC: the flat image spans the whole physical flash WINDOW (264KB).
     * The EFFECTIVE flash_used is derived below from the firmware image itself
     * (file size for a .bin, highest written address for a .hex) and rounded up
     * to a 4KB page, NOT a per-firmware constant. */
    const uint32_t flash_size = PA_FLASH_SIZE;
    uint32_t effective = 0; /* highest byte offset written + 1 */
    File* f = storage_file_alloc(storage);
    File* out = storage_file_alloc(storage);
    LoadResult res = LoadIoError;
    bool out_open = false;
    if(!f || !out) {
        if(f) storage_file_free(f);
        if(out) storage_file_free(out);
        return LoadNoMem;
    }
    do {
        if(!storage_file_open(f, path, FSAM_READ, FSOM_OPEN_EXISTING)) break;
        uint64_t fsize = storage_file_size(f);
        if(fsize == 0 || fsize > 4u * 1024u * 1024u) {
            res = LoadBadFormat;
            break;
        }

        storage_common_mkdir(storage, PA_DATA_DIR); /* ok if it already exists */
        if(!storage_file_open(out, PA_FLASH_BIN, FSAM_READ_WRITE, FSOM_CREATE_ALWAYS)) break;
        out_open = true;
        if(!fill_file_ff(out, flash_size)) break; /* gaps/open-bus = 0xFF */

        /* detect format by the first byte */
        uint8_t first = 0;
        if(storage_file_read(f, &first, 1) != 1) break;
        storage_file_seek(f, 0, true);

        /* shared streaming buffers (static: not on the stack). ~1.1KB .bss */
        static uint8_t chunk[512];
        static char line[600];
        size_t rd;

        if(first == ':') {
            /* Intel HEX: read in chunks and parse by lines (streaming) ->
             * seek+write each data record into the flat temp file. */
            size_t linepos = 0;
            uint32_t ext_lin = 0;
            bool eof_rec = false;
            bool err = false;
            while(!eof_rec && (rd = storage_file_read(f, chunk, sizeof(chunk))) > 0) {
                for(size_t i = 0; i < rd; i++) {
                    char ch = (char)chunk[i];
                    if(ch == ':') {
                        linepos = 0; /* start of record */
                    } else if(ch == '\n' || ch == '\r') {
                        if(linepos > 0) {
                            int r = ihex_line(line, linepos, out, flash_size,
                                              &ext_lin, &effective);
                            if(r < 0) { err = true; break; }
                            if(r == 1) { eof_rec = true; break; }
                            linepos = 0;
                        }
                    } else {
                        if(linepos < sizeof(line) - 1) line[linepos++] = ch;
                    }
                }
                if(err) break;
            }
            if(err) {
                res = LoadBadFormat;
                break;
            }
        } else {
            /* plain binary: copy the first flash_size bytes verbatim. */
            if(!storage_file_seek(out, 0, true)) break;
            uint32_t off = 0;
            bool werr = false;
            while(off < flash_size && (rd = storage_file_read(f, chunk, sizeof(chunk))) > 0) {
                size_t n = rd;
                if(off + n > flash_size) n = flash_size - off;
                if(storage_file_write(out, chunk, n) != n) { werr = true; break; }
                off += (uint32_t)n;
            }
            if(werr) break;
            effective = off; /* flat .bin: the whole file is the firmware */
        }
        /* AGNOSTIC flash_used: page-align the effective size up to 4KB and clamp
         * to the window; fall back to the window if nothing was written. Mirrors
         * the ESP32 emu_init sizing (page-align + margin). */
        if(effective == 0) effective = flash_size;
        effective = (effective + FLASH_PAGE_SIZE - 1u) & ~(FLASH_PAGE_SIZE - 1u);
        if(effective > flash_size) effective = flash_size;
        app->flash_used = effective;
        res = LoadOk;
    } while(false);
    if(out_open) storage_file_close(out);
    storage_file_free(out);
    storage_file_close(f);
    storage_file_free(f);
    return res;
}

/* ---------------------------------------------------------------- FlashCache */

/* Reads one 4KB page from the flat .bin into `dst`. A short read (last partial
 * page / SD hiccup) is padded with 0xFF (open-bus). Returns false on seek/IO
 * fault only. */
static bool fc_read_page(FlashCache* fc, uint32_t page, uint8_t* dst) {
    fc->miss_count++;
    uint32_t base = page * FLASH_PAGE_SIZE;
    if(!storage_file_seek(fc->file, base, true)) return false;
    size_t got = storage_file_read(fc->file, dst, FLASH_PAGE_SIZE);
    if(got < FLASH_PAGE_SIZE) memset(dst + got, 0xFF, FLASH_PAGE_SIZE - got);
    return true;
}

/* Allocates the cache slots (each an independent 4KB safe_malloc -> small blocks
 * that fit a fragmented heap) and the bookkeeping arrays, leaving FLASH_HEAP_RESERVE
 * free for the GUI/system. If every page gets a slot the flash is fully resident
 * (no eviction). Returns LoadOk / LoadNoMem. Faithful port of FlipperGB's cache
 * init (rom_load slot loop). */
static LoadResult fc_init(AppState* app) {
    FlashCache* fc = &app->fc;
    fc->flash_used = app->flash_used; /* AGNOSTIC: derived at load, not per-fw */
    fc->num_pages = (fc->flash_used + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    fc->use_counter = 0;
    fc->miss_count = 0;
    fc->num_slots = 0;
    fc->fully_resident = false;

    fc->file = storage_file_alloc(app->storage);
    if(!fc->file) return LoadNoMem;
    if(!storage_file_open(fc->file, PA_FLASH_BIN, FSAM_READ, FSOM_OPEN_EXISTING)) {
        FURI_LOG_E(TAG, "flash cache: cannot open %s", PA_FLASH_BIN);
        return LoadIoError;
    }

    /* bookkeeping arrays sized for the best case (all pages resident). Tiny:
     * num_pages * (ptr + 2*u32) ~= 22 * 12 = 264 B. */
    uint32_t cap = fc->num_pages;
    fc->slots = (uint8_t**)safe_malloc(cap * sizeof(uint8_t*));
    fc->slot_page = (uint32_t*)safe_malloc(cap * sizeof(uint32_t));
    fc->slot_use = (uint32_t*)safe_malloc(cap * sizeof(uint32_t));
    if(!fc->slots || !fc->slot_page || !fc->slot_use) return LoadNoMem;

    /* Allocate as many 4KB slots as fit, keeping runtime memory for the RX queue,
     * GUI direct draw, and system services. Degrade the reserve (14K -> 12K ->
     * 8K) only as needed to reach the minimum useful boot working set.
     * Each slot is its own small malloc (no big contiguous block). */
    static const size_t reserve_ladder[3] = {
        FLASH_HEAP_RESERVE, 12u * 1024u, 8u * 1024u};
    for(int step = 0; step < 3; step++) {
        size_t reserve = reserve_ladder[step];
        while(fc->num_slots < cap) {
            if(memmgr_get_free_heap() < reserve + FLASH_PAGE_SIZE + ALLOC_MARGIN) break;
            uint8_t* slot = (uint8_t*)safe_malloc(FLASH_PAGE_SIZE);
            if(!slot) break;
            fc->slots[fc->num_slots] = slot;
            fc->slot_page[fc->num_slots] = 0xFFFFFFFFu; /* empty */
            fc->slot_use[fc->num_slots] = 0;
            fc->num_slots++;
        }
        if(fc->num_slots >= FLASH_MIN_SLOTS || fc->num_slots >= fc->num_pages) break;
        FURI_LOG_W(TAG, "flash cache: degrading heap reserve to %uK for more slots",
                   (unsigned)(reserve_ladder[step + 1 < 3 ? step + 1 : step] / 1024u));
    }

    if(fc->num_slots < FLASH_MIN_SLOTS && fc->num_slots < fc->num_pages) {
        FURI_LOG_E(TAG, "flash cache: only %u slots (need >= %u)", fc->num_slots,
                   (unsigned)FLASH_MIN_SLOTS);
        return LoadNoMem;
    }

    if(fc->num_slots >= fc->num_pages) {
        /* fully resident: pre-load every page, slot i == page i, no eviction */
        for(uint32_t i = 0; i < fc->num_pages; i++) {
            if(!fc_read_page(fc, i, fc->slots[i])) return LoadIoError;
            fc->slot_page[i] = i;
            fc->slot_use[i] = ++fc->use_counter;
        }
        fc->fully_resident = true;
        /* the flat file is no longer needed for runtime paging */
        storage_file_close(fc->file);
        storage_file_free(fc->file);
        fc->file = NULL;
        FURI_LOG_I(TAG, "flash cache: fully resident (%lu pages x 4KB)",
                   (unsigned long)fc->num_pages);
    } else {
        /* streaming LRU: warm the first num_slots pages (the boot/copy-to-RAM
         * region is the low flash, so sequential warming is a good seed). */
        for(uint16_t i = 0; i < fc->num_slots; i++) {
            if(!fc_read_page(fc, i, fc->slots[i])) return LoadIoError;
            fc->slot_page[i] = i;
            fc->slot_use[i] = ++fc->use_counter;
        }
        FURI_LOG_I(TAG, "flash cache: streaming LRU (%u slots / %lu pages)",
                   fc->num_slots, (unsigned long)fc->num_pages);
    }
    return LoadOk;
}

static void fc_deinit(FlashCache* fc) {
    if(fc->slots) {
        for(uint16_t i = 0; i < fc->num_slots; i++)
            if(fc->slots[i]) free(fc->slots[i]);
        free(fc->slots);
        fc->slots = NULL;
    }
    if(fc->slot_page) { free(fc->slot_page); fc->slot_page = NULL; }
    if(fc->slot_use) { free(fc->slot_use); fc->slot_use = NULL; }
    if(fc->file) {
        storage_file_close(fc->file);
        storage_file_free(fc->file);
        fc->file = NULL;
    }
    fc->num_slots = 0;
}

/* Keep LRU timestamps ordered after long sessions. A wrap to zero would make
 * freshly used pages look oldest and cause avoidable SD reads. */
static void fc_lru_renormalize(FlashCache* fc) {
    for(uint16_t i = 0; i < fc->num_slots; i++) {
        uint32_t rank = 1;
        for(uint16_t j = 0; j < fc->num_slots; j++) {
            if(fc->slot_use[j] < fc->slot_use[i] ||
               (fc->slot_use[j] == fc->slot_use[i] && j < i)) {
                rank++;
            }
        }
        fc->slot_use[i] = rank;
    }
    fc->use_counter = fc->num_slots;
}

static uint32_t fc_lru_next(FlashCache* fc) {
    if(fc->use_counter == UINT32_MAX) fc_lru_renormalize(fc);
    return ++fc->use_counter;
}

/* Streams `page` into the LRU victim slot and returns its index. Eviction is
 * plain LRU (oldest slot_use). Unlike FlipperGB there are NO mapped-pointer
 * invariants to protect here: every flash access resolves a single byte and
 * returns immediately (the core copies the byte out; it never holds a page
 * pointer across instructions), so any slot except the just-loaded ones is a
 * safe victim. Faithful port of FlipperGB's rc_fill LRU core. */
static uint16_t fc_fill(FlashCache* fc, uint32_t page) {
    uint16_t lru = 0;
    uint32_t lru_use = 0xFFFFFFFFu;
    for(uint16_t i = 0; i < fc->num_slots; i++) {
        if(fc->slot_use[i] < lru_use) {
            lru_use = fc->slot_use[i];
            lru = i;
        }
    }
    if(!fc_read_page(fc, page, fc->slots[lru])) {
        /* IO fault: fill with open-bus so we never serve stale data */
        memset(fc->slots[lru], 0xFF, FLASH_PAGE_SIZE);
    }
    fc->slot_page[lru] = page;
    fc->slot_use[lru] = fc_lru_next(fc);
    return lru;
}

/* Returns a pointer to the 4KB slot that holds `page`, loading it on a miss.
 * O(1) when fully resident; otherwise a short linear scan + LRU fill. */
static const uint8_t* fc_page_ptr(FlashCache* fc, uint32_t page) {
    if(fc->fully_resident) return fc->slots[page]; /* slot i == page i */
    for(uint16_t i = 0; i < fc->num_slots; i++) {
        if(fc->slot_page[i] == page) {
            fc->slot_use[i] = fc_lru_next(fc);
            return fc->slots[i];
        }
    }
    return fc->slots[fc_fill(fc, page)];
}

/* Reads up to `size` (1/2/4) bytes of emulated flash at `addr` through the cache.
 * Bytes at/above flash_used (the unused 0xFF region) are returned as 0xFF WITHOUT
 * touching the cache. Handles the (rare) page-straddling access byte-by-byte. */
static uint32_t fc_read(FlashCache* fc, uint32_t addr, int size) {
    uint32_t v = 0;
    for(int i = 0; i < size; i++) {
        uint32_t a = addr + (uint32_t)i;
        uint8_t b;
        if(a >= fc->flash_used) {
            b = 0xFF; /* open-bus: unused flash, do not pollute the cache */
        } else {
            uint32_t page = a / FLASH_PAGE_SIZE;
            uint32_t poff = a % FLASH_PAGE_SIZE;
            const uint8_t* p = fc_page_ptr(fc, page);
            b = p[poff];
        }
        v |= (uint32_t)b << (8 * i);
    }
    return v;
}

/* AGNOSTIC: there is only ONE hardware profile now. The SAME profile runs ANY
 * .hex for this MCU, so we always return PROFILE_PANDORA_HW regardless of the
 * firmware (the knock-code, which IS firmware-specific, is handled separately
 * as a GPIO autotest, not by the profile). (void)path keeps the signature. */
static const ArmProfile* pick_profile(const char* path) {
    (void)path;
    return &PROFILE_PANDORA_HW;
}

/* ------------------------------------------------------------ CC1101 RF */

/* CC1101 OOK/ASK preset (650kHz BW, async serial). Register pairs {addr,val}
 * followed by the 8-byte PATABLE. Identical to the Flipper's
 * subghz_device_cc1101_preset_ook_650khz_async_regs. Used for Pandora OOK
 * firmwares and the vast majority of OOK sub-GHz remotes. */
static const uint8_t PRESET_OOK650[] = {
    0x02, 0x0D, 0x07, 0x04, 0x08, 0x32, 0x0B, 0x06, 0x10, 0xF8, 0x11, 0x32, 0x12, 0x30,
    0x14, 0x00, 0x15, 0x00, 0x18, 0x18, 0x19, 0x16, 0x1B, 0x07, 0x1C, 0x00, 0x1D, 0x91,
    0x20, 0xFB, 0x21, 0xB6, 0x22, 0x11, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00,
};

/* CC1101 2-FSK preset (async serial). Register pairs {addr,val} + 8-byte PATABLE.
 * Derived from the Flipper's subghz_device_cc1101_preset_2fsk_dev47_6khz_async_regs
 * (deviation ~47.6 kHz, RxBW 270.8 kHz, data rate ~4.8 kBaud). This is a
 * best-effort generic 2-FSK/GFSK front-end: the CC1101 cannot reproduce the exact
 * Si4432 deviation/data-rate programmed by every firmware, but it switches the
 * modem into FSK demod/mod so FSK Pandora variants are heard/sent instead of
 * being silently decoded as OOK. The wide deviation/BW tolerates the most
 * variants. Register meanings:
 *   0x02 IOCFG0=0x0D (async serial data in/out), 0x0B FSCTRL1=0x06,
 *   0x08 PKTCTRL0=0x32 (async continuous, no whitening), 0x07 PKTCTRL1=0x04,
 *   0x14 MDMCFG0=0x00, 0x13 MDMCFG1=0x02, 0x12 MDMCFG2=0x04 (2-FSK, no sync),
 *   0x11 MDMCFG3=0x83, 0x10 MDMCFG4=0x67, 0x15 DEVIATN=0x47 (~47.6 kHz),
 *   0x18 MCSM0=0x18, 0x19 FOCCFG=0x16, 0x1D AGCCTRL0=0x91, 0x1C AGCCTRL1=0x00,
 *   0x1B AGCCTRL2=0x07, 0x20 WORCTRL=0xFB, 0x22 FREND0=0x10, 0x21 FREND1=0x56. */
static const uint8_t PRESET_2FSK[] = {
    0x02, 0x0D, 0x0B, 0x06, 0x08, 0x32, 0x07, 0x04, 0x14, 0x00, 0x13, 0x02, 0x12, 0x04,
    0x11, 0x83, 0x10, 0x67, 0x15, 0x47, 0x18, 0x18, 0x19, 0x16, 0x1D, 0x91, 0x1C, 0x00,
    0x1B, 0x07, 0x20, 0xFB, 0x22, 0x10, 0x21, 0x56, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
};

/* Select and (re)load the CC1101 modulation preset that matches what the firmware
 * programmed into the Si4432 modulation type (REG[0x71] modtyp):
 *   00 unmodulated / 01 OOK -> OOK preset,
 *   10 FSK / 11 GFSK        -> 2-FSK preset (closest the CC1101 async modem can do).
 * The preset is reloaded ONLY when it changes AND the radio is idle. Loading a
 * custom preset drives the SPI bus and resets the modem, so callers must stop any
 * active async RX/TX and go idle first (handled by rf_reapply_preset). Returns
 * true if the loaded preset changed. */
static bool cc_preset_for_modtyp(uint8_t modtyp) {
    /* returns true if the desired preset is 2-FSK */
    return (modtyp == 2 || modtyp == 3);
}

/* The CC1101 exposes three useful async 2-FSK presets in this firmware tree.
 * Pick the closest available deviation to the value programmed by the Si4432. */
static uint8_t cc1101_fsk_deviation_reg(uint32_t requested_hz) {
    static const uint32_t deviations[] = {2380u, 12695u, 47607u};
    static const uint8_t registers[] = {0x04u, 0x30u, 0x47u};
    if(requested_hz == 0) requested_hz = deviations[2];
    uint8_t best = registers[0];
    uint32_t best_error = UINT32_MAX;
    for(size_t i = 0; i < sizeof(deviations) / sizeof(deviations[0]); i++) {
        uint32_t error = requested_hz > deviations[i] ?
                             requested_hz - deviations[i] : deviations[i] - requested_hz;
        if(error < best_error) {
            best_error = error;
            best = registers[i];
        }
    }
    return best;
}

static void radio_init(AppState* app) {
    app->frequency = 433920000; /* default / fallback until the firmware sets it */
    furi_hal_subghz_reset();
    furi_hal_subghz_load_custom_preset(PRESET_OOK650);
    furi_hal_subghz_set_frequency_and_path(app->frequency);
    furi_hal_subghz_idle();
    app->radio_on = true;
    app->rf_mode = RfModeOff;
    /* Default to "RF follows firmware": the CC1101 mode and frequency track the
     * Si4432 the firmware programs over SPI. Manual hold-BACK cycling is a
     * diagnostic override (it clears this flag). */
    app->rf_follow = true;
    app->si_modtyp = 1; /* assume OOK until the firmware says otherwise */
    app->si_dtmod = DtModDirectGpio; /* assume bit-bang until REG[0x71] says FIFO */
    app->cc_preset = CcPresetOok;
    app->cc_fsk_deviation_reg = 0;
    FURI_LOG_I(TAG, "CC1101 init OOK650 @%lu Hz (RF follows firmware)",
               (unsigned long)app->frequency);
}

/* Re-apply the CC1101 modulation preset to match the current Si4432 modtyp. Must
 * be called with the radio IDLE-able; it stops any active async RX/TX, goes idle,
 * loads the preset, restores the frequency, and leaves the radio idle. The caller
 * is responsible for restarting RX/TX afterwards if needed. This is invoked right
 * before starting a TX or RX so the radio is always in the modulation the
 * firmware asked for. */
static void rf_reapply_preset(AppState* app) {
    if(!app->radio_on) return;
    bool want_fsk = cc_preset_for_modtyp(app->si_modtyp);
    CcPreset want = want_fsk ? CcPreset2Fsk : CcPresetOok;
    uint8_t deviation_reg = cc1101_fsk_deviation_reg(app->si_devi_hz);
    if(want == app->cc_preset &&
       (!want_fsk || deviation_reg == app->cc_fsk_deviation_reg))
        return; /* already loaded: nothing to do */

    /* Loading a custom preset must happen on an idle radio. */
    if(app->tx_active) {
        furi_hal_subghz_stop_async_tx();
        app->tx_active = false;
    }
    if(app->rx_running) {
        furi_hal_subghz_stop_async_rx();
        /* keep app->rx_running as-is: the caller decides whether to restart */
    }
    furi_hal_subghz_idle();
    if(want_fsk) {
        uint8_t preset[sizeof(PRESET_2FSK)];
        memcpy(preset, PRESET_2FSK, sizeof(preset));
        for(size_t i = 0; preset[i] != 0; i += 2) {
            if(preset[i] == 0x15) {
                preset[i + 1] = deviation_reg;
                break;
            }
        }
        furi_hal_subghz_load_custom_preset(preset);
    } else {
        furi_hal_subghz_load_custom_preset(PRESET_OOK650);
    }
    furi_hal_subghz_set_frequency_and_path(app->frequency);
    furi_hal_subghz_idle();
    app->cc_preset = want;
    app->cc_fsk_deviation_reg = want_fsk ? deviation_reg : 0;
    static const char* const MODN[4] = {"OOK(unmod)", "OOK", "FSK", "GFSK"};
    FURI_LOG_I(TAG,
               "RF: modulation %s (REG71=0x%02X devi=%lu Hz) -> CC1101 preset %s",
               MODN[app->si_modtyp & 3], app->si.regs[0x71],
               (unsigned long)app->si_devi_hz,
               want_fsk ? (deviation_reg == 0x04 ? "2FSK/2.38k" :
                           deviation_reg == 0x30 ? "2FSK/12.7k" : "2FSK/47.6k") :
                          "OOK650");
}

/* --------------------------------------------- Si4432 -> CC1101 freq mapping
 * Si4432 Rev B1 datasheet, "Frequency Control" (REG 0x75..0x77):
 *   fb    = REG75 & 0x1F            (frequency band, 5 bits)
 *   hbsel = (REG75 >> 5) & 1        (high band select; 0: low, 1: high band)
 *   fc    = (REG76 << 8) | REG77    (nominal carrier, 16 bits)
 * Carrier:
 *   fcarrier = 10 MHz * (hbsel + 1) * (fb + 24 + fc/64000)
 * i.e. low band  (hbsel=0): 240..480 MHz in steps,
 *      high band (hbsel=1): 480..960 MHz.
 * We compute in Hz with integer math (the fc/64000 term expanded):
 *   fHz = 10e6*(hbsel+1)*(fb+24) + 10e6*(hbsel+1)*fc/64000
 *       = (hbsel+1) * ( (fb+24)*10000000 + fc*(10000000/64000) )
 *   10000000/64000 = 156.25 -> use fc*15625/100 to keep precision. */
static uint32_t si4432_regs_to_hz(uint8_t reg75, uint8_t reg76, uint8_t reg77) {
    uint32_t fb = (uint32_t)(reg75 & 0x1F);
    uint32_t hbsel = (uint32_t)((reg75 >> 5) & 1u);
    uint32_t fc = ((uint32_t)reg76 << 8) | (uint32_t)reg77;
    /* base term: (fb+24) * 10 MHz */
    uint64_t base = (uint64_t)(fb + 24u) * 10000000ull;
    /* fc term: fc * 156.25 Hz = fc * 15625 / 100 */
    uint64_t fcterm = ((uint64_t)fc * 15625ull) / 100ull;
    uint64_t hz = (base + fcterm) * (uint64_t)(hbsel + 1u);
    return (uint32_t)hz;
}

/* Clamp to a CC1101 band the Flipper supports (300-348, 387-464, 779-928 MHz).
 * Returns true if the value falls inside a valid band. */
static bool cc1101_band_ok(uint32_t hz) {
    return (hz >= 300000000u && hz <= 348000000u) ||
           (hz >= 387000000u && hz <= 464000000u) ||
           (hz >= 779000000u && hz <= 928000000u);
}

/* Re-map the CC1101 carrier from the firmware's current Si4432 REG[0x75..0x77].
 * Keeps the last valid frequency (default 433.92 MHz) if the computed value is
 * out of band or the registers are still zero (firmware has not programmed the
 * synth yet -> do NOT disturb the boot default). Re-tunes the CC1101 live if a
 * receive/transmit is active. */
static void rf_follow_apply_freq(AppState* app) {
    uint8_t r75 = app->si.regs[0x75];
    uint8_t r76 = app->si.regs[0x76];
    uint8_t r77 = app->si.regs[0x77];
    if(r75 == 0 && r76 == 0 && r77 == 0) return; /* synth not programmed yet */
    uint32_t hz = si4432_regs_to_hz(r75, r76, r77);
    if(!cc1101_band_ok(hz)) {
        FURI_LOG_W(TAG,
                   "RF: firmware freq %lu Hz out of CC1101 band; keeping %lu Hz "
                   "(REG75=0x%02X 76=0x%02X 77=0x%02X)",
                   (unsigned long)hz, (unsigned long)app->frequency, r75, r76, r77);
        return;
    }
    if(hz == app->frequency) return; /* no change */
    app->frequency = hz;
    FURI_LOG_I(TAG, "RF: firmware set freq -> %lu Hz (REG75=0x%02X 76=0x%02X 77=0x%02X)",
               (unsigned long)hz, r75, r76, r77);
    /* Re-tune live if the radio is mid-RX so the new carrier takes effect.
     * Direct-mode RX only (async serial); no FIFO/packet receiver. */
    if(app->radio_on && app->rx_running) {
        furi_hal_subghz_stop_async_rx();
        furi_hal_subghz_idle();
        furi_hal_subghz_set_frequency_and_path(app->frequency);
        furi_hal_subghz_start_async_rx(radio_rx_callback, app);
    } else if(app->radio_on) {
        furi_hal_subghz_idle();
        furi_hal_subghz_set_frequency_and_path(app->frequency);
    }
}

/* Map the firmware's Si4432 operating mode (REG[0x07]) onto the CC1101 when the
 * "follow firmware" bridge is active. The firmware driving its Si4432 into RX is
 * what makes the Flipper start receiving real air traffic AUTOMATICALLY, no
 * manual hold-BACK needed.
 *   new_mode: 3=TX, 2=RX, 1=READY, 0=IDLE (from si4432_write). */
static void rf_follow_apply_mode(AppState* app, int new_mode) {
    if(!app->rf_follow || !app->radio_on) return;
    /* DIRECT MODE ONLY (DYN_HW_D605.md): both firmwares bit-bang the waveform on
     * a GPIO pin and demodulate RX by software. There is no FIFO/packet path.
     * RX -> inject into PB0 (demod ISR); TX -> capture PB3 edges and replay. */
    switch(new_mode) {
    case 2: /* RX */
        /* AGNOSTIC: direct/bit-bang RX by injecting edges on PB0 + TIMER1. Every
         * firmware of this keyfob uses direct mode, so RX is always available
         * (no per-firmware rf_sw_demod / demod-ISR address). */
        app->rf_mode = RfModeRx;
        if(!app->rx_running) {
            rf_reapply_preset(app); /* match modulation before RX */
            rx_bridge_start(app);
            if(app->rx_running)
                FURI_LOG_I(TAG, "RF: CC1101 RX started @%lu Hz (firmware rf_rx_on)",
                           (unsigned long)app->frequency);
        }
        break;
    case 3: /* TX */
        if(app->rx_running) rx_bridge_stop(app);
        app->rf_mode = RfModeTx;
        /* Direct/bit-bang TX: capture the PB3 TX-DATA pin edges and replay them. */
        app->tx_head = app->tx_tail = 0;
        app->tx_edges_frame = 0;
        rf_reapply_preset(app); /* match modulation before TX */
        tx_on_si_mode(app, 3); /* begin capturing PB3 edges now */
        break;
    case 1: /* READY */
    case 0: /* IDLE */
    default:
        if(app->rx_running) rx_bridge_stop(app);
        if(app->tx_capturing || app->tx_active ||
           app->tx_head != app->tx_tail) {
            /* A TX frame is being captured / is pending / is in flight. Close the
             * capture but STAY in RfModeTx so tx_bridge_flush can replay & idle
             * the CC1101 when async_tx completes. Do not force idle here. */
            if(app->tx_capturing) tx_on_si_mode(app, new_mode);
            /* rf_mode left as RfModeTx on purpose */
        } else {
            app->rf_mode = RfModeOff;
            furi_hal_subghz_idle();
        }
        break;
    }
}

static void radio_deinit(AppState* app) {
    if(!app->radio_on) return;
    if(app->tx_active) {
        furi_hal_subghz_stop_async_tx();
        app->tx_active = false;
    }
    if(app->rx_running) rx_bridge_stop(app); /* direct-mode RX only */
    furi_hal_subghz_idle();
    furi_hal_subghz_sleep();
    app->radio_on = false;
    FURI_LOG_I(TAG, "CC1101 sleep");
}

/* ===================================================================== TX bridge */

/* Called when the firmware changes the Si4432 mode (REG[0x07]). On TX we start
 * capturing the DATA pin edges; when leaving TX we close the captured frame.
 * Replica of PandoraPIC's rf_tx_on detection. */
static void tx_on_si_mode(AppState* app, int new_mode) {
    if(app->rf_mode != RfModeTx) return; /* we only capture if the overlay is in TX */
    if(new_mode == 3) { /* TX */
        app->tx_capturing = true;
        /* TX waveform is bit-banged on PB3 (tx_data pin), not PB0 (RX-data). */
        app->tx_last_level =
            (app->gpio_dout[app->prof->tx_data_port] >> app->prof->tx_data_pin) & 1u;
        app->tx_last_insn = emu_insn_now(app);
        app->tx_edges_frame = 0;
        app->tx_edges_dropped = 0;
        FURI_LOG_I(TAG, "RF: firmware rf_tx_on -> capturing TX-DATA pin (B%u.%u)",
                   app->prof->tx_data_port, app->prof->tx_data_pin);
    } else { /* IDLE/READY/RX: end of the frame */
        if(app->tx_capturing) {
            /* The last GPIO level remains on the pin after its final toggle.
             * Include that trailing mark/space so replay does not truncate the
             * frame at the final edge. */
            uint64_t elapsed = emu_insn_now(app) - app->tx_last_insn;
            uint64_t scaled = (elapsed * ARM_INSN_US_NUM) / ARM_INSN_US_DEN;
            uint32_t dur_us = scaled > UINT32_MAX ? UINT32_MAX : (uint32_t)scaled;
            if(dur_us == 0) dur_us = 1;
            uint32_t head = app->tx_head;
            uint32_t next = (head + 1) % TX_RING_LEN;
            if(next != app->tx_tail) {
                app->tx_ring[head].level = app->tx_last_level;
                app->tx_ring[head].duration = dur_us;
                app->tx_head = next;
                app->tx_edges_frame++;
            } else {
                app->tx_edges_dropped++;
            }
            app->tx_capturing = false;
            FURI_LOG_I(TAG, "RF: firmware TX end (%lu edges captured, %lu dropped)",
                       (unsigned long)app->tx_edges_frame,
                       (unsigned long)app->tx_edges_dropped);
        }
    }
}

/* Captures an edge of the firmware output DATA pin: accumulates the level that
 * JUST ended with its duration (ninsn between toggles) into the TX ring. */
static void tx_capture_data_edge(AppState* app, bool level) {
    if(!app->tx_capturing) return;
    if(level == app->tx_last_level) return; /* no edge */
    uint64_t now = emu_insn_now(app);
    uint64_t dins = now - app->tx_last_insn;
    app->tx_last_insn = now;
    uint64_t scaled = (dins * ARM_INSN_US_NUM) / ARM_INSN_US_DEN;
    uint32_t dur_us = scaled > UINT32_MAX ? UINT32_MAX : (uint32_t)scaled;
    if(dur_us == 0) dur_us = 1;
    uint32_t head = app->tx_head;
    uint32_t next = (head + 1) % TX_RING_LEN;
    if(next != app->tx_tail) {
        app->tx_ring[head].level = app->tx_last_level;
        app->tx_ring[head].duration = dur_us;
        app->tx_head = next;
        app->tx_edges_frame++;
    } else {
        app->tx_edges_dropped++;
    }
    app->tx_last_level = level;
}

/* async_tx callback: drains the ring of captured edges. */
static LevelDuration radio_tx_callback(void* context) {
    AppState* app = (AppState*)context;
    uint32_t tail = app->tx_tail;
    if(tail == app->tx_head) return level_duration_reset();
    RfEdge e = app->tx_ring[tail];
    app->tx_tail = (tail + 1) % TX_RING_LEN;
    return level_duration_make(e.level, e.duration);
}

/* Starts/finishes the replay of the accumulated edges. Main loop. */
static void tx_bridge_flush(AppState* app) {
    if(app->rf_mode != RfModeTx || !app->radio_on) return;
    if(app->tx_active) {
        if(furi_hal_subghz_is_async_tx_complete()) {
            furi_hal_subghz_stop_async_tx();
            furi_hal_subghz_idle();
            app->tx_active = false;
            app->tx_frames++;
            FURI_LOG_I(TAG, "TX frame #%lu sent", (unsigned long)app->tx_frames);
        }
        return;
    }
    uint32_t pending = (app->tx_head + TX_RING_LEN - app->tx_tail) % TX_RING_LEN;
    if(pending == 0) return;
    if(app->tx_capturing) return; /* frame still being captured: wait for the close */
    if(!furi_hal_subghz_is_tx_allowed(app->frequency)) {
        FURI_LOG_E(TAG, "TX NOT allowed @%lu Hz; dropping %lu edges",
                   (unsigned long)app->frequency, (unsigned long)pending);
        app->tx_tail = app->tx_head;
        return;
    }
    /* log the frame (first edges: level+duration). */
    FURI_LOG_I(TAG, "TX async start: %lu edges", (unsigned long)pending);
    {
        char line[96];
        int off = 0;
        uint32_t t = app->tx_tail;
        for(int i = 0; i < 8 && t != app->tx_head; i++) {
            RfEdge e = app->tx_ring[t];
            int w = snprintf(line + off, sizeof(line) - (size_t)off, "%c%lu ",
                             e.level ? 'H' : 'L', (unsigned long)e.duration);
            if(w < 0 || (size_t)(off + w) >= sizeof(line)) break;
            off += w;
            t = (t + 1) % TX_RING_LEN;
        }
        FURI_LOG_D(TAG, "TX frame[0..7]: %s", line);
    }
    if(furi_hal_subghz_start_async_tx(radio_tx_callback, app)) {
        app->tx_active = true;
    } else {
        FURI_LOG_E(TAG, "start_async_tx failed");
        app->tx_tail = app->tx_head;
    }
}

/* ===================================================================== RX bridge */

/* Publish every captured pulse from the CC1101 ISR without filtering/blocking. */
static void radio_rx_enqueue(AppState* app, const RfEdge* e) {
    uint32_t head = __atomic_load_n(&app->rx_head, __ATOMIC_RELAXED);
    uint32_t tail = __atomic_load_n(&app->rx_tail, __ATOMIC_ACQUIRE);
    uint16_t capacity = app->rx_ring_capacity;
    if(!capacity || (uint32_t)(head - tail) >= capacity) {
        __atomic_fetch_add(&app->rx_dropped, 1u, __ATOMIC_RELAXED);
        return;
    }
    uint16_t slot = (uint16_t)(head % capacity);
    app->rx_ring_duration[slot] = (uint16_t)e->duration;
    uint8_t flags = (e->level ? 1u : 0u) |
                    (__atomic_load_n(&app->rx_gate_open_isr, __ATOMIC_RELAXED) ? 2u : 0u);
    uint8_t shift = (uint8_t)((slot & 3u) * 2u);
    uint8_t* packed = &app->rx_ring_flags[slot >> 2u];
    *packed = (uint8_t)((*packed & (uint8_t)~(3u << shift)) | (flags << shift));
    __atomic_store_n(&app->rx_head, head + 1u, __ATOMIC_RELEASE);
}

static void rx_preroll_push(AppState* app, const RxEdge* edge);

/* Paired with radio_rx_enqueue's release store. Avoids the RTOS queue/event
 * loop notification on every captured edge in the high-rate TIM2 ISR. */
static bool radio_rx_dequeue(AppState* app, RxEdge* e) {
    uint32_t tail = __atomic_load_n(&app->rx_tail, __ATOMIC_RELAXED);
    uint32_t head = __atomic_load_n(&app->rx_head, __ATOMIC_ACQUIRE);
    if(tail == head) return false;
    uint16_t slot = (uint16_t)(tail % app->rx_ring_capacity);
    uint8_t shift = (uint8_t)((slot & 3u) * 2u);
    uint8_t flags = (uint8_t)((app->rx_ring_flags[slot >> 2u] >> shift) & 3u);
    e->duration = app->rx_ring_duration[slot];
    e->level = flags & 1u;
    e->gate_open = (flags >> 1u) & 1u;
    __atomic_store_n(&app->rx_tail, tail + 1u, __ATOMIC_RELEASE);
    return true;
}

/* CC1101 callback runs in the capture IRQ. RSSI is sampled by the emulator
 * thread; the IRQ only snapshots the gate state and appends the edge. */
static void radio_rx_callback(bool level, uint32_t duration, void* context) {
    AppState* app = (AppState*)context;
    RfEdge e = {.level = level, .duration = duration};
    __atomic_fetch_add(&app->rx_events, 1u, __ATOMIC_RELAXED);
    bool gate = __atomic_load_n(&app->rx_gate_open_isr, __ATOMIC_ACQUIRE) != 0;
    if(gate) {
        radio_rx_enqueue(app, &e);
        return;
    }

    /* After RSSI closes, forward a bounded 100 ms tail. Idle noise stays in
     * the tiny pre-roll buffer and cannot fill the main capture ring. */
    uint32_t post = __atomic_load_n(&app->rx_postroll_isr_us, __ATOMIC_RELAXED);
    if(post) {
        radio_rx_enqueue(app, &e);
        post = duration >= post ? 0u : post - duration;
        __atomic_store_n(&app->rx_postroll_isr_us, post, __ATOMIC_RELAXED);
    }

    if(!__atomic_load_n(&app->rx_preroll_frozen, __ATOMIC_ACQUIRE)) {
        RxEdge compact = {
            .duration = (uint16_t)duration,
            .level = level ? 1u : 0u,
            .gate_open = 0u,
        };
        rx_preroll_push(app, &compact);
        __atomic_fetch_add(&app->rx_idle_edges, 1u, __ATOMIC_RELAXED);
    }
}

/* RSSI opens/closes a software squelch around the raw edge stream. Keep this
 * out of the capture IRQ: get_rssi() performs an SPI transaction. */
static void rx_rssi_update(AppState* app) {
    if(!app->rx_running) return;
    uint32_t now = furi_get_tick();
    if(app->rx_rssi_sample_tick &&
       (uint32_t)(now - app->rx_rssi_sample_tick) < RX_RSSI_POLL_MS)
        return;
    app->rx_rssi_sample_tick = now;
    app->rx_rssi_dbm = furi_hal_subghz_get_rssi();

    bool gate_open = __atomic_load_n(&app->rx_gate_open_isr, __ATOMIC_RELAXED) != 0;
    if(!app->rx_rssi_initialized) {
        /* Seed from the actual idle environment instead of assuming a fixed
         * noise floor. Subsequent samples decide whether a signal rose above it. */
        app->rx_noise_floor_dbm = app->rx_rssi_dbm;
        app->rx_rssi_initialized = true;
        app->rx_gate_open_dbm = app->rx_noise_floor_dbm + RX_RSSI_NOISE_MARGIN_DB;
        if(app->rx_gate_open_dbm < RX_RSSI_MIN_OPEN_DBM)
            app->rx_gate_open_dbm = RX_RSSI_MIN_OPEN_DBM;
        FURI_LOG_I(TAG, "RX RSSI baseline rssi=%.1f open=%.1f dBm",
                   (double)app->rx_rssi_dbm, (double)app->rx_gate_open_dbm);
        return;
    }
    if(!gate_open) {
        /* Follow a changing noise floor slowly; a signal peak must not raise it
         * quickly enough to make the rest of that burst disappear. */
        float delta = app->rx_rssi_dbm - app->rx_noise_floor_dbm;
        app->rx_noise_floor_dbm += delta * (delta < 0.0f ? 0.20f : 0.015f);
    }

    float open_threshold = app->rx_noise_floor_dbm + RX_RSSI_NOISE_MARGIN_DB;
    if(open_threshold < RX_RSSI_MIN_OPEN_DBM) open_threshold = RX_RSSI_MIN_OPEN_DBM;
    app->rx_gate_open_dbm = open_threshold;

    if(!gate_open) {
        if(app->rx_rssi_dbm >= open_threshold) {
            app->rx_gate_below_tick = 0;
            app->rx_gate_open_count++;
            app->rx_gate_drop_snapshot =
                __atomic_load_n(&app->rx_dropped, __ATOMIC_RELAXED);
            __atomic_store_n(&app->rx_gate_open_isr, 1u, __ATOMIC_RELEASE);
            __atomic_store_n(&app->rx_preroll_frozen, 1u, __ATOMIC_RELEASE);
            FURI_LOG_I(TAG,
                       "RX RSSI gate OPEN rssi=%.1f floor=%.1f threshold=%.1f preroll=%u",
                       (double)app->rx_rssi_dbm, (double)app->rx_noise_floor_dbm,
                       (double)open_threshold, (unsigned)app->rx_preroll_count);
        }
    } else {
        float close_threshold = open_threshold - RX_RSSI_HYSTERESIS_DB;
        if(app->rx_rssi_dbm < close_threshold) {
            if(app->rx_gate_below_tick == 0) app->rx_gate_below_tick = now;
            if((uint32_t)(now - app->rx_gate_below_tick) >= RX_RSSI_HANG_MS) {
                __atomic_store_n(&app->rx_preroll_frozen, 1u, __ATOMIC_RELEASE);
                app->rx_preroll_head = 0;
                app->rx_preroll_count = 0;
                __atomic_store_n(&app->rx_postroll_isr_us, RX_POSTROLL_MS * 1000u,
                                 __ATOMIC_RELEASE);
                __atomic_store_n(&app->rx_gate_open_isr, 0u, __ATOMIC_RELEASE);
                __atomic_store_n(&app->rx_preroll_frozen, 0u, __ATOMIC_RELEASE);
                app->rx_gate_close_count++;
                app->rx_gate_below_tick = 0;
                FURI_LOG_I(TAG,
                           "RX RSSI gate CLOSE rssi=%.1f floor=%.1f threshold=%.1f hold=%u ms",
                           (double)app->rx_rssi_dbm, (double)app->rx_noise_floor_dbm,
                           (double)close_threshold, RX_RSSI_HANG_MS);
            }
        } else {
            app->rx_gate_below_tick = 0;
        }
    }
}

/* ------------------------------------------------------------ RX injection --
 *
 * AGNOSTIC RX injection, ported from the ESP32 emu_rx_inject_edge and
 * pandora_tui.py's hardware-faithful RF path. The edge is presented on the REAL
 * RX-data pin (PB0 = data_port.data_pin) and the pulse WIDTH is written to
 * TIMER1's capture CNT (rx_pulse_ticks), which the firmware's own demod reads
 * via TIMER1_CNT (mmio_read returns rx_pulse_ticks while rx_feeding). We DO NOT
 * call the firmware's demod ISR by address, and we DO NOT poke any profile RAM
 * offset (the old call_rx_demod_samples / rx_feed_spooled_edge / rx_prime did
 * both). The firmware detects/processes the edge through its OWN GPIO/EXTI +
 * TIMER1-capture path, driven by the agnostic NVIC scheduler (TIMER1 = IRQ12),
 * exactly like real hardware. Agnostic for ANY firmware -- and this is what
 * removes the ~20 per-firmware RX RAM addresses.
 *
 * duration_us -> CNT ticks: the Python core keeps ~6 ticks/us (HFPERCLK/presc
 * proxy); we mirror that. The level is held on PB0 until the next edge. A few
 * foreground bursts let the firmware poll/service the edge before returning. */
static void emu_rx_inject_edge(AppState* app, bool level, uint32_t duration_us) {
    const ArmProfile* p = app->prof;
    uint8_t port = p->data_port, pin = p->data_pin;

    /* present the edge on the real RX-data pin (DIN) */
    if(level)
        app->gpio_din[port] |= (uint16_t)(1u << pin);
    else
        app->gpio_din[port] &= (uint16_t)~(1u << pin);

    /* set TIMER1 capture CNT = pulse width (the demod reads TIMER1_CNT) */
    app->rx_pulse_ticks = (duration_us * 6u) & 0xFFFFu;
    app->rx_feeding = true;

    /* let the firmware's own foreground + TIMER1/GPIO IRQ path service the edge.
     * Pure time (bursts); no PC/address sync, no ISR-by-address. */
    for(int i = 0; i < 8; i++) run_burst(app, 20000u);

    app->rx_feeding = false;
    app->rx_prev_level = level;
    app->rx_injected++;
}

static void rx_preroll_push(AppState* app, const RxEdge* edge) {
    if(app->rx_preroll_count == (uint16_t)RX_PREROLL_LEN) {
        __atomic_fetch_add(&app->rx_preroll_overwritten, 1u, __ATOMIC_RELAXED);
    } else {
        app->rx_preroll_count++;
    }
    app->rx_preroll[app->rx_preroll_head] = *edge;
    app->rx_preroll_head = (uint16_t)((app->rx_preroll_head + 1u) % RX_PREROLL_LEN);
}

static bool rx_deliver_edge(AppState* app, const RxEdge* edge);

/* Preserve the player's exact next edge before reusing its RAM buffer for a
 * new spool writer. play_pos points past the whole loaded batch, so rewind by
 * the undelivered suffix and let playback refill after capture finishes. */
static bool rx_spool_suspend_player(AppState* app) {
    if(app->rx_spool_player < 0) return true;
    RxSpool* s = &app->rx_spool[(uint8_t)app->rx_spool_player];
    uint32_t unread = app->rx_spool_play_count - app->rx_spool_play_index;
    uint32_t unread_bytes = unread * sizeof(RxEdge);
    if(unread_bytes > s->play_pos) return false;
    uint32_t resume_pos = s->play_pos - unread_bytes;
    if(!storage_file_seek(s->file, resume_pos, true)) {
        FURI_LOG_E(TAG, "RX spool pause seek failed slot=%c at=%lu",
                   (char)('A' + app->rx_spool_player), (unsigned long)resume_pos);
        return false;
    }
    s->play_pos = resume_pos;
    app->rx_spool_play_count = 0;
    app->rx_spool_play_index = 0;
    return true;
}

static int8_t rx_spool_alloc(AppState* app) {
    FS_Error mkdir_err = storage_common_mkdir(app->storage, PA_DATA_DIR);
    if(mkdir_err != FSE_OK && mkdir_err != FSE_EXIST) {
        FURI_LOG_E(TAG, "RX spool SD directory unavailable");
        app->rx_spool_lost++;
        return -1;
    }
    for(uint8_t i = 0; i < RX_SPOOL_SLOTS; i++) {
        RxSpool* s = &app->rx_spool[i];
        if(s->state != RxSpoolFree) continue;
        if(!s->file) s->file = storage_file_alloc(app->storage);
        if(!s->file) break;
        snprintf(s->path, sizeof(s->path), PA_DATA_DIR "/rx_%c.bin", (char)('a' + i));
        if(!storage_file_open(s->file, s->path, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
            FURI_LOG_E(TAG, "RX spool open failed slot=%u", i);
            continue;
        }
        if(!rx_spool_suspend_player(app)) {
            storage_file_close(s->file);
            s->state = RxSpoolFree;
            app->rx_spool_lost++;
            return -1;
        }
        s->state = RxSpoolWriting;
        s->bytes = 0;
        s->play_pos = 0;
        s->drop_start = app->rx_gate_drop_snapshot;
        s->order = ++app->rx_spool_order;
        app->rx_spool_batch_count = 0;
        app->rx_spool_writer = (int8_t)i;
        app->rx_gate_sample_count = 0;
        FURI_LOG_I(TAG, "RX spool burst #%lu start slot=%c",
                   (unsigned long)app->rx_spool_bursts + 1u, (char)('A' + i));
        return (int8_t)i;
    }
    FURI_LOG_E(TAG, "RX spool unavailable (both SD slots busy); burst cannot be preserved");
    app->rx_spool_lost++;
    return -1;
}

static bool rx_spool_flush(AppState* app) {
    if(app->rx_spool_writer < 0 || app->rx_spool_batch_count == 0) return true;
    RxSpool* s = &app->rx_spool[(uint8_t)app->rx_spool_writer];
    size_t bytes = (size_t)app->rx_spool_batch_count * sizeof(RxEdge);
    if(storage_file_write(s->file, app->rx_spool_buffer, bytes) != bytes) {
        s->write_errors++;
        FURI_LOG_E(TAG, "RX spool write failed slot=%c at=%lu bytes=%lu",
                   (char)('A' + app->rx_spool_writer), (unsigned long)s->bytes,
                   (unsigned long)bytes);
        storage_file_close(s->file);
        s->state = RxSpoolFailed; /* leave partial file on SD for diagnosis */
        app->rx_spool_writer = -1;
        app->rx_spool_batch_count = 0;
        app->rx_capture_active = false;
        app->rx_capture_blocked = true;
        app->rx_spool_lost++;
        return false;
    }
    s->bytes += (uint32_t)bytes;
    app->rx_spool_batch_count = 0;
    return true;
}

static void rx_spool_finish(AppState* app) {
    if(app->rx_spool_writer < 0) return;
    int8_t idx = app->rx_spool_writer;
    if(!rx_spool_flush(app)) return;
    RxSpool* s = &app->rx_spool[(uint8_t)idx];
    storage_file_close(s->file);
    uint32_t dropped = __atomic_load_n(&app->rx_dropped, __ATOMIC_RELAXED) - s->drop_start;
    if(dropped) {
        s->state = RxSpoolFree; /* truncated waveform is retained until this slot is reused */
        app->rx_spool_writer = -1;
        app->rx_capture_blocked = true;
        app->rx_spool_lost++;
        FURI_LOG_E(TAG,
                   "RX spool burst incomplete slot=%c edges=%lu ring_dropped=%lu; retained, not replayed",
                   (char)('A' + idx), (unsigned long)(s->bytes / sizeof(RxEdge)),
                   (unsigned long)dropped);
        return;
    }
    s->state = RxSpoolReady;
    app->rx_spool_writer = -1;
    app->rx_spool_bursts++;
    FURI_LOG_I(TAG, "RX spool burst #%lu ready slot=%c edges=%lu bytes=%lu",
               (unsigned long)app->rx_spool_bursts, (char)('A' + idx),
               (unsigned long)(s->bytes / sizeof(RxEdge)), (unsigned long)s->bytes);
    if(app->rx_gate_sample_count) {
        char sample[96];
        size_t used = 0;
        for(uint8_t i = 0; i < app->rx_gate_sample_count; i++) {
            int written = snprintf(sample + used, sizeof(sample) - used,
                                   "%s%u:%u", i ? "," : "",
                                   app->rx_gate_sample_level[i],
                                   app->rx_gate_sample_duration[i]);
            if(written < 0 || (size_t)written >= sizeof(sample) - used) break;
            used += (size_t)written;
        }
        FURI_LOG_I(TAG, "RX waveform slot=%c gate-onset=%s (callback-level:duration-us)",
                   (char)('A' + idx), sample);
    }
}

static void rx_spool_append(AppState* app, const RxEdge* edge) {
    if(app->rx_spool_writer < 0) return;
    RxSpool* s = &app->rx_spool[(uint8_t)app->rx_spool_writer];
    uint32_t projected = s->bytes +
                         ((uint32_t)app->rx_spool_batch_count + 1u) * sizeof(RxEdge);
    if(projected > RX_SPOOL_MAX_BYTES) {
        FURI_LOG_E(TAG, "RX spool burst exceeded %u bytes; preserving partial and dropping burst",
                   RX_SPOOL_MAX_BYTES);
        (void)rx_spool_flush(app);
        if(app->rx_spool_writer >= 0) {
            storage_file_close(s->file);
            s->state = RxSpoolFailed; /* partial capture is not replayed as a complete burst */
            app->rx_spool_writer = -1;
        }
        app->rx_capture_active = false;
        app->rx_capture_blocked = true;
        app->rx_spool_lost++;
        return;
    }
    app->rx_spool_buffer[app->rx_spool_batch_count++] = *edge;
    if(app->rx_spool_batch_count == RX_SPOOL_BATCH) (void)rx_spool_flush(app);
}

static void rx_spool_start_playback(AppState* app) {
    if(app->rx_spool_player >= 0) return;
    int8_t best = -1;
    for(uint8_t i = 0; i < RX_SPOOL_SLOTS; i++) {
        RxSpool* s = &app->rx_spool[i];
        if(s->state == RxSpoolReady &&
           (best < 0 || (int32_t)(s->order - app->rx_spool[(uint8_t)best].order) < 0))
            best = (int8_t)i;
    }
    if(best < 0) return;
    RxSpool* s = &app->rx_spool[(uint8_t)best];
    if(!storage_file_open(s->file, s->path, FSAM_READ, FSOM_OPEN_EXISTING)) {
        FURI_LOG_E(TAG, "RX spool playback open failed slot=%c; file retained",
                   (char)('A' + best));
        s->state = RxSpoolFailed; /* preserve on disk; don't spin retrying an I/O error */
        app->rx_spool_lost++;
        return;
    }
    s->play_pos = 0;
    app->rx_spool_play_count = 0;
    app->rx_spool_play_index = 0;
    s->state = RxSpoolPlaying;
    app->rx_spool_player = best;
    FURI_LOG_I(TAG, "RX spool playback start slot=%c edges=%lu",
               (char)('A' + best), (unsigned long)(s->bytes / sizeof(RxEdge)));
    FURI_LOG_I(TAG, "RX replay path: clean widths -> inject on PB0 + TIMER1 "
                    "(AGNOSTIC; firmware's own GPIO/EXTI decoder)");
}

static void rx_spool_play_pump(AppState* app) {
    /* A new capture owns the shared buffer until its writer is closed. */
    if(app->rx_spool_writer >= 0) return;
    /* AGNOSTIC: a decoded frame is consumed by the firmware's own foreground RX
     * loop (we no longer poke a profile rx_pending RAM byte). Give run_burst() a
     * brief chance to redraw the UI before feeding the next frame; capture /
     * spooling continues independently in rx_bridge_pump(). */
    if(app->rx_reprime_pending) {
        run_burst(app, 40000u);
        app->rx_reprime_pending = false;
    }

    rx_spool_start_playback(app);
    if(app->rx_spool_player < 0) return;
    RxSpool* s = &app->rx_spool[(uint8_t)app->rx_spool_player];
    uint32_t remain = (s->bytes - s->play_pos) / sizeof(RxEdge);
    if(app->rx_spool_play_index == app->rx_spool_play_count && remain == 0) {
        int8_t idx = app->rx_spool_player;
        storage_file_close(s->file);
        if(storage_common_remove(app->storage, s->path) == FSE_OK) {
            FURI_LOG_I(TAG,
                       "RX spool playback complete slot=%c; SD file deleted",
                       (char)('A' + idx));
            app->rx_spool_replayed++;
            s->state = RxSpoolFree;
        } else {
            FURI_LOG_E(TAG, "RX spool delete failed slot=%c; file retained",
                       (char)('A' + idx));
            s->state = RxSpoolFailed;
            app->rx_spool_lost++;
        }
        app->rx_spool_player = -1;
        app->rx_spool_play_count = 0;
        app->rx_spool_play_index = 0;
        return;
    }
    if(app->rx_spool_play_index == app->rx_spool_play_count) {
        uint32_t count = remain < RX_SPOOL_BATCH ? remain : RX_SPOOL_BATCH;
        size_t nbytes = (size_t)count * sizeof(RxEdge);
        size_t got = storage_file_read(s->file, app->rx_spool_buffer, nbytes);
        if(got != nbytes) {
            int8_t idx = app->rx_spool_player;
            FURI_LOG_E(TAG, "RX spool playback read failed slot=%c at=%lu; file retained",
                       (char)('A' + idx), (unsigned long)s->play_pos);
            storage_file_close(s->file);
            s->state = RxSpoolFailed;
            app->rx_spool_player = -1;
            app->rx_spool_play_count = 0;
            app->rx_spool_play_index = 0;
            app->rx_spool_lost++;
            return;
        }
        s->play_pos += (uint32_t)got;
        app->rx_spool_play_count = (uint16_t)count;
        app->rx_spool_play_index = 0;
        /* AGNOSTIC: seed PB0 at the first edge's level before replaying; the
         * firmware's own GPIO/EXTI path samples it (no rx_prime RAM seeding). */
        if(app->rx_spool_player >= 0)
            emu_rx_inject_edge(app, app->rx_spool_buffer[0].level != 0, 1u);
    }
    uint16_t delivered = 0;
    while(app->rx_spool_play_index < app->rx_spool_play_count &&
          delivered < RX_PLAY_MAX_EDGES_PER_PUMP) {
        RxEdge* edge = &app->rx_spool_buffer[app->rx_spool_play_index++];
        delivered++;
        /* Let the emulated main loop observe each completed frame before feeding
         * later frames, as the physical radio ISR would. Keep this slice short:
         * the callback ring must be drained between replay slices or it
         * overflows while the firmware ISR is being emulated. */
        if(rx_deliver_edge(app, edge)) break;
    }
}

static bool rx_deliver_edge(AppState* app, const RxEdge* edge) {
    /* AGNOSTIC: inject the edge over the REAL RX-data pin (PB0) + TIMER1 CNT.
     * FuriHalSubGhzCaptureCallback reports true on a falling edge after the high
     * pulse, false on a rising edge after the low pulse; the firmware samples the
     * post-edge DATA level, so feed the opposite level. We do NOT call the demod
     * ISR by address and do NOT read/write any profile RX RAM byte -- the
     * firmware processes the edge via its own GPIO/EXTI + TIMER1 path. A decoded
     * frame is OBSERVABLE through the OLED redrawing (fb_hash change below). */
    uint32_t prev_hash = fb_hash(app);
    emu_rx_inject_edge(app, edge->level == 0, edge->duration);

    /* observe whether the firmware redrew the screen (frame accepted) */
    bool redrew = (fb_hash(app) != prev_hash);
    if(redrew) {
        app->rx_frames++;
        if(!app->rx_last_frame_valid) {
            FURI_LOG_I(TAG, "RX frame #%lu observed (OLED redrew)",
                       (unsigned long)app->rx_frames);
        }
        app->rx_last_frame_valid = true;
        app->rx_reprime_pending = true;
        return true; /* break the replay slice so the UI can settle */
    }
    return false;
}

static void rx_bridge_start(AppState* app) {
    if(app->rx_running || !app->radio_on) return;
    /* AGNOSTIC: every firmware of this keyfob uses the Si4432 in DIRECT mode
     * (bit-bang OOK/FSK). The emulator injects RX edges over PB0 + TIMER1, so RX
     * is always available -- no per-firmware rf_sw_demod flag. */
    if(app->tx_active) {
        furi_hal_subghz_stop_async_tx();
        app->tx_active = false;
    }
    furi_hal_subghz_idle();
    /* AGNOSTIC: no rx_prime RAM seeding; the DATA pin idles high (released) and
     * the firmware's own GPIO/EXTI path samples the injected edges. */
    app->rx_reprime_pending = false;
    app->rx_last_frame_valid = false;
    app->rx_preroll_count = 0;
    app->rx_preroll_head = 0;
    app->rx_capture_active = false;
    app->rx_capture_blocked = false;
    app->rx_capture_post_remaining_us = 0;
    app->rx_spool_batch_count = 0;
    app->rx_preroll_head = 0;
    app->rx_preroll_count = 0;
    app->rx_preroll_frozen = 0;
    app->rx_postroll_isr_us = 0;
    app->rx_gate_below_tick = 0;
    app->rx_rssi_sample_tick = 0;
    app->rx_rssi_initialized = false;
    app->rx_noise_floor_dbm = -105.0f;
    app->rx_gate_open_dbm = RX_RSSI_MIN_OPEN_DBM;
    __atomic_store_n(&app->rx_gate_open_isr, 0u, __ATOMIC_RELEASE);
    uint32_t queued = __atomic_load_n(&app->rx_head, __ATOMIC_ACQUIRE) -
                      __atomic_load_n(&app->rx_tail, __ATOMIC_ACQUIRE);
    furi_hal_subghz_start_async_rx(radio_rx_callback, app);
    app->rx_running = true;
    FURI_LOG_I(TAG,
               "RX bridge ON (AGNOSTIC: inject edges on PB0=B%u.%u + TIMER1, "
               "queued=%lu, RSSI gate)",
               app->prof->data_port, app->prof->data_pin, (unsigned long)queued);
}

static void rx_bridge_stop(AppState* app) {
    if(!app->rx_running) return;
    furi_hal_subghz_stop_async_rx();
    furi_hal_subghz_idle();
    app->rx_running = false;
    app->rx_primed = false;
    __atomic_store_n(&app->rx_gate_open_isr, 0u, __ATOMIC_RELEASE);
    app->rx_capture_active = false;
    app->rx_capture_post_remaining_us = 0;
    rx_spool_finish(app);
    /* Leave an in-progress playback file and cursor intact. The firmware
     * briefly switches READY while consuming each frame; RX resumes on the
     * next loop, where the same player continues from the next edge. */
    FURI_LOG_I(TAG, "RX bridge OFF (%lu edges injected, %lu frames)",
               (unsigned long)app->rx_injected, (unsigned long)app->rx_frames);
}

/* Drain capture into bounded SD bursts, then replay each burst to the firmware ISR. */
static void rx_bridge_pump(AppState* app, bool replay) {
    if(!app->rx_running) return;
    RxEdge e;
    uint32_t processed = 0;
    /* Empty the currently queued capture in one pass. Capping this at half the
     * ring left a large backlog while the SD spool was doing synchronous
     * writes, so the capture ISR could wrap and discard the rest of a burst. */
    uint32_t pump_limit = app->rx_ring_capacity;
    while(processed < pump_limit && radio_rx_dequeue(app, &e)) {
        processed++;
        if(e.gate_open) {
            if(!app->rx_capture_active && !app->rx_capture_blocked) {
                if(rx_spool_alloc(app) >= 0) {
                    app->rx_capture_active = true;
                    /* Keep the complete pulse history immediately before RSSI rose. */
                    uint16_t oldest = (uint16_t)((app->rx_preroll_head + RX_PREROLL_LEN -
                                                  app->rx_preroll_count) % RX_PREROLL_LEN);
                    for(uint16_t i = 0; i < app->rx_preroll_count; i++)
                        rx_spool_append(app, &app->rx_preroll[(oldest + i) % RX_PREROLL_LEN]);
                    app->rx_preroll_count = 0;
                } else {
                    app->rx_capture_blocked = true;
                }
            }
            app->rx_capture_post_remaining_us = 0;
            if(app->rx_capture_active) {
                if(app->rx_gate_sample_count < 8u) {
                    uint8_t sample = app->rx_gate_sample_count++;
                    app->rx_gate_sample_duration[sample] = e.duration;
                    app->rx_gate_sample_level[sample] = e.level;
                }
                rx_spool_append(app, &e);
            }
        } else if(app->rx_capture_active) {
            rx_spool_append(app, &e);
            if(app->rx_capture_active) {
                if(app->rx_capture_post_remaining_us == 0)
                    app->rx_capture_post_remaining_us = RX_POSTROLL_MS * 1000u;
                if(e.duration >= app->rx_capture_post_remaining_us) {
                    app->rx_capture_active = false;
                    app->rx_capture_post_remaining_us = 0;
                    rx_spool_finish(app);
                } else {
                    app->rx_capture_post_remaining_us -= e.duration;
                }
            }
        } else {
            app->rx_capture_blocked = false;
        }
    }
    if(replay) rx_spool_play_pump(app);
    if(processed) app->buzz_request = false;
}

/* Toggles the RF bridge: OFF -> TX -> RX -> OFF. Logged (like PandoraPIC). */
/* Manual hold-BACK override. Normally the bridge follows the firmware
 * (app->rf_follow). The first hold-BACK DROPS OUT of follow mode into a manual
 * diagnostic cycle: FOLLOW -> manual OFF -> TX -> RX -> FOLLOW (re-enables auto).
 * This preserves the pre-existing OFF/TX/RX cycling while making "follow the
 * firmware" the default and reachable again at the end of the cycle. */
static void rf_cycle_mode(AppState* app) {
    if(app->rf_follow) {
        /* leaving auto-follow: tear down whatever the firmware had running */
        app->rf_follow = false;
        if(app->rx_running) rx_bridge_stop(app);
        app->tx_capturing = false;
        if(app->tx_active) {
            furi_hal_subghz_stop_async_tx();
            app->tx_active = false;
        }
        app->rf_mode = RfModeOff;
        furi_hal_subghz_idle();
        FURI_LOG_I(TAG, "RF mode MANUAL/OFF (follow-firmware disabled)");
        return;
    }
    switch(app->rf_mode) {
    case RfModeOff:
        app->rf_mode = RfModeTx;
        app->tx_head = app->tx_tail = 0;
        app->tx_capturing = false;
        app->tx_edges_frame = 0;
        /* if the firmware is already in TX, start capturing immediately */
        if(app->si.mode == 3) tx_on_si_mode(app, 3);
        FURI_LOG_I(TAG, "RF mode TX (manual: capture firmware DATA pin)");
        break;
    case RfModeTx:
        app->tx_capturing = false;
        if(app->tx_active) {
            furi_hal_subghz_stop_async_tx();
            app->tx_active = false;
        }
        app->rf_mode = RfModeRx;
        rx_bridge_start(app);
        if(!app->rx_running) {
            /* profile without SW-demod: jump straight back to follow */
            app->rf_follow = true;
            app->rf_mode = RfModeOff;
            FURI_LOG_I(TAG, "RF mode FOLLOW (RX not supported on this profile)");
        } else {
            FURI_LOG_I(TAG, "RF mode RX (manual: inject into firmware demodulator)");
        }
        break;
    case RfModeRx:
    default:
        rx_bridge_stop(app);
        /* back to the default: let the firmware drive the radio again */
        app->rf_follow = true;
        app->rf_mode = RfModeOff;
        FURI_LOG_I(TAG, "RF mode FOLLOW (follow-firmware re-enabled)");
        break;
    }
}

/* ------------------------------------------------------------ overlay help */

/* In-emulator overlay (direct-draw): status + RF bridge control (Left). */
static void overlay_draw(AppState* app) {
    Canvas* c = app->canvas;
    canvas_reset(c);
    canvas_clear(c);
    canvas_set_font(c, FontPrimary);
    canvas_draw_str(c, 2, 10, "Pandora ARM");
    canvas_draw_line(c, 0, 12, 127, 12);
    canvas_set_font(c, FontSecondary);
    char buf[48];
    snprintf(buf, sizeof(buf), "FW:%s %s", app->prof ? app->prof->name : "?",
             app->unlocked ? "UNLOCKED" : "LOCKED");
    canvas_draw_str(c, 2, 24, buf);
    const char* rf = "OFF";
    if(app->rf_mode == RfModeTx) rf = "TX";
    if(app->rf_mode == RfModeRx) rf = "RX";
    /* show whether the firmware is driving the radio (FOLLOW) or manual */
    snprintf(buf, sizeof(buf), "RF:%s%s %luMHz", app->rf_follow ? "~" : "", rf,
             (unsigned long)(app->frequency / 1000000u));
    canvas_draw_str(c, 2, 34, buf);
    snprintf(buf, sizeof(buf), "TX:%lu RX:%lu", (unsigned long)app->tx_frames,
             (unsigned long)app->rx_frames);
    canvas_draw_str(c, 2, 44, buf);
    canvas_draw_str(c, 2, 56, "OK=cont Left=RF Back=exit");
    canvas_commit(c);
}

/* ============================================================ phase 1 menu */
/* view_dispatcher + submenu (NO number_input: the ARM uses a knock-code). Same
 * pattern as PandoraPIC: the dispatcher BLOCKS; the items set a flag and stop
 * the dispatcher; main opens the file browser / re-creates the menu / starts the
 * emulator. view_dispatcher and direct-draw NEVER coexist. */

/* Views (just one: the submenu is re-populated for main-menu / knock-builder) */
typedef enum {
    PandoraViewSubmenu,
    PandoraViewAbout,
} PandoraView;

/* Main menu items */
typedef enum {
    MenuItemFirmware,
    MenuItemKnock,
    MenuItemLaunch,
    MenuItemAbout,
} MenuItemMain;

/* Knock-code builder items (buttons + Done/Clear) */
typedef enum {
    KnockItemUp,
    KnockItemDown,
    KnockItemOk,
    KnockItemBack,
    KnockItemB5,
    KnockItemB6,
    KnockItemDone,
    KnockItemClear,
} KnockItem;

/* Flow requested when the dispatcher is stopped. */
typedef enum {
    FlowExit,
    FlowFileBrowser,
    FlowLaunch,
} MenuFlow;

static volatile MenuFlow g_menu_flow = FlowExit;

static const char* btn_name(uint8_t b) {
    switch(b) {
    case B_UP: return "UP";
    case B_DOWN: return "DOWN";
    case B_OK: return "OK";
    case B_BACK: return "BACK";
    case B5: return "B5";
    case B6: return "B6";
    default: return "?";
    }
}

/* Builds the string of the current knock sequence (profile default if the user
 * did not define any). */
/* Resolve the DEFAULT knock-code (FIRMWARE CONFIG) from the firmware file name.
 * Agnostic: the hardware does not carry a knock; this is only a convenience for
 * the two known builds. Unknown firmware -> no default (the user enters it in
 * the UI). Returns the sequence and sets *len / *gate. */
static const uint8_t* default_knock_for_name(const char* name, uint8_t* len, int* gate) {
    *gate = B5;
    if(name && (strstr(name, "MAX") || strstr(name, "max"))) {
        *len = (uint8_t)sizeof(KNOCK_PMAX);
        return KNOCK_PMAX;
    }
    if(name && (strstr(name, "2_5253") || strstr(name, "pandora") ||
                strstr(name, "PANDORA"))) {
        *len = (uint8_t)sizeof(KNOCK_PANDORA);
        return KNOCK_PANDORA;
    }
    *len = 0;
    *gate = -1;
    return NULL;
}

static void knock_seq_str(AppState* app, char* out, size_t n) {
    const uint8_t* seq;
    uint8_t len;
    bool is_default = (app->knock_user_len == 0);
    if(is_default) {
        int gate;
        seq = default_knock_for_name(app->fw_name, &len, &gate);
    } else {
        seq = app->knock_user;
        len = app->knock_user_len;
    }
    int off = 0;
    if(is_default) {
        int w = snprintf(out, n, "(def) ");
        off = (w > 0) ? w : 0;
    }
    for(uint8_t i = 0; i < len && seq; i++) {
        int w = snprintf(out + off, n - (size_t)off, "%s%s", btn_name(seq[i]),
                         (i + 1 < len) ? "," : "");
        if(w < 0 || (size_t)(off + w) >= n) break;
        off += w;
    }
    if(len == 0) snprintf(out, n, "<empty>");
}

static void menu_submenu_callback(void* context, uint32_t index);
static void knock_submenu_callback(void* context, uint32_t index);

/* (Re)builds the knock-builder submenu with the current sequence. */
static void knock_rebuild(AppState* app) {
    submenu_reset(app->submenu);
    char hdr[64];
    char seq[48];
    knock_seq_str(app, seq, sizeof(seq));
    snprintf(hdr, sizeof(hdr), "PIN: %s", seq);
    submenu_set_header(app->submenu, hdr);
    submenu_add_item(app->submenu, "UP", KnockItemUp, knock_submenu_callback, app);
    submenu_add_item(app->submenu, "DOWN", KnockItemDown, knock_submenu_callback, app);
    submenu_add_item(app->submenu, "OK", KnockItemOk, knock_submenu_callback, app);
    submenu_add_item(app->submenu, "BACK", KnockItemBack, knock_submenu_callback, app);
    submenu_add_item(app->submenu, "B5", KnockItemB5, knock_submenu_callback, app);
    submenu_add_item(app->submenu, "B6", KnockItemB6, knock_submenu_callback, app);
    submenu_add_item(app->submenu, "Done", KnockItemDone, knock_submenu_callback, app);
    submenu_add_item(app->submenu, "Clear", KnockItemClear, knock_submenu_callback, app);
}

/* About screen text (scrollable). Describes what the firmware is and the full
 * button mapping (verified against the physical keyfob). */
static const char* const ABOUT_TEXT =
    "\e#Pandora D-605 (ARM)\e#\n"
    "Emulator of the real Pandora D-605 car-alarm keyfob firmware (Silicon Labs "
    "EFM32, ARM Cortex-M3). The firmware runs unmodified; this app emulates the "
    "keyfob hardware: the OLED screen and the 6 physical buttons, and bridges the "
    "Flipper CC1101 for real RX/TX.\n"
    "\n"
    "Buttons (Flipper D-pad = keyfob button number):\n"
    "Up = Btn6  (trunk; hold: back to menu)\n"
    "Left = Btn2  (confirm / mode; hold: erase cell)\n"
    "Ok = Btn5  (AM+FM / open car)\n"
    "Right = Btn1  (On / menu / cell+)\n"
    "Down = Btn4  (RX freq / close car)\n"
    "Back = Btn3  (menu / cell-; hold: auto)\n"
    "Hold Up (>1s) = EXIT app\n"
    "Hold Back (>1s) = cycle RF (OFF/TX/RX)\n"
    "Any key held = long press of that keyfob button. The firmware decides what "
    "each button does in each mode.\n"
    "\n"
    "PIN: the screen is black until the unlock PIN (blind button sequence) is "
    "entered; the app auto-unlocks on launch (pandora: Up,Ok,Ok,Up,Up,Down / max: "
    "Up,Ok,Down,Ok,Up,Down).";

/* (Re)builds the main menu with the updated labels. */
static void menu_rebuild(AppState* app) {
    submenu_reset(app->submenu);
    submenu_set_header(app->submenu, "Pandora ARM");
    char buf[64];
    snprintf(buf, sizeof(buf), "Firmware: %s", app->fw_name[0] ? app->fw_name : "<none>");
    submenu_add_item(app->submenu, buf, MenuItemFirmware, menu_submenu_callback, app);
    char seq[48];
    knock_seq_str(app, seq, sizeof(seq));
    snprintf(buf, sizeof(buf), "PIN: %s", seq);
    submenu_add_item(app->submenu, buf, MenuItemKnock, menu_submenu_callback, app);
    submenu_add_item(app->submenu, "Launch", MenuItemLaunch, menu_submenu_callback, app);
    submenu_add_item(app->submenu, "About", MenuItemAbout, menu_submenu_callback, app);
}

static void knock_submenu_callback(void* context, uint32_t index) {
    AppState* app = (AppState*)context;
    int add = -1;
    switch(index) {
    case KnockItemUp: add = B_UP; break;
    case KnockItemDown: add = B_DOWN; break;
    case KnockItemOk: add = B_OK; break;
    case KnockItemBack: add = B_BACK; break;
    case KnockItemB5: add = B5; break;
    case KnockItemB6: add = B6; break;
    case KnockItemDone:
        FURI_LOG_I(TAG, "knock-builder: sequence confirmed (len=%u)", app->knock_user_len);
        app->menu_active = false;
        menu_rebuild(app);
        return;
    case KnockItemClear:
        app->knock_user_len = 0; /* back to the profile default */
        FURI_LOG_I(TAG, "knock-builder: cleared (uses profile default)");
        knock_rebuild(app);
        return;
    default: return;
    }
    if(add >= 0 && app->knock_user_len < sizeof(app->knock_user)) {
        app->knock_user[app->knock_user_len++] = (uint8_t)add;
        FURI_LOG_I(TAG, "knock-builder: +%s (len=%u)", btn_name((uint8_t)add),
                   app->knock_user_len);
        knock_rebuild(app);
    }
}

static void menu_submenu_callback(void* context, uint32_t index) {
    AppState* app = (AppState*)context;
    switch(index) {
    case MenuItemFirmware:
        g_menu_flow = FlowFileBrowser;
        view_dispatcher_stop(app->view_dispatcher);
        break;
    case MenuItemKnock:
        app->menu_active = true; /* "en knock-builder" */
        knock_rebuild(app);
        break;
    case MenuItemLaunch:
        g_menu_flow = FlowLaunch;
        view_dispatcher_stop(app->view_dispatcher);
        break;
    case MenuItemAbout:
        app->menu_active = true; /* reuse flag: "in a sub-view" -> Back returns to menu */
        widget_reset(app->about_widget);
        widget_add_text_scroll_element(app->about_widget, 0, 0, 128, 64, ABOUT_TEXT);
        view_dispatcher_switch_to_view(app->view_dispatcher, PandoraViewAbout);
        break;
    default: break;
    }
}

/* Back: from a sub-view (knock-builder or About) it returns to the main menu;
 * from the main menu it exits. */
static bool menu_back_callback(void* context) {
    AppState* app = (AppState*)context;
    /* Determining the active view is cumbersome with the API; we use a simple
     * flag (menu_active) that is set to true whenever we enter a sub-view
     * (knock-builder re-populates the submenu; About switches to its own Widget
     * view). On Back we rebuild the main menu and return to the submenu view. */
    if(app->menu_active) { /* reuse menu_active as "in a sub-view" */
        app->menu_active = false;
        menu_rebuild(app);
        view_dispatcher_switch_to_view(app->view_dispatcher, PandoraViewSubmenu);
        return true;
    }
    g_menu_flow = FlowExit;
    view_dispatcher_stop(app->view_dispatcher);
    return true;
}

/* Builds and runs the menu (blocks until stop). Creates/destroys the
 * view_dispatcher HERE to leave the GUI free on return. */
static MenuFlow run_menu(AppState* app, Gui* gui) {
    g_menu_flow = FlowExit;
    app->menu_active = false;

    app->view_dispatcher = view_dispatcher_alloc();
    app->submenu = submenu_alloc();
    app->about_widget = widget_alloc();
    if(!app->view_dispatcher || !app->submenu || !app->about_widget) {
        FURI_LOG_E(TAG, "menu allocation failed");
        if(app->submenu) submenu_free(app->submenu);
        if(app->about_widget) widget_free(app->about_widget);
        if(app->view_dispatcher) view_dispatcher_free(app->view_dispatcher);
        app->submenu = NULL;
        app->about_widget = NULL;
        app->view_dispatcher = NULL;
        return FlowExit;
    }

    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_navigation_event_callback(app->view_dispatcher, menu_back_callback);

    view_dispatcher_add_view(
        app->view_dispatcher, PandoraViewSubmenu, submenu_get_view(app->submenu));
    /* a single submenu instance serves both logical views; to be able to map
     * Back differently we reuse the menu_active flag. To make the knock view
     * independent, we use the same re-populated submenu. */
    view_dispatcher_add_view(
        app->view_dispatcher, PandoraViewAbout, widget_get_view(app->about_widget));

    menu_rebuild(app);
    view_dispatcher_attach_to_gui(app->view_dispatcher, gui, ViewDispatcherTypeFullscreen);
    view_dispatcher_switch_to_view(app->view_dispatcher, PandoraViewSubmenu);

    view_dispatcher_run(app->view_dispatcher); /* BLOCKS */

    view_dispatcher_remove_view(app->view_dispatcher, PandoraViewSubmenu);
    view_dispatcher_remove_view(app->view_dispatcher, PandoraViewAbout);
    submenu_free(app->submenu);
    widget_free(app->about_widget);
    view_dispatcher_free(app->view_dispatcher);
    app->submenu = NULL;
    app->about_widget = NULL;
    app->view_dispatcher = NULL;
    return g_menu_flow;
}

static void set_fw_name_from_path(AppState* app) {
    const char* p = furi_string_get_cstr(app->fw_path);
    const char* slash = strrchr(p, '/');
    const char* base = slash ? slash + 1 : p;
    strncpy(app->fw_name, base, sizeof(app->fw_name) - 1);
    app->fw_name[sizeof(app->fw_name) - 1] = 0;
}

/* ------------------------------------------------------------ boot */

static void emu_reset_and_unlock(AppState* app) {
    /* core reset: SP=*(0), PC=*(4) from flash (read through the paged cache) */
    uint32_t sp0 = fc_read(&app->fc, 0, 4);
    uint32_t rst = fc_read(&app->fc, 4, 4);
    memset(app->ram, 0, PA_RAM_SIZE);
    thumb_reset(app->cpu, sp0, rst);
    app->next_pc = rst | 1u;
    app->ninsn = 0;
    app->in_irq = 0;
    app->irq_rr = 0;
    app->pending_tick = 0;
    app->tick_count = 0;
    app->last_tick_insn = 0;
    app->unlocked = 0;
    app->max_step = -1;
    app->nvic_enabled = 0; /* agnostic: fw re-writes ISER during boot */
    app->ppb_vtor = 0;     /* agnostic: fw installs VTOR during boot */
    app->wfi_ram_scanned = 0;
    app->spi_phase = -1;
    app->spi_rx_head = app->spi_rx_tail = 0;

    /* AGNOSTIC knock-code config (FIRMWARE CONFIG, pressed over GPIO by
     * auto_unlock). Prefer the user sequence from the UI; else a model default
     * resolved from the firmware name. The gate is the final held button. */
    if(app->knock_user_len > 0) {
        app->knock_seq = app->knock_user;
        app->knock_len = app->knock_user_len;
        app->knock_gate = app->prof->gate; /* B5 for this keyfob hardware */
        FURI_LOG_I(TAG, "knock: using user sequence (len=%u)", app->knock_user_len);
    } else {
        int gate = B5;
        const uint8_t* seq = default_knock_for_name(app->fw_name, &app->knock_len, &gate);
        app->knock_seq = seq;
        app->knock_gate = gate;
        FURI_LOG_I(TAG, "knock: default for '%s' (len=%u gate=%d)",
                   app->fw_name[0] ? app->fw_name : "?", app->knock_len, gate);
    }
    /* faithful timer model init: TOP=0xFFFF, rate like the Python model
     * (TIMER0 drives delay(): rate 0xC00 -> consistent per-delay cost). */
    for(int i = 0; i < 4; i++) {
        app->timer_t0[i] = 0;
        app->timer_top[i] = 0xFFFF;
        app->timer_rate[i] = (i == 0) ? 0xC00u : 0x400u;
        app->timer_cnt[i] = 0;
    }
    /* OLED double-buffer reset (SPI capture). The visible buffer starts blank
     * (black screen, matching the locked keyfob) and only fills from the first
     * real flush commit. */
    memset(app->oled_fb, 0, OLED_FB_SIZE);
    memset(app->oled_work, 0, OLED_FB_SIZE);
    app->oled_page = 0;
    app->oled_col = 0;
    app->oled_work_dirty = 0;
    app->oled_frames = 0;
    /* LED / buzzer output-mirror state reset. buzz_cand_port=0xFF means "no
     * candidate buzzer pin yet". The LED/speaker are not touched here; they are
     * driven from mmio_write (LED) and the hot loop (speaker). */
    app->led_on = false;
    app->led_logged = false;
    app->buzz_cand_port = 0xFF;
    app->buzz_cand_pin = 0xFF;
    app->buzz_edges = 0;
    app->buzz_last_edge_insn = 0;
    app->buzz_period_insn = 0;
    app->buzz_logged = false;
    app->buzz_timer_logged = false;
    app->spk_owned = false;
    app->spk_playing = false;
    app->spk_freq = BUZZ_TONE_DEFAULT_HZ;
    app->spk_freq_playing = 0.0f;
    app->buzz_active_tick = 0;
    app->buzz_request = false;
    init_buttons_released(app);
    si4432_init(&app->si);
    app->gpio_if = 0;
    app->pending_gpio_even = 0;
    app->pending_gpio_odd = 0;
    /* DIRECT MODE ONLY: dtmod defaults to direct-GPIO (tracked for diagnostics;
     * the FIFO bridge state no longer exists). */
    app->si_dtmod = DtModDirectGpio;
    auto_unlock(app);
}

/* ============================================================ phase 2 emulator */

/* Non-blocking tentative-beep driver, called once per UI frame from the hot loop.
 * The speaker is a SHARED resource: we acquire it lazily (non-blocking: timeout 0;
 * if acquire fails we simply skip sound, no crash), start the tone, and stop +
 * release it once the buzzer activity has been quiet for BUZZ_HOLD_MS. We NEVER
 * block the emulator loop while holding the speaker, and we NEVER leave it held.
 *
 * buzz_request is set by mmio_write when a square-wave GPIO burst is detected;
 * buzz_active_tick is refreshed on every detected edge. This function reads those
 * and owns all furi_hal_speaker calls (mmio_write never touches the HAL). */
static void beep_pump(AppState* app) {
    uint32_t now = furi_get_tick();
    bool want = app->buzz_request &&
                ((uint32_t)(now - app->buzz_active_tick) < BUZZ_HOLD_MS);

    if(want) {
        if(!app->spk_owned) {
            /* lazy, non-blocking acquire; on failure continue silently */
            if(furi_hal_speaker_acquire(SPK_ACQUIRE_TIMEOUT)) {
                app->spk_owned = true;
            } else {
                /* could not take the shared speaker right now; try again next
                 * frame. Clear the one-shot request so we do not busy-retry. */
                app->buzz_request = false;
                return;
            }
        }
        if(app->spk_owned) {
            float f = app->spk_freq;
            if(f < BUZZ_TONE_MIN_HZ || f > BUZZ_TONE_MAX_HZ) f = BUZZ_TONE_DEFAULT_HZ;
            if(!app->spk_playing || f != app->spk_freq_playing) {
                furi_hal_speaker_start(f, BUZZ_VOLUME);
                app->spk_playing = true;
                app->spk_freq_playing = f;
            }
        }
    } else {
        /* activity ceased (or timed out): stop the tone and release the speaker
         * so other apps can use it. */
        if(app->spk_owned) {
            if(app->spk_playing) {
                furi_hal_speaker_stop();
                app->spk_playing = false;
            }
            furi_hal_speaker_release();
            app->spk_owned = false;
        }
        app->buzz_request = false;
    }
}

/* Release the speaker and clear the LED unconditionally (teardown safety). Safe to
 * call even if nothing was acquired/lit. */
static void outputs_teardown(AppState* app) {
    if(app->spk_owned) {
        if(app->spk_playing) {
            furi_hal_speaker_stop();
            app->spk_playing = false;
        }
        furi_hal_speaker_release();
        app->spk_owned = false;
    }
    /* turn the Flipper LED off so it does not stay lit after we exit */
    if(app->led_on) {
        furi_hal_light_set(LightRed, 0x00);
        app->led_on = false;
    }
}

/* Runs the emulator with a direct-draw takeover. Firmware bytes are loaded and
 * the view_dispatcher is already freed, but MCU boot/unlock runs only after the
 * direct-draw screen is acquired so long startup work never leaves the launcher
 * spinner on screen. */
static void run_emulator(AppState* app, Gui* gui) {
    Canvas* canvas = NULL;
    bool fb_cb_added = false;

    if(!app->rx_ring_storage || !app->rx_ring_capacity) {
        FURI_LOG_E(TAG, "rx ring unavailable");
        return;
    }
    app->rx_spool_writer = -1;
    app->rx_spool_player = -1;

    app->gui = gui;
    gui_add_framebuffer_callback(gui, framebuffer_commit_callback, app);
    fb_cb_added = true;
    canvas = gui_direct_draw_acquire(gui);
    if(!canvas) {
        FURI_LOG_E(TAG, "direct_draw_acquire failed");
        goto teardown;
    }
    app->canvas = canvas;

    canvas_reset(canvas);
    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 22, "Pandora ARM");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 36, "Booting MCU...");
    canvas_commit(canvas);

    uint32_t boot_tick = furi_get_tick();
    FURI_LOG_I(TAG, "boot: locating WFI and starting %s", app->prof->name);
    find_wfi(app); /* AGNOSTIC flash scan (from 0x40); sets wfi_set + wfi_pc_hint */
    FURI_LOG_I(TAG, "boot: WFI scan done (%lu ms, count=%lu first=0x%06lX)",
               (unsigned long)(furi_get_tick() - boot_tick),
               (unsigned long)app->wfi_count, (unsigned long)app->wfi_pc_hint);
    emu_reset_and_unlock(app);
    FURI_LOG_I(TAG, "boot OK -> PC=0x%06lX insn=%llu", (unsigned long)app->pc,
               (unsigned long long)app->ninsn);
    FURI_LOG_I(TAG, "emulator: main loop (fw=%s, %s)", app->prof->name,
               app->unlocked ? "UNLOCKED" : "LOCKED");

    uint32_t epoch = furi_get_tick();
    uint64_t next_us = 0;
    uint32_t rendered_oled_frames = UINT32_MAX;
    uint32_t last_screen_commit_tick = furi_get_tick();

    /* --- PHYSICAL KEY MAPPING (6 Flipper keys = 6 keyfob buttons) ---
     * AUTHORITATIVE source is FK_TO_BTN[] (see the enum + table near the top of
     * this file). This comment MUST track that table; the GPIO pin each keyfob
     * button drives is in PROFILE_PANDORA_HW (btn_port_pin):
     *   Flipper UP    -> keyfob B6     (trunk)        pin (0,6).  Hold >1s = EXIT app.
     *   Flipper OK    -> keyfob B5     (AM+FM/open)   pin (0,1).
     *   Flipper DOWN  -> keyfob B_BACK (RX freq/close)pin (0,15).
     *   Flipper LEFT  -> keyfob B_OK   (confirm/mode) pin (2,9).
     *   Flipper RIGHT -> keyfob B_UP   (On/menu/cell+)pin (5,7).
     *   Flipper BACK  -> keyfob B_DOWN (menu/cell-)   pin (5,5).
     *                    Hold >1s = cycle RF bridge (OFF/TX/RX) - does NOT steal
     *                    a keyfob button (the >1s window is above the keyfob's
     *                    long-press, and RF is a host-side utility).
     * Flipper UP (keyfob B6) long hold is repurposed as EXIT (host-side; the
     * keyfob B6 long-press "back to menu" is reachable via a normal short/long
     * nav since EXIT only fires past EXIT_HOLD_MS=1s).
     *
     * INJECTION: each Flipper key press is bridged to the keyfob over GPIO by
     * nav_button() -> press_button_timed() -> set_button() (drive the DIN pin
     * low, run bursts, drive it high, deliver the RELEASE edge, adaptive settle).
     * This is the SAME oracle-validated press cycle used by auto_unlock and the
     * ESP32 emu_press_button; the firmware polls the DIN ports (NOT EXTI) and
     * commits the action on the release flank.
     *
     * Hold detection: poll_raw_keys() gives the instantaneous state. We track
     * per-key press time with furi_get_tick() (ms). On release < HOLD_MS -> short
     * nav_button(); a held key past its threshold fires once (long press / exit /
     * RF) and is marked consumed so the subsequent release does nothing. */
    bool key_down[FK_COUNT] = {false};
    uint32_t key_t0[FK_COUNT] = {0};
    bool key_consumed[FK_COUNT] = {false}; /* long action already fired this hold */

    while(!app->exit_requested) {
        uint8_t keys = poll_raw_keys();
        uint32_t now_tick = furi_get_tick();

        for(int fk = 0; fk < FK_COUNT; fk++) {
            bool down = (keys & FK_TO_KBIT[fk]) != 0;

            if(down && !key_down[fk]) {
                /* rising edge: start timing, defer the action to release/hold */
                key_down[fk] = true;
                key_t0[fk] = now_tick;
                key_consumed[fk] = false;
            } else if(down && key_down[fk] && !key_consumed[fk]) {
                /* still held: check hold thresholds (fire once) */
                uint32_t held = now_tick - key_t0[fk];
                if(fk == FK_UP) {
                    if(held >= EXIT_HOLD_MS) { /* Flipper UP long hold = EXIT app
                                                * (host-side; keyfob button 6 has
                                                * no long-press conflict) */
                        app->exit_requested = true;
                        key_consumed[fk] = true;
                    }
                } else if(fk == FK_BACK) {
                    if(held >= RF_HOLD_MS) { /* Flipper BACK long hold = RF cycle */
                        rf_cycle_mode(app);
                        /* brief status overlay (RF mode / counters) as feedback;
                         * the normal render resumes on the next frame. */
                        overlay_draw(app);
                        rendered_oled_frames = UINT32_MAX; /* restore keyfob image next slice */
                        furi_delay_ms(600);
                        key_consumed[fk] = true;
                    }
                }
                /* other keys: the long-vs-short decision is made on release */
            } else if(!down && key_down[fk]) {
                /* release: decide short vs long for the keyfob button */
                uint32_t held = now_tick - key_t0[fk];
                key_down[fk] = false;
                if(key_consumed[fk]) {
                    /* exit / RF already handled on hold; no keyfob button */
                } else if(app->unlocked) {
                    int btn = FK_TO_BTN[fk];
                    if(fk == FK_UP) {
                        /* Flipper UP -> keyfob 6; short click only (its >1s hold
                         * is the host-side EXIT, consumed above). */
                        nav_button(app, btn, false);
                    } else if(fk == FK_BACK) {
                        /* Flipper BACK -> keyfob 3; short vs long press. (>1s was
                         * RF cycle, handled above as consumed.) */
                        nav_button(app, btn, held >= HOLD_MS);
                    } else {
                        nav_button(app, btn, held >= HOLD_MS);
                    }
                } else {
                    /* pre-unlock fallback: use the knock tap() (scanner). Normally
                     * auto_unlock already ran at boot, so this path is rare. */
                    tap(app, FK_TO_BTN[fk]);
                }
            }
        }

        if(app->exit_requested) break;

        /* Poll RSSI outside the GPIO capture IRQ; the edge callback only records
         * the current gate bit and stays constant-time. */
        rx_rssi_update(app);

        /* Drain the capture ring before running another firmware slice. During
         * RX the host radio can produce edges faster than the emulator's
         * foreground instruction loop; postponing this drain until afterward
         * needlessly consumes ring headroom and delays completed-frame handoff. */
        rx_bridge_pump(app, false);

        /* run a slice of the real firmware (captures TX via mmio_write).
         * EMU_INSN_PER_FRAME caps how much firmware runs per UI frame. The
         * firmware has an inactivity timeout (~280k insn) that returns a
         * sub-mode to the main carousel -- a REAL feature. Running too fast makes
         * it fire instantly; this value keeps it at a human scale. Tune on real
         * hardware if the pacing feels off (lower = slower fw / longer timeout). */
        run_burst(app, app->rx_running ? EMU_INSN_PER_RX_FRAME : EMU_INSN_PER_FRAME);
        /* Drain once more after the firmware slice to collect edges captured
         * while the interpreter was running. Replay is kept here so a frame
         * decoded in the slice is consumed by firmware before more edges feed. */
        rx_bridge_pump(app, true);

        /* DIAG: heartbeat every ~2s so we can confirm the loop keeps running and
         * measure the REAL emulation rate on hardware (insn/s = ninsn delta / dt).
         * If this stops printing, the loop is stuck (e.g. inside run_burst). */
        {
            static uint32_t s_hb_tick = 0;
            static uint64_t s_hb_ninsn = 0;
            static uint32_t s_hb_flash_misses = 0;
            uint32_t t = furi_get_tick();
            if(s_hb_tick == 0) {
                s_hb_tick = t;
                s_hb_ninsn = app->ninsn;
                s_hb_flash_misses = app->fc.miss_count;
            }
            if((uint32_t)(t - s_hb_tick) >= 2000) {
                uint64_t di = app->ninsn - s_hb_ninsn;
                uint32_t flash_misses = app->fc.miss_count;
                uint32_t dt = t - s_hb_tick;
                /* Keep emulator performance chatter out of RX-focused logs. */
                if(!app->rx_running) {
                    FURI_LOG_D(
                        TAG,
                        "hb: ninsn=%llu rate=%lu k-insn/s unlocked=%d flash_misses=+%lu",
                        (unsigned long long)app->ninsn,
                        (unsigned long)(dt ? (di / dt) : 0), /* insn/ms == k-insn/s */
                        app->unlocked,
                        (unsigned long)(flash_misses - s_hb_flash_misses));
                }
                s_hb_tick = t;
                s_hb_ninsn = app->ninsn;
                s_hb_flash_misses = flash_misses;
            }
        }

        /* RX-only diagnostic summary: raw activity, loss, RSSI gate, spool and
         * firmware decode counts. Keep it slow enough for readable CLI captures. */
        if(app->rx_running) {
            static uint32_t s_rxhb_tick = 0;
            uint32_t t = furi_get_tick();
            if(s_rxhb_tick == 0) s_rxhb_tick = t;
            if((uint32_t)(t - s_rxhb_tick) >= RX_HEARTBEAT_MS) {
                uint32_t events = __atomic_load_n(&app->rx_events, __ATOMIC_RELAXED);
                uint32_t dropped = __atomic_load_n(&app->rx_dropped, __ATOMIC_RELAXED);
                uint32_t queued =
                    __atomic_load_n(&app->rx_head, __ATOMIC_ACQUIRE) -
                    __atomic_load_n(&app->rx_tail, __ATOMIC_ACQUIRE);
                uint32_t de = events - app->rx_events_hb;
                uint32_t df = app->rx_frames - app->rx_frames_hb;
                uint32_t dd = dropped - app->rx_dropped_hb;
                uint32_t gate_opens = app->rx_gate_open_count;
                uint32_t gate_closes = app->rx_gate_close_count;
                uint32_t play_done = 0;
                uint32_t play_total = 0;
                if(app->rx_spool_player >= 0) {
                    RxSpool* playing = &app->rx_spool[(uint8_t)app->rx_spool_player];
                    play_total = playing->bytes / sizeof(RxEdge);
                    play_done = playing->play_pos / sizeof(RxEdge);
                    uint32_t unread = app->rx_spool_play_count - app->rx_spool_play_index;
                    play_done = play_done >= unread ? play_done - unread : 0;
                }
                FURI_LOG_I(
                    TAG,
                    "RX: @%luHz air=+%lu in=%lu q=%lu drop=+%lu gate=%u RSSI=%.1f/%.1f/%.1f "
                    "gate_evt=+%lu/-%lu spool=%lu play=%lu/%lu replay=%lu lost=%lu frame=+%lu/%lu",
                    (unsigned long)app->frequency, (unsigned long)de,
                    (unsigned long)app->rx_injected, (unsigned long)queued,
                    (unsigned long)dd,
                    (unsigned)__atomic_load_n(&app->rx_gate_open_isr, __ATOMIC_RELAXED),
                    (double)app->rx_rssi_dbm, (double)app->rx_noise_floor_dbm,
                    (double)app->rx_gate_open_dbm,
                    (unsigned long)(gate_opens - app->rx_gate_open_hb),
                    (unsigned long)(gate_closes - app->rx_gate_close_hb),
                    (unsigned long)app->rx_spool_bursts,
                    (unsigned long)play_done, (unsigned long)play_total,
                    (unsigned long)app->rx_spool_replayed,
                    (unsigned long)app->rx_spool_lost,
                    (unsigned long)df, (unsigned long)app->rx_frames);
                app->rx_events_hb = events;
                app->rx_dropped_hb = dropped;
                app->rx_frames_hb = app->rx_frames;
                app->rx_gate_open_hb = gate_opens;
                app->rx_gate_close_hb = gate_closes;
                s_rxhb_tick = t;
            }
        }

        /* RF bridge: replay accumulated TX / inject received RX.
         * DIRECT MODE ONLY: TX replays the PB3 bit-bang edges; RX injects the
         * demodulated OOK stream into PB0 for the firmware's demod ISR. There is
         * no FIFO/packet path (removed; DYN_HW_D605.md: 0 FIFO accesses). */
        tx_bridge_flush(app);

        /* tentative buzzer beep: non-blocking, owns the shared speaker only while
         * active and releases it as soon as the firmware's buzzer burst ceases. */
        beep_pump(app);

        /* New firmware frames are shown immediately. Keep a periodic repaint as
         * well: some firmware leaves its framebuffer unchanged for long periods,
         * and the Flipper's direct-draw canvas can otherwise appear black until
         * the next keypress causes a commit. */
        uint32_t screen_tick = furi_get_tick();
        bool oled_changed = app->oled_frames != rendered_oled_frames;
        bool refresh_due =
            (uint32_t)(screen_tick - last_screen_commit_tick) >= SCREEN_REFRESH_FALLBACK_MS;
        if(oled_changed || refresh_due) {
            render_oled(app);
            if(app->rx_frames && oled_changed) {
                uint16_t lit_pixels = 0;
                int min_x = SCREEN_W, min_y = SCREEN_H;
                int max_x = -1, max_y = -1;
                for(int y = 0; y < SCREEN_H; y++) {
                    for(int x = 0; x < SCREEN_W; x++) {
                        size_t idx = (size_t)(y >> 3) * SCREEN_W + (size_t)x;
                        if(app->screen[idx] & (uint8_t)(1u << (y & 7))) {
                            lit_pixels++;
                            if(x < min_x) min_x = x;
                            if(x > max_x) max_x = x;
                            if(y < min_y) min_y = y;
                            if(y > max_y) max_y = y;
                        }
                    }
                }
                FURI_LOG_I(TAG,
                           "RX UI render oled=%lu src=92x64/2bpp crop=88 offset=20 "
                           "lit=%u bbox=%d,%d-%d,%d gui_commit=%u",
                           (unsigned long)app->oled_frames, (unsigned)lit_pixels,
                           min_x, min_y, max_x, max_y, (unsigned)!app->menu_active);
            }
            if(!app->menu_active) canvas_commit(canvas);
            rendered_oled_frames = app->oled_frames;
            last_screen_commit_tick = screen_tick;
        }

        /* pacing + yield */
        next_us += FRAME_US;
        uint32_t target = epoch + (uint32_t)((next_us + 500) / 1000);
        uint32_t now = furi_get_tick();
        int32_t ahead = (int32_t)(target - now);
        if(ahead > 4 * 33) {
            epoch = now;
            next_us = 0;
            furi_delay_ms(1);
        } else if(ahead > 0) {
            furi_delay_until_tick(target);
        } else {
            furi_delay_ms(1);
        }
    }

teardown:
    /* DIAG: log each teardown step so a crash (furi_check failed) that happens on
     * exit / USB disconnect can be localised to the exact resource being freed. */
    FURI_LOG_D(TAG, "teardown: begin (exit_req=%d)", app->exit_requested);
    outputs_teardown(app); /* stop/release the speaker and clear the LED */
    FURI_LOG_D(TAG, "teardown: outputs (speaker/LED) released");
    radio_deinit(app);
    FURI_LOG_D(TAG, "teardown: radio_deinit done");
    for(uint8_t i = 0; i < RX_SPOOL_SLOTS; i++) {
        RxSpool* s = &app->rx_spool[i];
        if(s->file) {
            if(s->state == RxSpoolWriting || s->state == RxSpoolPlaying)
                storage_file_close(s->file);
            storage_file_free(s->file);
            s->file = NULL;
        }
    }
    if(fb_cb_added) gui_remove_framebuffer_callback(gui, framebuffer_commit_callback, app);
    FURI_LOG_D(TAG, "teardown: fb callback removed");
    wait_inflight_zero(&s_fb_cb_inflight);
    FURI_LOG_D(TAG, "teardown: inflight drained");
    if(canvas) gui_direct_draw_release(gui);
    FURI_LOG_D(TAG, "teardown: direct_draw released");
    if(app->rx_ring_storage) {
        free(app->rx_ring_storage);
        app->rx_ring_storage = NULL;
        app->rx_ring_duration = NULL;
        app->rx_ring_flags = NULL;
    }
    FURI_LOG_D(TAG, "emulator: exit");
}

/* ------------------------------------------------------------ main */

int32_t pandora_arm_app(void* p) {
    UNUSED(p);

    AppState* app = (AppState*)malloc(sizeof(AppState));
    if(!app) return -1;
    memset(app, 0, sizeof(AppState));

    Storage* storage = (Storage*)furi_record_open(RECORD_STORAGE);
    DialogsApp* dialogs = (DialogsApp*)furi_record_open(RECORD_DIALOGS);
    Gui* gui = (Gui*)furi_record_open(RECORD_GUI);
    app->storage = storage; /* kept so the flash cache can stream pages at runtime */
    app->fw_path = furi_string_alloc_set_str("/ext");
    if(!storage || !dialogs || !gui || !app->fw_path) {
        FURI_LOG_E(TAG, "startup allocation/record open failed");
        if(app->fw_path) furi_string_free(app->fw_path);
        if(gui) furi_record_close(RECORD_GUI);
        if(dialogs) furi_record_close(RECORD_DIALOGS);
        if(storage) furi_record_close(RECORD_STORAGE);
        free(app);
        return -1;
    }

    bool launch = false;

    /* --- PHASE 1: menu (view_dispatcher). Loop until Launch or Exit. --- */
    while(true) {
        /* keep the profile up to date to show the knock default */
        if(app->fw_name[0]) app->prof = pick_profile(furi_string_get_cstr(app->fw_path));

        MenuFlow flow = run_menu(app, gui);
        if(flow == FlowExit) {
            launch = false;
            break;
        } else if(flow == FlowFileBrowser) {
            DialogsFileBrowserOptions opts;
            dialog_file_browser_set_basic_options(&opts, "*", NULL);
            opts.base_path = "/ext";
            opts.hide_ext = false;
            if(dialog_file_browser_show(dialogs, app->fw_path, app->fw_path, &opts)) {
                set_fw_name_from_path(app);
                app->prof = pick_profile(furi_string_get_cstr(app->fw_path));
                FURI_LOG_I(TAG, "firmware chosen: %s (profile %s)", app->fw_name,
                           app->prof->name);
            }
            continue;
        } else if(flow == FlowLaunch) {
            if(app->fw_name[0] == 0) {
                FURI_LOG_W(TAG, "Launch without firmware; opening file browser");
                DialogsFileBrowserOptions opts;
                dialog_file_browser_set_basic_options(&opts, "*", NULL);
                opts.base_path = "/ext";
                opts.hide_ext = false;
                if(!dialog_file_browser_show(dialogs, app->fw_path, app->fw_path, &opts)) {
                    continue;
                }
                set_fw_name_from_path(app);
                app->prof = pick_profile(furi_string_get_cstr(app->fw_path));
            }
            launch = true;
            break;
        }
    }

    /* --- PHASE 2: emulator (direct-draw). Only if the user requested Launch. --- */
    if(launch) {
        do {
            app->prof = pick_profile(furi_string_get_cstr(app->fw_path));

            app->cpu = (ThumbCore*)malloc(sizeof(ThumbCore));
            if(!app->cpu) {
                FURI_LOG_E(TAG, "cpu alloc failed");
                break;
            }
            thumb_init(app->cpu);

            FURI_LOG_I(
                TAG, "free heap=%uK, max block=%uK, need flash=%luK + ram=%uK",
                (unsigned)(memmgr_get_free_heap() / 1024u),
                (unsigned)(memmgr_heap_get_max_free_block() / 1024u),
                (unsigned long)(app->flash_used / 1024u),
                (unsigned)(PA_RAM_SIZE / 1024u));

            /* Allocate the 32KB RAM region FIRST (single contiguous block, fits
             * well under the 67KB limit). It MUST come before the flash cache so
             * the cache sizes its slots against the heap that truly remains (and
             * so RAM can never be starved by the cache grabbing it first). */
            app->ram = (uint8_t*)safe_malloc(PA_RAM_SIZE);
            if(!app->ram) {
                FURI_LOG_E(TAG, "ram alloc failed");
                DialogMessage* msg = dialog_message_alloc();
                if(msg) {
                    dialog_message_set_text(
                        msg, "Out of memory\n(RAM 32K)", 64, 30, AlignCenter, AlignCenter);
                    dialog_message_set_buttons(msg, NULL, "OK", NULL);
                    dialog_message_show(dialogs, msg);
                    dialog_message_free(msg);
                }
                break;
            }
            memset(app->ram, 0, PA_RAM_SIZE);

            /* Reserve RX transfer memory before the flash cache takes the
             * remaining heap. Keep this staging ring compact: the SD spool is
             * the long-burst store, while the ring only bridges capture to the
             * emulator. The 2-bit flags keep 2048 edges at 4608 bytes, leaving
             * enough heap for the minimum three 4KB flash-cache pages. */
            static const uint16_t rx_ring_sizes[] = {
                RX_RING_MAX_LEN, 1536u, 1024u, 512u};
            for(size_t i = 0; i < COUNT_OF(rx_ring_sizes); i++) {
                uint16_t capacity = rx_ring_sizes[i];
                size_t duration_bytes = (size_t)capacity * sizeof(uint16_t);
                size_t flags_bytes = ((size_t)capacity + 3u) / 4u;
                app->rx_ring_storage = (uint8_t*)safe_malloc(duration_bytes + flags_bytes);
                if(app->rx_ring_storage) {
                    app->rx_ring_capacity = capacity;
                    app->rx_ring_duration = (uint16_t*)app->rx_ring_storage;
                    app->rx_ring_flags = app->rx_ring_storage + duration_bytes;
                    memset(app->rx_ring_storage, 0, duration_bytes + flags_bytes);
                    FURI_LOG_I(TAG, "RX ring reserved: %u edges (%u bytes)", capacity,
                               (unsigned)(duration_bytes + flags_bytes));
                    break;
                }
            }
            if(!app->rx_ring_storage) {
                FURI_LOG_E(TAG, "RX ring allocation failed (free=%uK max=%uK)",
                           (unsigned)(memmgr_get_free_heap() / 1024u),
                           (unsigned)(memmgr_heap_get_max_free_block() / 1024u));
                break;
            }

            /* STEP 1: normalise the firmware into a flat temp .bin on the SD
             * (streaming; no big RAM buffer). STEP 2: init the paged flash cache
             * (small 4KB slots -> fits a fragmented heap; NO single big malloc). */
            LoadResult lr = flatten_firmware(app, storage, furi_string_get_cstr(app->fw_path));
            if(lr == LoadOk) lr = fc_init(app);
            if(lr != LoadOk) {
                char tbuf[64];
                if(lr == LoadNoMem)
                    snprintf(
                        tbuf, sizeof(tbuf), "Out of memory\nfree:%uK max:%uK",
                        (unsigned)(memmgr_get_free_heap() / 1024u),
                        (unsigned)(memmgr_heap_get_max_free_block() / 1024u));
                else if(lr == LoadBadFormat)
                    snprintf(tbuf, sizeof(tbuf), "Invalid format\n(.hex or .bin)");
                else
                    snprintf(tbuf, sizeof(tbuf), "Load error");
                FURI_LOG_E(TAG, "load failed: %s", tbuf);
                DialogMessage* msg = dialog_message_alloc();
                if(msg) {
                    dialog_message_set_text(msg, tbuf, 64, 30, AlignCenter, AlignCenter);
                    dialog_message_set_buttons(msg, NULL, "OK", NULL);
                    dialog_message_show(dialogs, msg);
                    dialog_message_free(msg);
                }
                break;
            }

            /* connect memory: ONLY RAM is a direct region now. FLASH is paged, so
             * flash_ptr=NULL -> every flash read falls to the read callback
             * (mmio_read -> fc_read -> cache). CRITICAL for speed: the firmware is
             * RAM-resident, so instruction fetch hits the DIRECT RAM region and
             * does NOT pay the cache cost; only the few flash constant loads do. */
            thumb_set_regions(
                app->cpu, NULL, PA_FLASH_BASE, 0, app->ram, PA_RAM_BASE, PA_RAM_SIZE);
            thumb_set_mem_cb(app->cpu, mmio_read, mmio_write, app);
            thumb_set_hook(app->cpu, code_hook, app);

            app->fb_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
            if(!app->fb_mutex) {
                FURI_LOG_E(TAG, "fb_mutex alloc failed");
                break;
            }

            radio_init(app);

            FURI_LOG_I(TAG, "profile: %s (knock %s)", app->prof->name,
                       app->knock_user_len ? "user" : "default");

            run_emulator(app, gui);
        } while(false);
    }

    /* --- teardown --- */
    if(app->cpu) free(app->cpu);
    if(app->wfi_set) { free(app->wfi_set); app->wfi_set = NULL; app->wfi_count = app->wfi_cap = 0; }
    fc_deinit(&app->fc); /* free the 4KB slots + bookkeeping + close the .bin */
    /* The selected HEX/BIN remains the source of truth. This canonical flat
     * image is only a paging backing file for this app session, so remove it
     * after closing the cache to reclaim SD space on normal exit or load error. */
    if(app->storage) (void)storage_common_remove(app->storage, PA_FLASH_BIN);
    if(app->ram) free(app->ram);
    if(app->rx_ring_storage) {
        free(app->rx_ring_storage);
        app->rx_ring_storage = NULL;
        app->rx_ring_duration = NULL;
        app->rx_ring_flags = NULL;
        app->rx_ring_capacity = 0;
    }
    if(app->fb_mutex) furi_mutex_free(app->fb_mutex);
    furi_string_free(app->fw_path);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_DIALOGS);
    furi_record_close(RECORD_STORAGE);
    free(app);
    return 0;
}
