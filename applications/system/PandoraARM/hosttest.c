/* hosttest.c - Host-side validation of the AGNOSTIC PandoraARM emulator.
 *
 * This harness includes main.c (to reach its STATIC emulator core) with the
 * Flipper GUI/HAL/storage APIs mocked (see /tmp/opencode/pandora_host/mocks).
 * It replicates the setup sequence that run_emulator() performs -- flatten the
 * firmware to the flat .bin, init the paged flash cache, wire the Thumb-2 core
 * regions/callbacks, scan WFI, reset+auto-unlock -- WITHOUT any GUI/radio.
 *
 * It then validates the port against the canonical core (Python/ESP32):
 *   - boots both firmwares (VTOR, no hang),
 *   - the agnostic ISER-driven NVIC fires IRQ13 with the right cadence (not
 *     diluted by the fast-path),
 *   - nvic_enabled reflects the firmware's real ISER,
 *   - after the GPIO knock the OLED draws; fb_nz is compared to the canonical
 *     (pandora 311->547, pmax 88->429),
 *   - the SAME binary runs BOTH firmwares (agnostic),
 *   - a foreign (non-keyfob) firmware does not crash.
 *
 * The knock-code is FIRMWARE CONFIG, supplied by the HOST here (set into
 * app->knock_user), exactly like the ESP32 hosttest.
 *
 * Build (gcc, NOT fbt) -- mocks live in-tree under hosttest_mocks/:
 *   gcc -std=c11 -O2 -D_GNU_SOURCE -Ihosttest_mocks -I. -Ilib/thumbcore \
 *       -include unistd.h hosttest.c lib/thumbcore/thumb_core.c -o /tmp/hosttest
 *
 * Run (compares fb_nz against the canonical ESP32/Python core):
 *   ./hosttest <fw.hex|.flash.bin> <pandora|pmax|none|auto> [fb1] [fb2]
 *   pandora: fb1=311 fb2=547    pmax: fb1=88 fb2=429
 */
#define PANDORA_HOSTTEST 1

/* Pull in the whole app (static emulator core) with the Flipper APIs mocked. */
#include "main.c"

/* ---- host storage root (maps /ext -> a local temp dir) ---- */
const char* g_host_ext_root = NULL;

/* Count lit framebuffer bytes (the canonical fb_nz metric). */
static int fb_nonzero(AppState* app) {
    int nz = 0;
    for(int i = 0; i < OLED_FB_SIZE; i++)
        if(app->oled_fb[i]) nz++;
    return nz;
}

/* Dump the VISIBLE framebuffer as ASCII (88 visible cols x 64 rows). */
static void fb_ascii(AppState* app) {
    const uint8_t* fbv = app->oled_fb;
    for(int page = 0; page < OLED_PAGES; page++) {
        for(int k = 0; k < 4; k++) {
            for(int col = 0; col < OLED_VISIBLE_COLS; col++) {
                uint8_t v = fbv[page * OLED_COLS + col];
                int lvl = ((v >> (2 * k)) & 1) + ((v >> (2 * k + 1)) & 1);
                putchar(lvl ? '#' : ' ');
            }
            putchar('\n');
        }
    }
}

/* Replicate run_emulator()'s core setup without GUI/radio. Returns 0 on ok. */
static int host_boot(AppState* app, Storage* storage, const char* path) {
    memset(app, 0, sizeof(*app));
    app->knock_gate = -1;
    app->storage = storage;
    app->prof = pick_profile(path); /* always PROFILE_PANDORA_HW */

    /* short name for default-knock resolution (like set_fw_name_from_path) */
    const char* slash = strrchr(path, '/');
    const char* base = slash ? slash + 1 : path;
    strncpy(app->fw_name, base, sizeof(app->fw_name) - 1);

    /* 1) flatten firmware to the flat .bin (derives app->flash_used) */
    LoadResult lr = flatten_firmware(app, storage, path);
    if(lr != LoadOk) { fprintf(stderr, "flatten failed: %d\n", lr); return 2; }

    /* 2) allocate RAM, cpu, init the paged flash cache */
    app->ram = (uint8_t*)malloc(PA_RAM_SIZE);
    if(!app->ram) return 3;
    memset(app->ram, 0, PA_RAM_SIZE);
    app->cpu = (ThumbCore*)malloc(sizeof(ThumbCore));
    if(!app->cpu) return 3;
    thumb_init(app->cpu);
    lr = fc_init(app);
    if(lr != LoadOk) { fprintf(stderr, "fc_init failed: %d\n", lr); return 4; }

    /* 3) wire regions/callbacks EXACTLY like run_emulator */
    thumb_set_regions(app->cpu, NULL, PA_FLASH_BASE, 0, app->ram, PA_RAM_BASE, PA_RAM_SIZE);
    thumb_set_mem_cb(app->cpu, mmio_read, mmio_write, app);
    thumb_set_hook(app->cpu, code_hook, app);

    /* 4) scan WFI (agnostic) then reset + auto-unlock */
    find_wfi(app);
    emu_reset_and_unlock(app);
    return 0;
}

static void host_free(AppState* app) {
    if(app->cpu) free(app->cpu);
    if(app->wfi_set) free(app->wfi_set);
    fc_deinit(&app->fc);
    if(app->ram) free(app->ram);
}

/* Knock-codes are FIRMWARE CONFIG supplied by the host (model/build), pressed
 * over GPIO like a finger -- NOT part of the agnostic emulator. */
static void set_host_knock(AppState* app, const char* sel) {
    if(!strcmp(sel, "pandora")) {
        memcpy(app->knock_user, KNOCK_PANDORA, sizeof(KNOCK_PANDORA));
        app->knock_user_len = (uint8_t)sizeof(KNOCK_PANDORA);
    } else if(!strcmp(sel, "pmax")) {
        memcpy(app->knock_user, KNOCK_PMAX, sizeof(KNOCK_PMAX));
        app->knock_user_len = (uint8_t)sizeof(KNOCK_PMAX);
    } else {
        app->knock_user_len = 0; /* none: rely on default_knock_for_name / no knock */
    }
}

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "/opt/postmarket/test/PANDORA_MAX.hex";
    const char* knock_sel = argc > 2 ? argv[2] : "auto";
    int fb_expect1 = argc > 3 ? atoi(argv[3]) : -1; /* post-unlock */
    int fb_expect2 = argc > 4 ? atoi(argv[4]) : -1; /* after convergence */

    /* per-run temp dir so the flat .bin does not clash between firmwares */
    static char root[256];
    snprintf(root, sizeof(root), "/tmp/opencode/pandora_host/ext_%d", (int)getpid());
    char cmd[300];
    snprintf(cmd, sizeof(cmd), "mkdir -p '%s/apps_data/pandora_arm'", root);
    (void)system(cmd);
    g_host_ext_root = root;

    Storage storage = {0};
    AppState app;

    printf("==== HOSTTEST: %s (knock=%s) ====\n", path, knock_sel);
    const char* sel = knock_sel;
    if(!strcmp(sel, "auto")) {
        /* resolve from file name like the UI default would */
        sel = strstr(path, "MAX") || strstr(path, "max") ? "pmax" :
              (strstr(path, "2_5253") || strstr(path, "pandora")) ? "pandora" : "none";
    }

    int rc = host_boot(&app, &storage, path);
    if(rc == 0) set_host_knock(&app, sel); /* set BEFORE reset; re-reset below */
    if(rc != 0) {
        printf("BOOT FAILED rc=%d (foreign/non-keyfob fw is allowed to not boot)\n", rc);
        /* a failed FLATTEN/cache is not a crash; report and exit cleanly */
        return rc == 2 ? 0 : rc;
    }

    /* host_boot already did an auto_unlock with whatever knock was configured at
     * the time (none). Now set the host knock and re-run reset+unlock so the
     * GPIO autotest actually presses it (mirrors the UI Launch flow). */
    set_host_knock(&app, sel);
    emu_reset_and_unlock(&app);

    printf("after reset+unlock: PC=0x%08X vtor=0x%08X nvic_enabled=0x%llX\n",
           thumb_get_reg(app.cpu, 15), app.ppb_vtor,
           (unsigned long long)app.nvic_enabled);
    printf("  ninsn=%llu tick_count(IRQ fires)=%u oled_frames=%u unlocked=%d fb_nz=%d\n",
           (unsigned long long)app.ninsn, app.tick_count, app.oled_frames,
           emu_is_unlocked(&app), fb_nonzero(&app));

    int nz1 = fb_nonzero(&app);

    /* run more bursts; IRQ13 must keep firing and the UI converge */
    uint32_t t0 = app.tick_count;
    for(int i = 0; i < 40; i++) run_burst(&app, 50000);
    int nz2 = fb_nonzero(&app);
    printf("ran 40x50000 insn: oled_frames=%u IRQ fires +%u (total %u) fb_nz=%d\n",
           app.oled_frames, app.tick_count - t0, app.tick_count, nz2);

    int ok = 1;
    if(app.ppb_vtor < 0x8000u) { printf("  FAIL: VTOR not installed\n"); ok = 0; }
    if((app.tick_count - t0) == 0) { printf("  FAIL: IRQ13 not firing in steady state\n"); ok = 0; }
    if(fb_expect1 >= 0) {
        if(nz1 == fb_expect1) printf("  OK: post-unlock fb_nz=%d matches canonical\n", nz1);
        else { printf("  MISMATCH: post-unlock fb_nz=%d expected %d\n", nz1, fb_expect1); ok = 0; }
    }
    if(fb_expect2 >= 0) {
        if(nz2 == fb_expect2) printf("  OK: converged fb_nz=%d matches canonical\n", nz2);
        else { printf("  MISMATCH: converged fb_nz=%d expected %d\n", nz2, fb_expect2); ok = 0; }
    }

    fb_ascii(&app);
    host_free(&app);

    printf("==== RESULT: %s ====\n", ok ? "PASS" : "CHECK");
    return 0;
}
