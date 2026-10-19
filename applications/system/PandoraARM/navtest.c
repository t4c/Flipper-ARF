/* navtest.c - drive the REAL nav path of PandoraARM on the host.
 *
 * Includes main.c with Flipper APIs mocked (same as hosttest.c), boots+unlocks
 * the firmware exactly like run_emulator, THEN exercises navigation two ways:
 *   (A) direct nav_button(btn) -- the injection primitive (oracle-validated).
 *   (B) the EXACT inline bridge state-machine from pandora_arm_app's event loop,
 *       fed by a scripted poll_raw_keys() sequence (press then release), so we
 *       test input->map->inject->run->render by the SAME code the Flipper runs.
 *
 * Build:
 *   gcc -std=c11 -O2 -D_GNU_SOURCE -Ihosttest_mocks -I. -Ilib/thumbcore \
 *       -include unistd.h navtest.c lib/thumbcore/thumb_core.c -o /tmp/opencode/navtest
 */
#define PANDORA_HOSTTEST 1
#include "main.c"

const char* g_host_ext_root = NULL;

#include <stdio.h>

static int fb_nonzero(AppState* app) {
    int nz = 0;
    for(int i = 0; i < OLED_FB_SIZE; i++) if(app->oled_fb[i]) nz++;
    return nz;
}
static uint32_t fbhash(AppState* app) { return fb_hash(app); }

static int host_boot(AppState* app, Storage* storage, const char* path, const char* sel) {
    memset(app, 0, sizeof(*app));
    app->knock_gate = -1;
    app->storage = storage;
    app->prof = pick_profile(path);
    const char* slash = strrchr(path, '/');
    const char* base = slash ? slash + 1 : path;
    strncpy(app->fw_name, base, sizeof(app->fw_name) - 1);
    LoadResult lr = flatten_firmware(app, storage, path);
    if(lr != LoadOk) { fprintf(stderr, "flatten failed: %d\n", lr); return 2; }
    app->ram = (uint8_t*)malloc(PA_RAM_SIZE);
    if(!app->ram) return 3;
    memset(app->ram, 0, PA_RAM_SIZE);
    app->cpu = (ThumbCore*)malloc(sizeof(ThumbCore));
    if(!app->cpu) return 3;
    thumb_init(app->cpu);
    lr = fc_init(app);
    if(lr != LoadOk) return 4;
    thumb_set_regions(app->cpu, NULL, PA_FLASH_BASE, 0, app->ram, PA_RAM_BASE, PA_RAM_SIZE);
    thumb_set_mem_cb(app->cpu, mmio_read, mmio_write, app);
    thumb_set_hook(app->cpu, code_hook, app);
    if(!strcmp(sel, "pandora")) {
        memcpy(app->knock_user, KNOCK_PANDORA, sizeof(KNOCK_PANDORA));
        app->knock_user_len = (uint8_t)sizeof(KNOCK_PANDORA);
    } else if(!strcmp(sel, "pmax")) {
        memcpy(app->knock_user, KNOCK_PMAX, sizeof(KNOCK_PMAX));
        app->knock_user_len = (uint8_t)sizeof(KNOCK_PMAX);
    }
    find_wfi(app);
    emu_reset_and_unlock(app);
    return 0;
}

/* ---- (B) replicate the EXACT inline bridge state machine of the event loop ---
 * This mirrors main.c lines ~3910-3973. We call it per "UI frame" with the
 * scripted key state (g_scripted_keys) standing in for poll_raw_keys(). */
typedef struct {
    bool key_down[FK_COUNT];
    uint32_t key_t0[FK_COUNT];
    bool key_consumed[FK_COUNT];
} BridgeState;

static uint32_t g_fake_tick = 0;

static void bridge_frame(AppState* app, BridgeState* bs, uint8_t keys) {
    uint32_t now_tick = g_fake_tick;
    for(int fk = 0; fk < FK_COUNT; fk++) {
        bool down = (keys & FK_TO_KBIT[fk]) != 0;
        if(down && !bs->key_down[fk]) {
            bs->key_down[fk] = true;
            bs->key_t0[fk] = now_tick;
            bs->key_consumed[fk] = false;
        } else if(down && bs->key_down[fk] && !bs->key_consumed[fk]) {
            uint32_t held = now_tick - bs->key_t0[fk];
            if(fk == FK_UP) {
                if(held >= EXIT_HOLD_MS) { app->exit_requested = true; bs->key_consumed[fk] = true; }
            } else if(fk == FK_BACK) {
                if(held >= RF_HOLD_MS) { /* RF cycle -- skip on host */ bs->key_consumed[fk] = true; }
            }
        } else if(!down && bs->key_down[fk]) {
            uint32_t held = now_tick - bs->key_t0[fk];
            bs->key_down[fk] = false;
            if(bs->key_consumed[fk]) {
            } else if(app->unlocked) {
                int btn = FK_TO_BTN[fk];
                if(fk == FK_UP) nav_button(app, btn, false);
                else if(fk == FK_BACK) nav_button(app, btn, held >= HOLD_MS);
                else nav_button(app, btn, held >= HOLD_MS);
            } else {
                tap(app, FK_TO_BTN[fk]);
            }
        }
    }
}

/* One full "button tap" through the bridge: rising edge frame, then release
 * frame (short press, held < HOLD_MS). Advances fake tick a little. */
static void bridge_tap(AppState* app, BridgeState* bs, uint8_t kbit) {
    g_fake_tick += 1;
    bridge_frame(app, bs, kbit);        /* press */
    g_fake_tick += 50;                  /* short press (< HOLD_MS) */
    bridge_frame(app, bs, 0);           /* release -> dispatch nav_button */
    /* mimic the event loop's steady-state firmware slice after dispatch */
    run_burst(app, EMU_INSN_PER_FRAME);
}

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "/opt/postmarket/test/2_5253623551453304897.hex";
    const char* sel = argc > 2 ? argv[2] : "pandora";

    static char root[256];
    snprintf(root, sizeof(root), "/tmp/opencode/pandora_host/nav_%d", (int)getpid());
    char cmd[300];
    snprintf(cmd, sizeof(cmd), "mkdir -p '%s/apps_data/pandora_arm'", root);
    if(system(cmd)) {}
    g_host_ext_root = root;

    Storage storage = {0};
    AppState app;
    printf("==== NAVTEST %s (%s) ====\n", path, sel);
    int rc = host_boot(&app, &storage, path, sel);
    if(rc) { printf("BOOT FAIL %d\n", rc); return rc; }

    printf("post-unlock: unlocked=%d fb_nz=%d hash=%08X\n",
           emu_is_unlocked(&app), fb_nonzero(&app), fbhash(&app));
    if(!emu_is_unlocked(&app)) { printf("NOT UNLOCKED, abort\n"); return 1; }

    /* --- (A) direct nav_button primitive: does the menu move at all? --- */
    printf("\n-- (A) direct nav_button --\n");
    int moved_a = 0;
    uint32_t h = fbhash(&app);
    int seq_a[] = { B_DOWN, B_DOWN, B_UP, B_OK, B_DOWN };
    for(int i = 0; i < 5; i++) {
        nav_button(&app, seq_a[i], false);
        uint32_t nh = fbhash(&app);
        printf("  nav %d: fb_nz=%d hash=%08X %s\n",
               seq_a[i], fb_nonzero(&app), nh, nh != h ? "CHANGED" : "same");
        if(nh != h) moved_a++;
        h = nh;
    }
    printf("  (A) frames changed: %d/5\n", moved_a);

    /* --- (B) full bridge path: input -> FK_TO_BTN -> nav_button -> run --- */
    printf("\n-- (B) bridge state-machine (FK_* keys) --\n");
    BridgeState bs; memset(&bs, 0, sizeof(bs));
    int moved_b = 0;
    h = fbhash(&app);
    /* Use Flipper-key bits (what poll_raw_keys would return). Avoid FK_UP's
     * exit-hold and FK_BACK's RF-hold by short taps only. */
    /* Flipper RIGHT->B_UP (menu/cell+), Flipper BACK->B_DOWN (menu/cell-): the
     * two real keyfob nav buttons. Use these to exercise menu movement. */
    struct { const char* name; uint8_t kbit; } taps[] = {
        {"RIGHT", KBIT_RIGHT}, {"RIGHT", KBIT_RIGHT}, {"BACK", KBIT_BACK},
        {"RIGHT", KBIT_RIGHT}, {"BACK", KBIT_BACK}, {"BACK", KBIT_BACK},
    };
    for(int i = 0; i < 6; i++) {
        uint32_t of0 = app.oled_frames;
        bridge_tap(&app, &bs, taps[i].kbit);
        uint32_t nh = fbhash(&app);
        printf("  flipper %-5s: fb_nz=%d hash=%08X oled_frames+%u %s\n",
               taps[i].name, fb_nonzero(&app), nh, app.oled_frames - of0,
               nh != h ? "CHANGED(render)" : "same");
        if(nh != h) moved_b++;
        h = nh;
    }
    printf("  (B) frames changed: %d/6\n", moved_b);

    printf("\n==== RESULT: A=%d/5 B=%d/6 %s ====\n", moved_a, moved_b,
           (moved_a >= 2 && moved_b >= 2) ? "NAV WORKS" : "NAV BROKEN");
    return 0;
}
