/* Host mocks for the Flipper furi/gui/storage/hal APIs used by PandoraARM.
 * Enough to (a) -fsyntax-only main.c and (b) run the emulator core host-side.
 * Storage is backed by real stdio so flatten_firmware + the flash cache work. */
#ifndef HOST_FURI_MOCK_H
#define HOST_FURI_MOCK_H

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- logging ---- */
#ifndef HOST_QUIET
#define FURI_LOG_I(tag, ...) do { fprintf(stderr, "[I]" tag " "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define FURI_LOG_W(tag, ...) do { fprintf(stderr, "[W]" tag " "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define FURI_LOG_E(tag, ...) do { fprintf(stderr, "[E]" tag " "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define FURI_LOG_D(tag, ...) do { } while(0)
#else
#define FURI_LOG_I(tag, ...) do { } while(0)
#define FURI_LOG_W(tag, ...) do { } while(0)
#define FURI_LOG_E(tag, ...) do { } while(0)
#define FURI_LOG_D(tag, ...) do { } while(0)
#endif

#define furi_check(x) do { if(!(x)) { fprintf(stderr, "furi_check failed\n"); abort(); } } while(0)
#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif
#ifndef COUNT_OF
#define COUNT_OF(a) (sizeof(a) / sizeof((a)[0]))
#endif

typedef enum { FuriStatusOk = 0, FuriStatusError = 1 } FuriStatus;

/* ---- time ---- */
static inline uint32_t furi_get_tick(void) { return 0; }
static inline void furi_delay_ms(uint32_t ms) { (void)ms; }
static inline void furi_delay_until_tick(uint32_t t) { (void)t; }

/* ---- memory ---- */
static inline size_t memmgr_get_free_heap(void) { return 256u * 1024u; }
static inline size_t memmgr_heap_get_max_free_block(void) { return 256u * 1024u; }

/* ---- mutex ---- */
typedef int FuriMutex;
typedef enum { FuriMutexTypeNormal } FuriMutexType;
#define FuriWaitForever 0xFFFFFFFFu
static inline FuriMutex* furi_mutex_alloc(FuriMutexType t) { (void)t; static FuriMutex m; return &m; }
static inline void furi_mutex_free(FuriMutex* m) { (void)m; }
static inline FuriStatus furi_mutex_acquire(FuriMutex* m, uint32_t to) { (void)m; (void)to; return FuriStatusOk; }
static inline void furi_mutex_release(FuriMutex* m) { (void)m; }

/* ---- records ---- */
#define RECORD_GUI "gui"
#define RECORD_DIALOGS "dialogs"
#define RECORD_STORAGE "storage"
static inline void* furi_record_open(const char* n) { (void)n; static int dummy; return &dummy; }
static inline void furi_record_close(const char* n) { (void)n; }

/* ---- FuriString ---- */
typedef struct { char* s; } FuriString;
static inline char* host_strdup(const char* s) {
    size_t n = strlen(s) + 1;
    char* d = (char*)malloc(n);
    if(d) memcpy(d, s, n);
    return d;
}
static inline FuriString* furi_string_alloc_set_str(const char* s) {
    FuriString* f = (FuriString*)malloc(sizeof(FuriString));
    f->s = host_strdup(s ? s : "");
    return f;
}
static inline void furi_string_free(FuriString* f) { if(f) { free(f->s); free(f); } }
static inline const char* furi_string_get_cstr(FuriString* f) { return f ? f->s : ""; }

/* ---- GPIO / light / speaker (HAL) ---- */
typedef int GpioPin;
static inline bool furi_hal_gpio_read(const GpioPin* p) { (void)p; return true; }
typedef enum { LightRed, LightGreen, LightBlue } Light;
static inline void furi_hal_light_set(Light l, uint8_t v) { (void)l; (void)v; }
static inline bool furi_hal_speaker_acquire(uint32_t to) { (void)to; return false; }
static inline void furi_hal_speaker_release(void) {}
static inline void furi_hal_speaker_start(float f, float v) { (void)f; (void)v; }
static inline void furi_hal_speaker_stop(void) {}

#endif /* HOST_FURI_MOCK_H */
