#ifndef HOST_SUBGHZ_MOCK_H
#define HOST_SUBGHZ_MOCK_H
#include "furi.h"

/* LevelDuration (Flipper type) */
typedef struct { bool level; uint32_t duration; bool reset; } LevelDuration;
static inline LevelDuration level_duration_make(bool l, uint32_t d) { LevelDuration x = { l, d, false }; return x; }
static inline LevelDuration level_duration_reset(void) { LevelDuration x = { false, 0, true }; return x; }

typedef LevelDuration (*FuriHalSubGhzAsyncTxCallback)(void* ctx);
typedef void (*FuriHalSubGhzCaptureCallback)(bool level, uint32_t duration, void* ctx);

static inline void furi_hal_subghz_reset(void) {}
static inline void furi_hal_subghz_idle(void) {}
static inline void furi_hal_subghz_sleep(void) {}
static inline void furi_hal_subghz_load_custom_preset(const uint8_t* p) { (void)p; }
static inline uint32_t furi_hal_subghz_set_frequency_and_path(uint32_t f) { return f; }
static inline bool furi_hal_subghz_is_tx_allowed(uint32_t f) { (void)f; return true; }
static inline float furi_hal_subghz_get_rssi(void) { return -110.0f; }
static inline bool furi_hal_subghz_start_async_tx(FuriHalSubGhzAsyncTxCallback cb, void* ctx) { (void)cb; (void)ctx; return false; }
static inline bool furi_hal_subghz_is_async_tx_complete(void) { return true; }
static inline void furi_hal_subghz_stop_async_tx(void) {}
static inline void furi_hal_subghz_start_async_rx(FuriHalSubGhzCaptureCallback cb, void* ctx) { (void)cb; (void)ctx; }
static inline void furi_hal_subghz_stop_async_rx(void) {}
#endif
