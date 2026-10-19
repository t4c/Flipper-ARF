#ifndef HOST_SUBMENU_MOCK_H
#define HOST_SUBMENU_MOCK_H
#include "../view_dispatcher.h"
typedef struct { int dummy; } Submenu;
typedef void (*SubmenuItemCallback)(void* ctx, uint32_t index);
static inline Submenu* submenu_alloc(void) { static Submenu s; return &s; }
static inline void submenu_free(Submenu* s) { (void)s; }
static inline void submenu_reset(Submenu* s) { (void)s; }
static inline void submenu_set_header(Submenu* s, const char* h) { (void)s; (void)h; }
static inline void submenu_add_item(Submenu* s, const char* l, uint32_t i, SubmenuItemCallback cb, void* ctx) { (void)s; (void)l; (void)i; (void)cb; (void)ctx; }
static inline View* submenu_get_view(Submenu* s) { (void)s; static View v; return &v; }
#endif
