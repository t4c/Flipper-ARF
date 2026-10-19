#ifndef HOST_VD_MOCK_H
#define HOST_VD_MOCK_H
#include "gui.h"
typedef struct { int dummy; } ViewDispatcher;
typedef struct { int dummy; } View;
typedef enum { ViewDispatcherTypeFullscreen } ViewDispatcherType;
typedef bool (*ViewDispatcherNavigationEventCallback)(void* ctx);
static inline ViewDispatcher* view_dispatcher_alloc(void) { static ViewDispatcher v; return &v; }
static inline void view_dispatcher_free(ViewDispatcher* v) { (void)v; }
static inline void view_dispatcher_set_event_callback_context(ViewDispatcher* v, void* c) { (void)v; (void)c; }
static inline void view_dispatcher_set_navigation_event_callback(ViewDispatcher* v, ViewDispatcherNavigationEventCallback cb) { (void)v; (void)cb; }
static inline void view_dispatcher_add_view(ViewDispatcher* v, uint32_t id, View* view) { (void)v; (void)id; (void)view; }
static inline void view_dispatcher_remove_view(ViewDispatcher* v, uint32_t id) { (void)v; (void)id; }
static inline void view_dispatcher_attach_to_gui(ViewDispatcher* v, Gui* g, ViewDispatcherType t) { (void)v; (void)g; (void)t; }
static inline void view_dispatcher_switch_to_view(ViewDispatcher* v, uint32_t id) { (void)v; (void)id; }
static inline void view_dispatcher_run(ViewDispatcher* v) { (void)v; }
static inline void view_dispatcher_stop(ViewDispatcher* v) { (void)v; }
#endif
