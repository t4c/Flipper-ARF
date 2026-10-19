#ifndef HOST_WIDGET_MOCK_H
#define HOST_WIDGET_MOCK_H
#include "../view_dispatcher.h"
typedef struct { int dummy; } Widget;
static inline Widget* widget_alloc(void) { static Widget w; return &w; }
static inline void widget_free(Widget* w) { (void)w; }
static inline void widget_reset(Widget* w) { (void)w; }
static inline void widget_add_text_scroll_element(Widget* w, int x, int y, int ww, int h, const char* t) { (void)w; (void)x; (void)y; (void)ww; (void)h; (void)t; }
static inline View* widget_get_view(Widget* w) { (void)w; static View v; return &v; }
#endif
