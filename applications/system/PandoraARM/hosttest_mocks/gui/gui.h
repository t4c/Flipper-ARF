#ifndef HOST_GUI_MOCK_H
#define HOST_GUI_MOCK_H
#include "../furi.h"

typedef struct { int dummy; } Gui;
typedef struct { int dummy; } Canvas;
typedef enum { CanvasOrientationHorizontal } CanvasOrientation;
typedef enum { FontPrimary, FontSecondary } Font;
typedef enum { AlignCenter, AlignLeft, AlignRight } Align;

typedef void (*GuiCanvasCommitCallback)(uint8_t* data, size_t size, CanvasOrientation o, void* ctx);

static inline Canvas* gui_direct_draw_acquire(Gui* g) { (void)g; static Canvas c; return &c; }
static inline void gui_direct_draw_release(Gui* g) { (void)g; }
static inline void gui_add_framebuffer_callback(Gui* g, GuiCanvasCommitCallback cb, void* ctx) { (void)g; (void)cb; (void)ctx; }
static inline void gui_remove_framebuffer_callback(Gui* g, GuiCanvasCommitCallback cb, void* ctx) { (void)g; (void)cb; (void)ctx; }
static inline void gui_commit(Gui* g) { (void)g; }

static inline void canvas_reset(Canvas* c) { (void)c; }
static inline void canvas_clear(Canvas* c) { (void)c; }
static inline void canvas_commit(Canvas* c) { (void)c; }
static inline void canvas_set_font(Canvas* c, Font f) { (void)c; (void)f; }
static inline void canvas_draw_str(Canvas* c, int x, int y, const char* s) { (void)c; (void)x; (void)y; (void)s; }
static inline void canvas_draw_line(Canvas* c, int x0, int y0, int x1, int y1) { (void)c; (void)x0; (void)y0; (void)x1; (void)y1; }

#endif
