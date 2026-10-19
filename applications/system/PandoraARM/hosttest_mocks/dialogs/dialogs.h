#ifndef HOST_DIALOGS_MOCK_H
#define HOST_DIALOGS_MOCK_H
#include "../furi.h"
typedef struct { int dummy; } DialogsApp;
typedef struct { int dummy; } DialogMessage;
typedef enum { DialogMessageButtonBack, DialogMessageButtonLeft, DialogMessageButtonCenter, DialogMessageButtonRight } DialogMessageButton;
typedef struct { const char* extension; void* icon; const char* base_path; bool hide_ext; bool skip_assets; } DialogsFileBrowserOptions;
static inline DialogMessage* dialog_message_alloc(void) { static DialogMessage d; return &d; }
static inline void dialog_message_free(DialogMessage* d) { (void)d; }
static inline void dialog_message_set_text(DialogMessage* d, const char* t, int x, int y, int ax, int ay) { (void)d; (void)t; (void)x; (void)y; (void)ax; (void)ay; }
static inline void dialog_message_set_buttons(DialogMessage* d, const char* l, const char* c, const char* r) { (void)d; (void)l; (void)c; (void)r; }
static inline DialogMessageButton dialog_message_show(DialogsApp* a, DialogMessage* d) { (void)a; (void)d; return DialogMessageButtonCenter; }
static inline void dialog_file_browser_set_basic_options(DialogsFileBrowserOptions* o, const char* ext, void* icon) { (void)o; (void)ext; (void)icon; }
static inline bool dialog_file_browser_show(DialogsApp* a, FuriString* path, FuriString* start, DialogsFileBrowserOptions* o) { (void)a; (void)path; (void)start; (void)o; return false; }
#endif
