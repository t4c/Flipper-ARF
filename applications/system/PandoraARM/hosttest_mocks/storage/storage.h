/* Host storage mock backed by real stdio (so flatten_firmware + flash cache work). */
#ifndef HOST_STORAGE_MOCK_H
#define HOST_STORAGE_MOCK_H
#include "../furi.h"

typedef struct { int dummy; } Storage;

typedef struct {
    FILE* fp;
    char path[512];
    bool open;
} File;

typedef enum { FSAM_READ = 1, FSAM_WRITE = 2, FSAM_READ_WRITE = 3 } FS_AccessMode;
typedef enum { FSOM_OPEN_EXISTING = 1, FSOM_OPEN_ALWAYS = 2, FSOM_CREATE_ALWAYS = 3 } FS_OpenMode;
typedef enum { FSE_OK = 0, FSE_ERR = 1, FSE_EXIST = 2 } FS_Error;

/* The host harness sets this so '/ext/...' paths map to a local temp dir. */
extern const char* g_host_ext_root;

static inline void host_map_path(const char* in, char* out, size_t cap) {
    if(g_host_ext_root && strncmp(in, "/ext", 4) == 0) {
        snprintf(out, cap, "%s%s", g_host_ext_root, in + 4);
    } else {
        snprintf(out, cap, "%s", in);
    }
}

static inline File* storage_file_alloc(Storage* s) {
    (void)s;
    File* f = (File*)calloc(1, sizeof(File));
    return f;
}
static inline void storage_file_free(File* f) {
    if(f) { if(f->open && f->fp) fclose(f->fp); free(f); }
}
static inline bool storage_file_open(File* f, const char* path, FS_AccessMode am, FS_OpenMode om) {
    if(!f) return false;
    host_map_path(path, f->path, sizeof(f->path));
    const char* mode;
    if(om == FSOM_CREATE_ALWAYS) mode = "w+b";
    else if(am == FSAM_READ) mode = "rb";
    else mode = "r+b";
    f->fp = fopen(f->path, mode);
    if(!f->fp && (am == FSAM_READ_WRITE)) f->fp = fopen(f->path, "w+b");
    f->open = (f->fp != NULL);
    return f->open;
}
static inline void storage_file_close(File* f) {
    if(f && f->open && f->fp) { fclose(f->fp); f->fp = NULL; f->open = false; }
}
static inline size_t storage_file_read(File* f, void* buf, size_t n) {
    if(!f || !f->fp) return 0;
    return fread(buf, 1, n, f->fp);
}
static inline size_t storage_file_write(File* f, const void* buf, size_t n) {
    if(!f || !f->fp) return 0;
    return fwrite(buf, 1, n, f->fp);
}
static inline bool storage_file_seek(File* f, uint32_t off, bool from_start) {
    if(!f || !f->fp) return false;
    return fseek(f->fp, (long)off, from_start ? SEEK_SET : SEEK_CUR) == 0;
}
static inline uint64_t storage_file_size(File* f) {
    if(!f || !f->fp) return 0;
    long cur = ftell(f->fp);
    fseek(f->fp, 0, SEEK_END);
    long n = ftell(f->fp);
    fseek(f->fp, cur, SEEK_SET);
    return (uint64_t)n;
}
static inline FS_Error storage_common_mkdir(Storage* s, const char* path) {
    (void)s;
    char mapped[512]; host_map_path(path, mapped, sizeof(mapped));
    char cmd[600]; snprintf(cmd, sizeof(cmd), "mkdir -p '%s'", mapped);
    (void)system(cmd);
    return FSE_OK;
}
static inline FS_Error storage_common_remove(Storage* s, const char* path) {
    (void)s;
    char mapped[512]; host_map_path(path, mapped, sizeof(mapped));
    return remove(mapped) == 0 ? FSE_OK : FSE_ERR;
}

#endif
