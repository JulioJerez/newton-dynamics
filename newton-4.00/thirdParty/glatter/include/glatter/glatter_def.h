/*
Copyright 2018 Ioannis Makris

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable:4996) // mbstowcs and friends: legacy WinAPI usage is intentional
#endif

#include <inttypes.h>

#include <glatter/glatter_config.h>
#include <glatter/glatter_atomic.h>
#include <glatter/glatter_platform_headers.h>
#include <glatter/glatter_masprintf.h>
#include <glatter/glatter_once.h>

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#if defined(__GNUC__) || defined(__clang__)
#   define GLATTER_UNUSED __attribute__((unused))
#else
#   define GLATTER_UNUSED
#endif

#if defined(_WIN32)
#   include <windows.h>
#   include <tchar.h>
#   if !defined(LOAD_LIBRARY_SEARCH_SYSTEM32)
#       define LOAD_LIBRARY_SEARCH_SYSTEM32 0x00000800
#   endif
#   if !defined(GLATTER_THREAD_LOCAL)
#       if defined(_MSC_VER)
#           define GLATTER_THREAD_LOCAL __declspec(thread)
#       elif defined(__GNUC__)
#           define GLATTER_THREAD_LOCAL __thread
#       else
#           define GLATTER_THREAD_LOCAL
#           if !defined(GLATTER_NO_TLS_WARNING)
#               define GLATTER_NO_TLS_WARNING
#               if defined(_MSC_VER)
#                   pragma message("glatter: TLS not detected for this compiler; some logs may interleave across threads.")
#               else
#                   warning "glatter: TLS not detected for this compiler; some logs may interleave across threads."
#               endif
#           endif
#       endif
#   endif
#else
#   if !defined(GLATTER_THREAD_LOCAL)
#       if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#           define GLATTER_THREAD_LOCAL _Thread_local
#       elif defined(__GNUC__)
#           define GLATTER_THREAD_LOCAL __thread
#       else
#           define GLATTER_THREAD_LOCAL
#           if !defined(GLATTER_NO_TLS_WARNING)
#               define GLATTER_NO_TLS_WARNING
#               if defined(_MSC_VER)
#                   pragma message("glatter: TLS not detected for this compiler; some logs may interleave across threads.")
#               else
#                   warning "glatter: TLS not detected for this compiler; some logs may interleave across threads."
#               endif
#           endif
#       endif
#   endif
#endif

#if defined(GLATTER_HEADER_ONLY)
#   if defined(__cplusplus) && __cplusplus >= 201703L
#       define GLATTER_LINKONCE_DECORATION
#       define GLATTER_LINKONCE_STORAGE inline
#   elif defined(_MSC_VER)
#       define GLATTER_LINKONCE_DECORATION __declspec(selectany)
#       define GLATTER_LINKONCE_STORAGE
#   elif defined(__GNUC__) || defined(__clang__)
#       define GLATTER_LINKONCE_DECORATION __attribute__((weak))
#       define GLATTER_LINKONCE_STORAGE
#   else
#       error "glatter: header-only mode requires inline variables (C++17) or selectany/weak support."
#   endif
#else
#   define GLATTER_LINKONCE_DECORATION
#   define GLATTER_LINKONCE_STORAGE static
#endif

#define GLATTER_LINKONCE GLATTER_LINKONCE_DECORATION GLATTER_LINKONCE_STORAGE

#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
    #include <dlfcn.h>
    #include <pthread.h>
#endif

/*
 * In non header-only builds, include this header only from glatter.c so the
 * non-inline definitions remain unique.
 */


#ifndef GLATTER_EXTERN_C_BEGIN
    #ifdef __cplusplus
        #define GLATTER_EXTERN_C_BEGIN extern "C" {
        #define GLATTER_EXTERN_C_END }
    #else
        #define GLATTER_EXTERN_C_BEGIN
        #define GLATTER_EXTERN_C_END
    #endif
#endif

GLATTER_EXTERN_C_BEGIN


 #ifdef GLATTER_HEADER_ONLY
    #undef GLATTER_INLINE_OR_NOT
    #define GLATTER_INLINE_OR_NOT static inline
 #else
    #undef GLATTER_INLINE_OR_NOT
    #define GLATTER_INLINE_OR_NOT
 #endif

#if defined(GLATTER_HEADER_ONLY) && defined(__cplusplus)
    #if __cplusplus < 201103L && !defined(_MSC_VER)
        #error "Header-only mode requires C++11 for thread-safe local statics."
    #endif
    #if defined(_MSC_VER) && _MSC_VER < 1900
        #error "Header-only mode requires MSVC 2015 (19.0) or newer."
    #endif
#endif



GLATTER_INLINE_OR_NOT
void glatter_default_log_handler(const char* str)
{
    fprintf(stderr, "%s", str);
}


typedef void (*glatter_log_handler_fn)(const char*);

static glatter_atomic(glatter_log_handler_fn) glatter_log_handler_state =
    GLATTER_ATOMIC_INIT_PTR(glatter_default_log_handler);

/* Log handler is frozen after the first log to avoid races with late setters. */
static glatter_atomic_int glatter_log_handler_frozen = GLATTER_ATOMIC_INT_INIT(0);

GLATTER_INLINE_OR_NOT
void glatter_log_handler_store(glatter_log_handler_fn handler_ptr)
{
    GLATTER_ATOMIC_STORE(glatter_log_handler_state, handler_ptr);
}

GLATTER_INLINE_OR_NOT
glatter_log_handler_fn glatter_log_handler_load(void)
{
    return (glatter_log_handler_fn)GLATTER_ATOMIC_LOAD(glatter_log_handler_state);
}


GLATTER_INLINE_OR_NOT
void (*glatter_log_handler())(const char*)
{
    return glatter_log_handler_load();
}


static const char glatter_log_fallback_message[] = "GLATTER: message formatting failed.\n";

static const char* glatter_log_stable_message(const char* message)
{
    enum { GLATTER_LOG_BUFFER_SLOTS = 8, GLATTER_LOG_BUFFER_CAP = 2048 };
    static GLATTER_THREAD_LOCAL char glatter_log_buffer_slots[GLATTER_LOG_BUFFER_SLOTS][GLATTER_LOG_BUFFER_CAP];
    static GLATTER_THREAD_LOCAL unsigned glatter_log_buffer_index;
    static GLATTER_THREAD_LOCAL char* glatter_log_heap_buffer = NULL;
    static GLATTER_THREAD_LOCAL size_t glatter_log_heap_capacity = 0;

    size_t len = strlen(message);
    if (len < GLATTER_LOG_BUFFER_CAP) {
        char* slot = glatter_log_buffer_slots[glatter_log_buffer_index++ % GLATTER_LOG_BUFFER_SLOTS];
        memcpy(slot, message, len + 1);
        return slot;
    }

    size_t needed = len + 1;
    if (glatter_log_heap_capacity < needed) {
        char* new_heap = (char*)realloc(glatter_log_heap_buffer, needed);
        if (!new_heap) {
            return NULL;
        }
        glatter_log_heap_buffer = new_heap;
        glatter_log_heap_capacity = needed;
    }
    memcpy(glatter_log_heap_buffer, message, needed);
    return glatter_log_heap_buffer;
}

GLATTER_INLINE_OR_NOT
const char* glatter_log(const char* str)
{
    const char* message = str ? str : glatter_log_fallback_message;
    const char* stable = glatter_log_stable_message(message);
    if (!stable) {
        /* Allocation failed for a long message; fall back to a static string. */
        message = glatter_log_fallback_message;
        stable = glatter_log_stable_message(message);
        if (!stable) {
            stable = glatter_log_fallback_message;
        }
    }
    /* Freeze the handler on first log, race-free. */
    int expected = 0;
    (void)GLATTER_ATOMIC_INT_CAS(glatter_log_handler_frozen, expected, 1);
    glatter_log_handler_fn handler = glatter_log_handler_load();
    handler(stable);
    return str;
}


GLATTER_INLINE_OR_NOT
void glatter_log_printf(const char* fmt, ...)
{
    char buffer[2048];
    va_list args;
    va_start(args, fmt);
    va_list args_copy;
    va_copy(args_copy, args);
    int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
    if (written >= 0 && written < (int)sizeof(buffer)) {
        glatter_log(buffer);
    }
    else
    if (written >= 0) {
        /* need written+1 bytes */
        char* heap = (char*)malloc((size_t)written + 1);
        if (heap) {
            vsnprintf(heap, (size_t)written + 1, fmt, args_copy);
            glatter_log(heap);
            free(heap);
        }
        else {
            glatter_log(NULL);
        }
    }
    else {
        glatter_log(NULL);
    }
    va_end(args_copy);
    va_end(args);
}


GLATTER_INLINE_OR_NOT
void glatter_set_log_handler(void(*handler_ptr)(const char*))
{
    /* If already frozen by the first log, ignore late changes. */
    if (GLATTER_ATOMIC_INT_LOAD(glatter_log_handler_frozen)) {
        return;
    }
    if (handler_ptr == NULL) {
        handler_ptr = glatter_default_log_handler;
    }
    glatter_log_handler_store(handler_ptr);
}

#if defined(_WIN32)
/* -------- TCHAR* logging helper (Windows only) -------- */
/* Prints TCHAR* as UTF-8 for logging.
 * Uses a small TLS ring so multiple %s in one call do not alias.
 */
static const char* glatter_pr_tstr(const TCHAR* ts) GLATTER_UNUSED;
static const char* glatter_pr_tstr(const TCHAR* ts)
{
    if (!ts) {
        return "(null)";
    }
#if defined(UNICODE) || defined(_UNICODE)
    enum { GLATTER_TSTR_SLOTS = 8, GLATTER_TSTR_CAP = 1024 };
    static GLATTER_THREAD_LOCAL char  tbuf[GLATTER_TSTR_SLOTS][GLATTER_TSTR_CAP];
    static GLATTER_THREAD_LOCAL unsigned tix;
    char* buf = tbuf[tix++ % GLATTER_TSTR_SLOTS];
    /* Convert UTF-16 -> UTF-8 */
    int n = WideCharToMultiByte(CP_UTF8, 0, ts, -1, buf, (int)GLATTER_TSTR_CAP, NULL, NULL);
    if (n > 0) {
        return buf;
    }
    /* Fallback on conversion failure */
    buf[0] = '?';
    buf[1] = '\0';
    return buf;
#else
    return (const char*)ts;
#endif
}
#endif /* _WIN32 */


typedef int glatter_wsi_t;

#ifndef GLATTER_WSI_DECIDING_VALUE
#define GLATTER_WSI_DECIDING_VALUE (-1)
#endif

#ifndef GLATTER_WSI_AUTO_VALUE
#define GLATTER_WSI_AUTO_VALUE 0
#define GLATTER_WSI_WGL_VALUE  1
#define GLATTER_WSI_GLX_VALUE  2
#define GLATTER_WSI_EGL_VALUE  3
#endif

#if defined(_WIN32)
typedef void* (WINAPI *glatter_egl_get_proc_fn)(const char*);
static const char* const glatter_windows_egl_names[] = {
    "libEGL.dll",
    "EGL.dll"
};

#define GLATTER_WINDOWS_GLES_MODULE_COUNT 3
static const char* const glatter_windows_gles_names[GLATTER_WINDOWS_GLES_MODULE_COUNT][4] = {
    { "libGLESv3.dll", "GLESv3.dll", NULL, NULL },
    { "libGLESv2.dll", "GLESv2.dll", NULL, NULL },
    { "libGLESv1_CM.dll", "GLESv1_CM.dll", "libGLES_CM.dll", "GLES_CM.dll" }
};
#else
#define GLATTER_GL_SONAME_COUNT 2
#define GLATTER_EGL_SONAME_COUNT 2
#define GLATTER_GLES_SONAME_COUNT 8
static const char* const glatter_gl_sonames[GLATTER_GL_SONAME_COUNT] = {
    "libGL.so.1",
    "libGL.so"
};
static const char* const glatter_egl_sonames[GLATTER_EGL_SONAME_COUNT] = {
    "libEGL.so.1",
    "libEGL.so"
};
static const char* const glatter_gles_sonames[GLATTER_GLES_SONAME_COUNT] = {
    "libGLESv3.so.3",
    "libGLESv3.so",
    "libGLESv2.so.2",
    "libGLESv2.so",
    "libGLESv1_CM.so.1",
    "libGLESv1_CM.so",
    "libGLES_CM.so.1",
    "libGLES_CM.so"
};
#endif

typedef struct glatter_loader_state {
    /* Loader WSI fields are atomic to avoid UB when multiple threads first-touch AUTO.
     * glatter_wsi_gate still serializes the ?decide once? phase. */
    glatter_atomic_int requested;    /* GLATTER_WSI_* enum values */
    glatter_atomic_int active;       /* GLATTER_WSI_* enum values */
    glatter_atomic_int wsi_explicit; /* 0/1 */
    glatter_atomic_int env_checked;
#if defined(_WIN32)
    HMODULE opengl32_module;
    HMODULE egl_module;
    HMODULE gles_modules[GLATTER_WINDOWS_GLES_MODULE_COUNT];
    PROC (WINAPI *wgl_get_proc)(LPCSTR);
    glatter_egl_get_proc_fn egl_get_proc;
#else
    /* Loader handles remain open for the process lifetime by design. We do not
     * dlclose/FreeLibrary to avoid driver/layer teardown side effects.
     */
    void* gl_handles[GLATTER_GL_SONAME_COUNT];
    void* egl_handles[GLATTER_EGL_SONAME_COUNT];
    void* gles_handles[GLATTER_GLES_SONAME_COUNT];
    void* (*glx_get_proc)(const GLubyte*);
    void* (*egl_get_proc)(const char*);
#endif
#if defined(GLATTER_GLX) && !defined(GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER)
    /* Atomic to avoid redundant installs/logs when multiple threads first-touch GLX. */
    glatter_atomic_int glx_error_handler_installed;
#endif
} glatter_loader_state;

/* THREADING NOTE:
 * Per-entry resolution uses atomic CAS and is thread-safe.
 * Loader state is static and updated in a read-mostly, idempotent way.
 */
/* WSI decision gate:

 * 0 = undecided, 1 = deciding, 2 = decided
 * In header-only builds this accessor returns the process-wide gate, ensuring
 * that all translation units share the same latch state.
 */
static glatter_atomic_int* glatter_wsi_gate_get(void)
{
    static glatter_atomic_int gate = GLATTER_ATOMIC_INT_INIT(0);
    return &gate;
}

GLATTER_LINKONCE glatter_loader_state glatter_loader_state_singleton = {
#if defined(GLATTER_WGL) && !defined(GLATTER_GLX) && !defined(GLATTER_EGL)
        GLATTER_ATOMIC_INT_INIT(GLATTER_WSI_WGL_VALUE),
#elif defined(GLATTER_GLX) && !defined(GLATTER_EGL) && !defined(GLATTER_WGL)
        GLATTER_ATOMIC_INT_INIT(GLATTER_WSI_GLX_VALUE),
#elif defined(GLATTER_EGL) && !defined(GLATTER_GLX) && !defined(GLATTER_WGL)
        GLATTER_ATOMIC_INT_INIT(GLATTER_WSI_EGL_VALUE),
#else
        GLATTER_ATOMIC_INT_INIT(GLATTER_WSI_AUTO_VALUE),
#endif
        GLATTER_ATOMIC_INT_INIT(GLATTER_WSI_AUTO_VALUE),
#if (defined(GLATTER_WGL) && !defined(GLATTER_GLX) && !defined(GLATTER_EGL)) || \
    (defined(GLATTER_GLX) && !defined(GLATTER_EGL) && !defined(GLATTER_WGL)) || \
    (defined(GLATTER_EGL) && !defined(GLATTER_GLX) && !defined(GLATTER_WGL))
        /* wsi_explicit: exactly one WSI is compiled-in -> ignore GLATTER_WSI */
        GLATTER_ATOMIC_INT_INIT(1),
#else
        /* wsi_explicit: multiple WSIs possible -> allow GLATTER_WSI to steer */
        GLATTER_ATOMIC_INT_INIT(0),
#endif
        /* env_checked */ GLATTER_ATOMIC_INT_INIT(0),
#if defined(_WIN32)
        /* opengl32_module */ NULL,
        /* egl_module */ NULL,
        /* gles_modules */ { NULL },
        /* wgl_get_proc */ NULL,
        /* egl_get_proc */ NULL,
#else
        /* gl_handles */ { NULL },
        /* egl_handles */ { NULL },
        /* gles_handles */ { NULL },
        /* glx_get_proc */ NULL,
        /* egl_get_proc */ NULL,
#endif
#if defined(GLATTER_GLX) && !defined(GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER)
        /* glx_error_handler_installed */ GLATTER_ATOMIC_INT_INIT(0)
#endif
    };

static glatter_loader_state* glatter_loader_state_get(void)
{
    return &glatter_loader_state_singleton;
}

#undef GLATTER_LINKONCE
#undef GLATTER_LINKONCE_STORAGE
#undef GLATTER_LINKONCE_DECORATION

static int glatter_equals_ignore_case(const char* a, const char* b)
{
    if (!a || !b) {
        return 0;
    }
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return 0;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

/* Windows module loading helpers */
#if defined(_WIN32)
/* Secure-by-default DLL load (Windows):
 * - Prefer LOAD_LIBRARY_SEARCH_SYSTEM32 when available.
 * - Fallback: build an absolute path to %SystemRoot%\System32.
 */
static HMODULE glatter_load_system32_dll_(const wchar_t* dll_w)
{
#if !defined(GLATTER_UNSAFE_DLL_SEARCH)
    HMODULE h = LoadLibraryExW(dll_w, NULL, LOAD_LIBRARY_SEARCH_SYSTEM32);
    if (h) {
        return h;
    }

    wchar_t sysdir[MAX_PATH];
    UINT n = GetSystemDirectoryW(sysdir, MAX_PATH);
    if (n > 0 && n < MAX_PATH) {
        wchar_t full[MAX_PATH];
        if (swprintf(full, MAX_PATH, L"%s\\%s", sysdir, dll_w) > 0) {
            h = LoadLibraryW(full);
            if (h) {
                return h;
            }
        }
    }

    return LoadLibraryW(dll_w);
#else
    return LoadLibraryW(dll_w);
#endif
}

static BOOL CALLBACK glatter_init_wgl_loader_once(PINIT_ONCE once, PVOID param, PVOID* context);
static BOOL CALLBACK glatter_init_egl_loader_once(PINIT_ONCE once, PVOID param, PVOID* context);
static BOOL CALLBACK glatter_init_gles_loader_once(PINIT_ONCE o, PVOID p, PVOID* c);
static INIT_ONCE glatter_wgl_loader_once = INIT_ONCE_STATIC_INIT;
static INIT_ONCE glatter_egl_loader_once = INIT_ONCE_STATIC_INIT;
static INIT_ONCE glatter_gles_loader_once = INIT_ONCE_STATIC_INIT;

static void* glatter_windows_resolve_wgl(glatter_loader_state* state, const char* name)
{
    InitOnceExecuteOnce(&glatter_wgl_loader_once, glatter_init_wgl_loader_once, NULL, NULL);

    if (state->wgl_get_proc) {
        PROC proc = state->wgl_get_proc(name);
        /* wglGetProcAddress may return small integer sentinels. These are not valid pointers. */
        if (proc && proc != (PROC)1 && proc != (PROC)2 && proc != (PROC)3 && proc != (PROC)-1) {
            return (void*)proc;
        }
    }

    if (state->opengl32_module) {
        return (void*)GetProcAddress(state->opengl32_module, name);
    }

    return NULL;
}

static void* glatter_windows_resolve_egl(glatter_loader_state* state, const char* name)
{
    InitOnceExecuteOnce(&glatter_egl_loader_once, glatter_init_egl_loader_once, NULL, NULL);

    if (state->egl_get_proc) {
        void* sym = state->egl_get_proc(name);
        if (sym) {
            return sym;
        }
    }

    if (state->egl_module) {
        void* sym = (void*)GetProcAddress(state->egl_module, name);
        if (sym) {
            return sym;
        }
    }

    if (name && strncmp(name, "gl", 2) == 0) {
        InitOnceExecuteOnce(&glatter_gles_loader_once, glatter_init_gles_loader_once, NULL, NULL);
        for (size_t i = 0; i < GLATTER_WINDOWS_GLES_MODULE_COUNT; ++i) {
            if (state->gles_modules[i]) {
                void* sym = (void*)GetProcAddress(state->gles_modules[i], name);
                if (sym) {
                    return sym;
                }
            }
        }
    }

    return NULL;
}

static BOOL CALLBACK glatter_init_wgl_loader_once(PINIT_ONCE once, PVOID param, PVOID* context)
{
    (void)once; (void)param; (void)context;
    glatter_loader_state* state = glatter_loader_state_get();
    static const char* const opengl_names[] = { "opengl32.dll" };

    wchar_t dll_w[MAX_PATH];
    mbstowcs(dll_w, opengl_names[0], MAX_PATH);
    HMODULE mod = glatter_load_system32_dll_(dll_w);

    if (mod) {
        state->opengl32_module = mod;
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif
        state->wgl_get_proc = (PROC (WINAPI*)(LPCSTR))GetProcAddress(mod, "wglGetProcAddress");
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
    }
    return TRUE;
}

static BOOL CALLBACK glatter_init_egl_loader_once(PINIT_ONCE once, PVOID param, PVOID* context)
{
    (void)once; (void)param; (void)context;
    glatter_loader_state* state = glatter_loader_state_get();

    for (size_t i = 0; i < sizeof(glatter_windows_egl_names) / sizeof(glatter_windows_egl_names[0]); ++i) {
        wchar_t dll_w[MAX_PATH];
        mbstowcs(dll_w, glatter_windows_egl_names[i], MAX_PATH);
        HMODULE mod = glatter_load_system32_dll_(dll_w);
        if (mod) {
            state->egl_module = mod;
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif
            state->egl_get_proc = (glatter_egl_get_proc_fn)GetProcAddress(mod, "eglGetProcAddress");
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
            break;
        }
    }
    return TRUE;
}

static BOOL CALLBACK glatter_init_gles_loader_once(PINIT_ONCE o, PVOID p, PVOID* c)
{
    (void)o;(void)p;(void)c;
    glatter_loader_state* state = glatter_loader_state_get();
    for (size_t i = 0; i < GLATTER_WINDOWS_GLES_MODULE_COUNT; ++i) {
        if (!state->gles_modules[i]) {
            for (size_t j = 0; j < 4 && glatter_windows_gles_names[i][j]; ++j) {
                wchar_t dll_w[MAX_PATH]; mbstowcs(dll_w, glatter_windows_gles_names[i][j], MAX_PATH);
                HMODULE m = glatter_load_system32_dll_(dll_w);
                if (m) { state->gles_modules[i] = m; break; }
            }
        }
    }
    return TRUE;
}

static int glatter_windows_probe_wgl_(void)
{
    InitOnceExecuteOnce(&glatter_wgl_loader_once, glatter_init_wgl_loader_once, NULL, NULL);
    glatter_loader_state* state = glatter_loader_state_get();
    return state->wgl_get_proc != NULL || (state->opengl32_module && GetProcAddress(state->opengl32_module, "wglCreateContext") != NULL);
}

static int glatter_windows_probe_egl_(void)
{
    InitOnceExecuteOnce(&glatter_egl_loader_once, glatter_init_egl_loader_once, NULL, NULL);
    glatter_loader_state* state = glatter_loader_state_get();
    return state->egl_get_proc != NULL || (state->egl_module && GetProcAddress(state->egl_module, "eglGetDisplay") != NULL);
}

#else
/* POSIX (dlopen) helpers */

static void glatter_init_posix_loader_once(void);
static void glatter_init_gles_loader_once(void);
static pthread_once_t glatter_posix_loader_once = PTHREAD_ONCE_INIT;
static pthread_once_t glatter_gles_loader_once = PTHREAD_ONCE_INIT;

static void* glatter_linux_lookup_in_handles(void** handles, size_t count, const char* name)
{
    for (size_t i = 0; i < count; ++i) {
        if (handles[i]) {
            void* sym = dlsym(handles[i], name);
            if (sym) {
                return sym;
            }
        }
    }
    return NULL;
}

static void* glatter_linux_lookup_glx(glatter_loader_state* state, const char* name)
{
    pthread_once(&glatter_posix_loader_once, glatter_init_posix_loader_once);

    if (state->glx_get_proc) {
        void* sym = state->glx_get_proc((const GLubyte*)name);
        if (sym) {
            return sym;
        }
    }

    return glatter_linux_lookup_in_handles(state->gl_handles, GLATTER_GL_SONAME_COUNT, name);
}

static void* glatter_linux_lookup_egl(glatter_loader_state* state, const char* name)
{
    pthread_once(&glatter_posix_loader_once, glatter_init_posix_loader_once);

    if (state->egl_get_proc) {
        void* sym = state->egl_get_proc(name);
        if (sym) {
            return sym;
        }
    }

    void* sym = glatter_linux_lookup_in_handles(state->egl_handles, GLATTER_EGL_SONAME_COUNT, name);
    if (sym) {
        return sym;
    }

    if (name && strncmp(name, "gl", 2) == 0) {
        pthread_once(&glatter_gles_loader_once, glatter_init_gles_loader_once);
        return glatter_linux_lookup_in_handles(state->gles_handles, GLATTER_GLES_SONAME_COUNT, name);
    }

    return NULL;
}

static void glatter_init_posix_loader_once(void)
{
    glatter_loader_state* state = glatter_loader_state_get();
    
    for (size_t i = 0; i < GLATTER_GL_SONAME_COUNT; ++i) {
        if (!state->gl_handles[i]) {
            state->gl_handles[i] = dlopen(glatter_gl_sonames[i], RTLD_LAZY | RTLD_LOCAL);
        }
    }
    for (size_t i = 0; i < GLATTER_EGL_SONAME_COUNT; ++i) {
        if (!state->egl_handles[i]) {
            state->egl_handles[i] = dlopen(glatter_egl_sonames[i], RTLD_LAZY | RTLD_LOCAL);
        }
    }

    for (size_t i = 0; i < GLATTER_GL_SONAME_COUNT; ++i) {
        if (state->gl_handles[i]) {
            void* proc = dlsym(state->gl_handles[i], "glXGetProcAddressARB");
            if (!proc)
                proc = dlsym(state->gl_handles[i], "glXGetProcAddress");
            if (proc) {
               state->glx_get_proc = (void* (*)(const GLubyte*))proc;
                break;
            }
        }
    }
    for (size_t i = 0; i < GLATTER_EGL_SONAME_COUNT; ++i) {
        if (state->egl_handles[i]) {
            void* proc = dlsym(state->egl_handles[i], "eglGetProcAddress");
            if (proc) {
                state->egl_get_proc = (void* (*)(const char*))proc;
                break;
            }
        }
    }
}

static void glatter_init_gles_loader_once(void)
{
    glatter_loader_state* state = glatter_loader_state_get();
    for (size_t i = 0; i < GLATTER_GLES_SONAME_COUNT; ++i) {
        if (!state->gles_handles[i]) {
            state->gles_handles[i] = dlopen(glatter_gles_sonames[i], RTLD_LAZY | RTLD_LOCAL);
        }
    }
}

static int glatter_posix_probe_glx_(void)
{
    pthread_once(&glatter_posix_loader_once, glatter_init_posix_loader_once);
    glatter_loader_state* state = glatter_loader_state_get();
    for (size_t i = 0; i < GLATTER_GL_SONAME_COUNT; ++i) {
        if (!state->gl_handles[i]) {
            continue;
        }
        // If any of the key symbols are found, we have GLX support. Return immediately.
        if (dlsym(state->gl_handles[i], "glXGetProcAddressARB") ||
            dlsym(state->gl_handles[i], "glXGetProcAddress")    ||
            dlsym(state->gl_handles[i], "glXCreateContext"))
        {
            return 1; // Found
        }
    }
    return 0; // Not found
}

static int glatter_posix_probe_egl_(void)
{
    pthread_once(&glatter_posix_loader_once, glatter_init_posix_loader_once);
    glatter_loader_state* state = glatter_loader_state_get();
    for (size_t i = 0; i < GLATTER_EGL_SONAME_COUNT; ++i) {
        if (!state->egl_handles[i]) continue;
        if (dlsym(state->egl_handles[i], "eglGetProcAddress") || dlsym(state->egl_handles[i], "eglGetDisplay")) {
            return 1; // Found
        }
    }
    return 0; // Not found
}
#endif

static int glatter_normalize_requested_wsi_(int requested)
{
#if defined(_WIN32)
    if (requested == GLATTER_WSI_GLX_VALUE) {
        return GLATTER_WSI_AUTO_VALUE;
    }
#else
    if (requested == GLATTER_WSI_WGL_VALUE) {
        return GLATTER_WSI_AUTO_VALUE;
    }
#endif
#if !defined(GLATTER_GLX)
    if (requested == GLATTER_WSI_GLX_VALUE) {
        return GLATTER_WSI_AUTO_VALUE;
    }
#endif
#if !defined(GLATTER_EGL)
    if (requested == GLATTER_WSI_EGL_VALUE) {
        return GLATTER_WSI_AUTO_VALUE;
    }
#endif
    return requested;
}

/* Lock in WSI exactly once: {AUTO, DECIDING} -> {WGL,GLX,EGL}. Returns 1 if we won. */
static int glatter_select_wsi_once_(glatter_loader_state* state, int chosen)
{
    int expected = GLATTER_WSI_AUTO_VALUE;
    if (GLATTER_ATOMIC_INT_CAS(state->requested, expected, chosen)) {
        GLATTER_ATOMIC_INT_STORE(state->wsi_explicit, 1);
        return 1;
    }

    expected = GLATTER_WSI_DECIDING_VALUE;
    if (GLATTER_ATOMIC_INT_CAS(state->requested, expected, chosen)) {
        GLATTER_ATOMIC_INT_STORE(state->wsi_explicit, 1);
        return 1;
    }

    return 0;
}

static void glatter_decide_wsi_once_(glatter_loader_state* state)
{
    int requested = GLATTER_ATOMIC_INT_LOAD(state->requested);
    requested = glatter_normalize_requested_wsi_(requested);
    GLATTER_ATOMIC_INT_STORE(state->requested, requested);

    int wsi_explicit = GLATTER_ATOMIC_INT_LOAD(state->wsi_explicit);
    if (requested != GLATTER_WSI_AUTO_VALUE || wsi_explicit) {
        return;
    }

#if defined(_WIN32)
    if (!wsi_explicit) {
        if (glatter_windows_probe_wgl_()) {
            requested = GLATTER_WSI_WGL_VALUE;
            wsi_explicit = 1;
        }
    }
    if (!wsi_explicit) {
        if (glatter_windows_probe_egl_()) {
            requested = GLATTER_WSI_EGL_VALUE;
            wsi_explicit = 1;
        }
    }
#else
    if (!wsi_explicit) {
        if (glatter_posix_probe_glx_()) {
            requested = GLATTER_WSI_GLX_VALUE;
            wsi_explicit = 1;
        }
    }
    if (!wsi_explicit) {
        if (glatter_posix_probe_egl_()) {
            requested = GLATTER_WSI_EGL_VALUE;
            wsi_explicit = 1;
        }
    }
#endif
    GLATTER_ATOMIC_INT_STORE(state->requested, requested);
    GLATTER_ATOMIC_INT_STORE(state->wsi_explicit, wsi_explicit);
}

static void glatter_detect_wsi_from_env(glatter_loader_state* state)
{
    int wsi_explicit = GLATTER_ATOMIC_INT_LOAD(state->wsi_explicit);
    if (GLATTER_ATOMIC_INT_LOAD(state->env_checked) || wsi_explicit) {
        return;
    }
    GLATTER_ATOMIC_INT_STORE(state->env_checked, 1);

    const char* env = getenv("GLATTER_WSI");
    if (!env || !*env) {
        return;
    }

    int requested = GLATTER_ATOMIC_INT_LOAD(state->requested);
    if (glatter_equals_ignore_case(env, "wgl")) {
        requested = GLATTER_WSI_WGL_VALUE;
    }
    else
    if (glatter_equals_ignore_case(env, "glx")) {
        requested = GLATTER_WSI_GLX_VALUE;
    }
    else
    if (glatter_equals_ignore_case(env, "egl")) {
        requested = GLATTER_WSI_EGL_VALUE;
    }

    requested = glatter_normalize_requested_wsi_(requested);
    GLATTER_ATOMIC_INT_STORE(state->requested, requested);
}

GLATTER_INLINE_OR_NOT
void glatter_set_wsi(int wsi)
{
    glatter_loader_state* state = glatter_loader_state_get();
    glatter_wsi_t value = (glatter_wsi_t)wsi;
    switch (value) {
        case GLATTER_WSI_WGL_VALUE:
        case GLATTER_WSI_GLX_VALUE:
        case GLATTER_WSI_EGL_VALUE:
        case GLATTER_WSI_AUTO_VALUE:
            break;
        default:
            value = GLATTER_WSI_AUTO_VALUE;
            break;
    }
    int normalized = glatter_normalize_requested_wsi_(value);
    GLATTER_ATOMIC_INT_STORE(state->requested, normalized);
    GLATTER_ATOMIC_INT_STORE(state->wsi_explicit, 1);
    GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_AUTO_VALUE);
}

GLATTER_INLINE_OR_NOT
int glatter_get_wsi(void)
{
    glatter_loader_state* state = glatter_loader_state_get();
    glatter_detect_wsi_from_env(state);
    int active = GLATTER_ATOMIC_INT_LOAD(state->active);
    if (active != GLATTER_WSI_AUTO_VALUE) {
        return active;
    }
    return GLATTER_ATOMIC_INT_LOAD(state->requested);
}

GLATTER_INLINE_OR_NOT
void* glatter_get_proc_address(const char* function_name)
{
    glatter_loader_state* state = glatter_loader_state_get();
    glatter_detect_wsi_from_env(state);

    glatter_atomic_int* gate = glatter_wsi_gate_get();
    if (GLATTER_ATOMIC_INT_LOAD(*gate) != 2) {
        int expected = 0;
        if (GLATTER_ATOMIC_INT_CAS(*gate, expected, 1)) {
            glatter_decide_wsi_once_(state);
            GLATTER_ATOMIC_INT_STORE(*gate, 2);
        }
        else {
            while (GLATTER_ATOMIC_INT_LOAD(*gate) != 2) { /* spin */ }
        }
    }

    for (;;) {
        int requested = GLATTER_ATOMIC_INT_LOAD(state->requested);

        if (requested == GLATTER_WSI_DECIDING_VALUE) {
            while (GLATTER_ATOMIC_INT_LOAD(state->requested) == GLATTER_WSI_DECIDING_VALUE) { /* spin */ }
            continue;
        }

#if defined(_WIN32)
        if (requested == GLATTER_WSI_WGL_VALUE) {
            void* ptr = glatter_windows_resolve_wgl(state, function_name);
            if (ptr) GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_WGL_VALUE);
            return ptr;
        }
        if (requested == GLATTER_WSI_EGL_VALUE) {
            void* ptr = glatter_windows_resolve_egl(state, function_name);
            if (ptr) GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_EGL_VALUE);
            return ptr;
        }

        if (requested == GLATTER_WSI_AUTO_VALUE) {
            int expected = GLATTER_WSI_AUTO_VALUE;
            if (GLATTER_ATOMIC_INT_CAS(state->requested, expected, GLATTER_WSI_DECIDING_VALUE)) {
                void* ptr = glatter_windows_resolve_wgl(state, function_name);
                if (ptr) {
                    GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_WGL_VALUE);
                    (void)glatter_select_wsi_once_(state, GLATTER_WSI_WGL_VALUE);
                    return ptr;
                }

                ptr = glatter_windows_resolve_egl(state, function_name);
                if (ptr) {
                    GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_EGL_VALUE);
                    (void)glatter_select_wsi_once_(state, GLATTER_WSI_EGL_VALUE);
                    return ptr;
                }

                GLATTER_ATOMIC_INT_STORE(state->requested, GLATTER_WSI_AUTO_VALUE);
                return NULL;
            }

            continue;
        }
#else
        if (requested == GLATTER_WSI_GLX_VALUE) {
            void* ptr = glatter_linux_lookup_glx(state, function_name);
            if (ptr) GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_GLX_VALUE);
            return ptr;
        }
        if (requested == GLATTER_WSI_EGL_VALUE) {
            void* ptr = glatter_linux_lookup_egl(state, function_name);
            if (ptr) GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_EGL_VALUE);
            return ptr;
        }

        if (requested == GLATTER_WSI_AUTO_VALUE) {
            int expected = GLATTER_WSI_AUTO_VALUE;
            if (GLATTER_ATOMIC_INT_CAS(state->requested, expected, GLATTER_WSI_DECIDING_VALUE)) {
                void* ptr = glatter_linux_lookup_glx(state, function_name);
                if (ptr) {
                    GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_GLX_VALUE);
                    (void)glatter_select_wsi_once_(state, GLATTER_WSI_GLX_VALUE);
                    return ptr;
                }

                ptr = glatter_linux_lookup_egl(state, function_name);
                if (ptr) {
                    GLATTER_ATOMIC_INT_STORE(state->active, GLATTER_WSI_EGL_VALUE);
                    (void)glatter_select_wsi_once_(state, GLATTER_WSI_EGL_VALUE);
                    return ptr;
                }

                GLATTER_ATOMIC_INT_STORE(state->requested, GLATTER_WSI_AUTO_VALUE);
                return NULL;
            }

            continue;
        }
#endif

        return NULL;
    }
}

#if defined(GLATTER_GL)
GLATTER_INLINE_OR_NOT
const char* enum_to_string_GL(GLATTER_ENUM_GL e);

GLATTER_INLINE_OR_NOT
void glatter_check_error_GL(const char* file, int line)
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        glatter_log_printf(
            "GLATTER: in '%s'(%d):\n", file, line
        );

        glatter_log_printf(
            "GLATTER: OpenGL call produced %s error.\n", enum_to_string_GL(err)
        );
    }
}


GLATTER_INLINE_OR_NOT
void* glatter_get_proc_address_GL(const char* function_name)
{
    return glatter_get_proc_address(function_name);
}
#endif

#if defined(GLATTER_GLX)
GLATTER_INLINE_OR_NOT
const char* enum_to_string_GLX(GLATTER_ENUM_GLX e);

/* ---- GLX error accounting across threads (per-Display) ---- */

#if !defined(GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER)
static int (*glatter_prev_x_error_handler)(Display*, XErrorEvent*) = NULL;
#endif

typedef struct {
    glatter_atomic(void*) dpy;     /* slot key: Display* (atomically installed) */
    glatter_atomic_int    count;   /* monotonic error counter for that Display */
} glatter_glx_err_slot;

/* Keep it small: most apps have 1 display; 8 covers unusual cases. */
static glatter_glx_err_slot glatter_glx_err_table[8] = {
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
    { GLATTER_ATOMIC_INIT_PTR(NULL), GLATTER_ATOMIC_INT_INIT(0) },
};

GLATTER_INLINE_OR_NOT
glatter_glx_err_slot* glatter_glx_err_slot_for(Display* dpy)
{
    /* Linear probe with atomic install of the key. */
    for (int i = 0; i < (int)(sizeof(glatter_glx_err_table)/sizeof(glatter_glx_err_table[0])); ++i) {
        void* cur = GLATTER_ATOMIC_LOAD(glatter_glx_err_table[i].dpy);
        if (cur == (void*)dpy) return &glatter_glx_err_table[i];
        if (cur == NULL) {
            void* expected = NULL;
            if (GLATTER_ATOMIC_CAS(glatter_glx_err_table[i].dpy, expected, (void*)dpy)) {
                return &glatter_glx_err_table[i];
            }
            /* someone else installed; re-check */
            if (GLATTER_ATOMIC_LOAD(glatter_glx_err_table[i].dpy) == (void*)dpy) {
                return &glatter_glx_err_table[i];
            }
        }
    }
    /* Fallback: coalesce into slot 0 if table is exhausted. */
    return &glatter_glx_err_table[0];
}

GLATTER_INLINE_OR_NOT
void glatter_glx_err_increment(Display* dpy)
{
    glatter_glx_err_slot* s = glatter_glx_err_slot_for(dpy);
    int oldv = GLATTER_ATOMIC_INT_LOAD(s->count);
    for (;;) {
        int newv = oldv + 1;
        if (GLATTER_ATOMIC_INT_CAS(s->count, oldv, newv)) break;
        oldv = GLATTER_ATOMIC_INT_LOAD(s->count);
    }
}

#if !defined(GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER)
GLATTER_INLINE_OR_NOT
int x_error_handler(Display *dsp, XErrorEvent *error)
{
    int glx_opcode = 0;
    int first_event = 0;
    int first_error = 0;
    if (XQueryExtension(dsp, "GLX", &glx_opcode, &first_event, &first_error) &&
        error->request_code == glx_opcode) {
        char error_string[128];
        XGetErrorText(dsp, error->error_code, error_string, (int)sizeof(error_string));
        glatter_log_printf(
            "GLATTER: GLX X Error: %s\n", error_string
        );

        /* count this error for the specific Display*;
           works no matter which thread is currently reading X. */
        glatter_glx_err_increment(dsp);
    }

    if (glatter_prev_x_error_handler) {
        return glatter_prev_x_error_handler(dsp, error);
    }
    return 0;
}
#endif

GLATTER_INLINE_OR_NOT
void* glatter_get_proc_address_GLX(const char* function_name)
{
    void* ptr = glatter_get_proc_address(function_name);

#if !defined(GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER)
    if (ptr) {
        glatter_loader_state* state = glatter_loader_state_get();
        if (GLATTER_ATOMIC_INT_LOAD(state->active) == GLATTER_WSI_GLX_VALUE) {
            int expected = 0;
            if (GLATTER_ATOMIC_INT_CAS(state->glx_error_handler_installed, expected, 1)) {
                glatter_prev_x_error_handler = XSetErrorHandler(x_error_handler);
                glatter_log("GLATTER: installed cooperative X error handler (define GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER to disable).\n");
            }
        }
    }
#endif

    return ptr;
}

GLATTER_INLINE_OR_NOT
void glatter_check_error_GLX(const char* file, int line)
{
#if defined(GLATTER_LOG_ERRORS)
    Display* dpy = glXGetCurrentDisplay();
    if (dpy) {
        /* Snapshot -> force processing -> compare */
        glatter_glx_err_slot* s = glatter_glx_err_slot_for(dpy);
        int before = GLATTER_ATOMIC_INT_LOAD(s->count);
        XSync(dpy, False);
        int after  = GLATTER_ATOMIC_INT_LOAD(s->count);
        if (after != before) {
            glatter_log_printf(
                "GLATTER: GLX error detected after call at '%s'(%d); see prior X error log for details.\n",
                file,
                line
            );
        }
    }
    else {
        (void)file; (void)line;
    }
#else
    (void)file; (void)line;
#endif
}
#endif

#if defined(GLATTER_WGL)
GLATTER_INLINE_OR_NOT
const char* enum_to_string_WGL(GLATTER_ENUM_WGL e);

GLATTER_INLINE_OR_NOT
void glatter_check_error_WGL(const char* file, int line)
{
    DWORD eid = GetLastError();
    if(eid == 0)
        return;

    LPVOID buffer = NULL;
    FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
        eid, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL);

    glatter_log_printf(
        "GLATTER: LastError after WGL call (may be stale) in %s(%d):\n%s\t", file, line, (char*)buffer
    );

    LocalFree(buffer);
}


GLATTER_INLINE_OR_NOT
void* glatter_get_proc_address_WGL(const char* function_name)
{
    return glatter_get_proc_address(function_name);
}
#endif

#if defined(GLATTER_EGL)
GLATTER_INLINE_OR_NOT
const char* enum_to_string_EGL(GLATTER_ENUM_EGL e);


GLATTER_INLINE_OR_NOT
void glatter_check_error_EGL(const char* file, int line)
{
    EGLint err = eglGetError();
    if (err != EGL_SUCCESS) {
        glatter_log_printf(
            "GLATTER: EGL call produced %s error in %s(%d)\n", enum_to_string_EGL(err), file, line
        );
    }
}


GLATTER_INLINE_OR_NOT
void* glatter_get_proc_address_EGL(const char* function_name)
{
    return glatter_get_proc_address(function_name);
}
#endif

#if defined(GLATTER_GLU)
GLATTER_INLINE_OR_NOT
const char* enum_to_string_GLU(GLATTER_ENUM_GLU e);


GLATTER_INLINE_OR_NOT
void glatter_check_error_GLU(const char* file, int line)
{
    glatter_check_error_GL(file, line);
}


#if defined(__linux__)
static pthread_once_t* glatter_glu_once_slot(void)
{
    static pthread_once_t once = PTHREAD_ONCE_INIT;
    return &once;
}

static void** glatter_glu_handle_slot(void)
{
    static void* handle = NULL;
    return &handle;
}

static void glatter_open_glu_handle(void)
{
    void** handle = glatter_glu_handle_slot();
    *handle = dlopen("libGLU.so", RTLD_LAZY | RTLD_LOCAL);
    if (*handle == NULL) {
        *handle = dlopen("libGLU.so.1", RTLD_LAZY | RTLD_LOCAL);
    }
}
#endif

GLATTER_INLINE_OR_NOT
void* glatter_get_proc_address_GLU(const char* function_name)
{
    void* ptr = NULL;

#if defined(__linux__)
    pthread_once(glatter_glu_once_slot(), glatter_open_glu_handle);
    void* handle = *glatter_glu_handle_slot();
    ptr = handle ? dlsym(handle, function_name) : NULL;
#elif defined(_WIN32)
    HMODULE module = GetModuleHandleA("glu32.dll");
    if (module == NULL) {
        module = glatter_load_system32_dll_(L"glu32.dll");
    }
    ptr = module ? (void*)GetProcAddress(module, function_name) : NULL;
#elif defined(__APPLE__)
    // On macOS, GLU is part of the OpenGL framework, which is already loaded.
    // RTLD_DEFAULT allows searching in the global symbol scope.
    ptr = dlsym(RTLD_DEFAULT, function_name);
#else
    #error There is no implementation for your platform. Your contribution would be greatly appreciated!
#endif

    return ptr;
}
#endif

GLATTER_EXTERN_C_END

#if defined(GLATTER_HEADER_ONLY) && defined(__cplusplus)
namespace glatter { namespace detail {

#if defined(_WIN32)
    using thread_id_t = DWORD;
    inline thread_id_t current_thread_id() { return GetCurrentThreadId(); }
    inline bool thread_ids_equal(thread_id_t a, thread_id_t b) { return a == b; }
#elif defined(__APPLE__) || defined(__unix__) || defined(__unix)
    using thread_id_t = pthread_t;
    inline thread_id_t current_thread_id() { return pthread_self(); }
    inline bool thread_ids_equal(thread_id_t a, thread_id_t b) { return pthread_equal(a, b) != 0; }
#else
    #error "Unsupported platform"
#endif

    inline thread_id_t& owner_thread_id()
    {
#if defined(GLATTER_REQUIRE_EXPLICIT_OWNER_BIND)
        static thread_id_t tid = thread_id_t();
#else
        static thread_id_t tid = current_thread_id();
#endif
        return tid;
    }

#if defined(GLATTER_REQUIRE_EXPLICIT_OWNER_BIND)
    inline bool& owner_thread_bound()
    {
        static bool bound = false;
        return bound;
    }
#endif

    inline bool is_owner_thread()
    {
#if defined(GLATTER_REQUIRE_EXPLICIT_OWNER_BIND)
        if (!owner_thread_bound()) {
            ::glatter_log("GLATTER: owner thread not bound. Call glatter_bind_owner_to_current_thread() on the intended render thread.\n");
            abort();
        }
#endif
        return thread_ids_equal(current_thread_id(), owner_thread_id());
    }

    inline void bind_owner_to_current_thread()
    {
        owner_thread_id() = current_thread_id();
#if defined(GLATTER_REQUIRE_EXPLICIT_OWNER_BIND)
        owner_thread_bound() = true;
#endif
    }

}} // namespace glatter::detail


#else  // !header-only C++

GLATTER_EXTERN_C_BEGIN

#if defined(_WIN32)
    extern INIT_ONCE glatter_thread_once;
    extern DWORD     glatter_thread_id;
    extern glatter_atomic_int glatter_owner_bound_explicitly;
    extern glatter_atomic_int glatter_owner_thread_initialized;

    static BOOL CALLBACK glatter_init_owner_once_win(PINIT_ONCE once, PVOID param, PVOID* context)
    {
        (void)once; (void)param; (void)context;
        if (!GLATTER_ATOMIC_INT_LOAD(glatter_owner_thread_initialized)) {
            glatter_thread_id = GetCurrentThreadId(); /* explicit or implicit: first caller wins */
            GLATTER_ATOMIC_INT_STORE(glatter_owner_thread_initialized, 1);
        }
        return TRUE;
    }
#elif defined(__APPLE__) || defined(__unix__) || defined(__unix)
    extern pthread_once_t glatter_thread_once;
    extern pthread_t      glatter_thread_id;
    extern glatter_atomic_int glatter_owner_bound_explicitly;
    extern glatter_atomic_int glatter_owner_thread_initialized;

    static void glatter_init_owner_once_posix(void)
    {
        if (!GLATTER_ATOMIC_INT_LOAD(glatter_owner_thread_initialized)) {
            glatter_thread_id = pthread_self(); /* explicit or implicit: first caller wins */
            GLATTER_ATOMIC_INT_STORE(glatter_owner_thread_initialized, 1);
        }
    }
#else
    #error "Unsupported platform"
#endif

GLATTER_EXTERN_C_END

#endif // header-only switch

// Re-enter C linkage for the remainder of the C API.
GLATTER_EXTERN_C_BEGIN

GLATTER_INLINE_OR_NOT
void glatter_pre_callback(const char* file, int line)
{
#if defined(GLATTER_HEADER_ONLY) && defined(__cplusplus)
    if (!glatter::detail::is_owner_thread()) {
        static GLATTER_THREAD_LOCAL int glatter_warned_cross_thread = 0;
        if (!glatter_warned_cross_thread) {
            glatter_warned_cross_thread = 1;
            char* m = glatter_masprintf("GLATTER: Calling OpenGL from a different thread, in %s(%d)\n", file, line);
            glatter_log(m);
            free(m);
        }
    }

#elif defined(_WIN32)
    InitOnceExecuteOnce(&glatter_thread_once, glatter_init_owner_once_win, NULL, NULL);
    if (GetCurrentThreadId() != glatter_thread_id) {
        static GLATTER_THREAD_LOCAL int glatter_warned_cross_thread = 0;
        if (!glatter_warned_cross_thread) {
            glatter_warned_cross_thread = 1;
            char* m = glatter_masprintf("GLATTER: Calling OpenGL from a different thread, in %s(%d)\n", file, line);
            glatter_log(m);
            free(m);
        }
    }

#elif defined(__APPLE__) || defined(__unix__) || defined(__unix)
    pthread_once(&glatter_thread_once, glatter_init_owner_once_posix);
    if (!pthread_equal(pthread_self(), glatter_thread_id)) {
        static GLATTER_THREAD_LOCAL int glatter_warned_cross_thread = 0;
        if (!glatter_warned_cross_thread) {
            glatter_warned_cross_thread = 1;
            char* m = glatter_masprintf("GLATTER: Calling OpenGL from a different thread, in %s(%d)\n", file, line);
            glatter_log(m);
            free(m);
        }
    }
#else
    #error "Unsupported platform"
#endif
}


GLATTER_INLINE_OR_NOT
void glatter_bind_owner_to_current_thread(void)
{
#if defined(GLATTER_HEADER_ONLY) && defined(__cplusplus)
    glatter::detail::bind_owner_to_current_thread();
#elif defined(_WIN32)
    GLATTER_ATOMIC_INT_STORE(glatter_owner_bound_explicitly, 1);
    InitOnceExecuteOnce(&glatter_thread_once, glatter_init_owner_once_win, NULL, NULL);
#elif defined(__APPLE__) || defined(__unix__) || defined(__unix)
    GLATTER_ATOMIC_INT_STORE(glatter_owner_bound_explicitly, 1);
    pthread_once(&glatter_thread_once, glatter_init_owner_once_posix);
#endif
}

/* Format up to 16 bytes of storage as a printable hex string.  The
 * logging helpers pass the resulting pointer outside this helper, so we
 * maintain a small TLS ring buffer to keep the string alive long enough
 * for the log handler to consume it without dangling. */
GLATTER_INLINE_OR_NOT
const char* get_prs(size_t sz, const void* obj)
{
    enum { GLATTER_PRS_SLOTS = 8, GLATTER_PRS_CAP = 3*16 + 4 };
    static GLATTER_THREAD_LOCAL char glatter_prs_buf[GLATTER_PRS_SLOTS][GLATTER_PRS_CAP];
    static GLATTER_THREAD_LOCAL unsigned glatter_prs_index;

    char* out = glatter_prs_buf[glatter_prs_index++ % GLATTER_PRS_SLOTS];
    size_t cap = GLATTER_PRS_CAP;
    size_t pos = 0;

    if (cap == 0) {
        return "";
    }

    out[pos++] = '[';

    const unsigned char* bytes = (const unsigned char*)obj;
    if (sz > 16) {
        sz = 16;
    }

    if (sz > 0 && pos < cap) {
        int n = snprintf(out + pos, cap - pos, "%02x", bytes[0]);
        if (n < 0) {
            n = 0;
        }
        if ((size_t)n >= cap - pos) {
            pos = cap - 1;
        }
        else {
            pos += (size_t)n;
            for (size_t i = 1; i < sz && pos < cap; ++i) {
                n = snprintf(out + pos, cap - pos, " %02x", bytes[i]);
                if (n < 0) {
                    break;
                }
                if ((size_t)n >= cap - pos) {
                    pos = cap - 1;
                    break;
                }
                pos += (size_t)n;
            }
        }
    }

    if (pos < cap - 1) {
        out[pos++] = ']';
    }
    else if (cap >= 2) {
        out[cap - 2] = ']';
        pos = cap - 1;
    }

    out[pos] = '\0';
    return out;
}



GLATTER_INLINE_OR_NOT
uint32_t glatter_djb2(const uint8_t *str)
{
    uint32_t hash = 5381;
    for (int c = *str; c; c = *++str)
        hash = ((hash << 5) + hash) + c;
    return hash;
}


typedef struct glatter_es_record_struct
{
    uint32_t hash;
    uint16_t index;
} glatter_es_record_t;


//==================

#if !defined(NDEBUG) && !defined(GLATTER_RESOLVE_RETURNS_ZERO)
#   define GLATTER_RESOLVE_ABORT_ON_MISSING 1
#else
#   define GLATTER_RESOLVE_ABORT_ON_MISSING 0
#endif

/* Missing-symbol policy: in debug, abort to surface configuration errors early. */

#define GLATTER_RETURN_VALUE(return_or_not, rtype, value) \
    GLATTER_RETURN_VALUE_##return_or_not(rtype, value)

#define GLATTER_RETURN_VALUE_return(rtype, value) return (value)
#define GLATTER_RETURN_VALUE_(rtype, value)       return

/* Note: header-only vs TU variants differ only in storage/linkage; call flow is identical. */
#ifdef GLATTER_HEADER_ONLY

/* Thread-safe first use:
 * Resolution uses a single atomic CAS on the function pointer.
 */
#define GLATTER_FBLOCK(return_or_not, family, cder, rtype, cconv, name, cargs, dargs)\
    typedef rtype (cconv *glatter_##name##_t) dargs;\
    static glatter_atomic(glatter_##name##_t) glatter_##name##_resolved = GLATTER_ATOMIC_INIT_PTR(0);\
    static inline rtype cconv glatter_##name dargs\
    {\
        glatter_##name##_t fn = (glatter_##name##_t)GLATTER_ATOMIC_LOAD(glatter_##name##_resolved);\
        if (!fn) {\
            glatter_##name##_t resolved = (glatter_##name##_t)glatter_get_proc_address_##family(#name);\
            if (!resolved) {\
                if (GLATTER_RESOLVE_ABORT_ON_MISSING) {\
                    glatter_log_printf("GLATTER: missing '%s' (aborting in debug)\n", #name);\
                    abort();\
                }\
                glatter_log_printf("GLATTER: failed to resolve '%s'\n", #name);\
                GLATTER_RETURN_VALUE(return_or_not, rtype, (rtype)0);\
            }\
            glatter_##name##_t expected = (glatter_##name##_t)0;\
            if (!GLATTER_ATOMIC_CAS(glatter_##name##_resolved, expected, resolved)) {\
                /* another thread won the race, use its result */\
            }\
            fn = (glatter_##name##_t)GLATTER_ATOMIC_LOAD(glatter_##name##_resolved);\
        }\
        return_or_not fn cargs;\
    }

#else /* !GLATTER_HEADER_ONLY */

#if defined(_WIN32)

#  define GLATTER_FBLOCK(return_or_not, family, cder, rtype, cconv, name, cargs, dargs) \
    cder rtype cconv name dargs; \
    typedef rtype (cconv *glatter_##name##_t) dargs; \
    static rtype cconv glatter_##name##_resolver dargs; \
    glatter_##name##_t glatter_##name = glatter_##name##_resolver; \
    static rtype cconv glatter_##name##_resolver dargs \
    { \
        glatter_##name##_t resolved = (glatter_##name##_t)glatter_get_proc_address_##family(#name); \
        if (!resolved) { \
            if (GLATTER_RESOLVE_ABORT_ON_MISSING) { \
                glatter_log_printf("GLATTER: missing '%s' (aborting in debug)\n", #name); \
                abort(); \
            } \
            glatter_log_printf("GLATTER: failed to resolve '%s'\n", #name); \
            GLATTER_RETURN_VALUE(return_or_not, rtype, (rtype)0); \
        } \
        (void)InterlockedCompareExchangePointer((volatile PVOID*)&glatter_##name, (PVOID)resolved, (PVOID)glatter_##name##_resolver); \
        return_or_not glatter_##name cargs; \
    }

#else  /* POSIX: wrapper + call_once, no mutation of public pointer */

#  define GLATTER_FBLOCK(return_or_not, family, cder, rtype, cconv, name, cargs, dargs) \
    cder rtype cconv name dargs; /* keep symbol available for debuggers if needed */ \
    typedef rtype (cconv *glatter_##name##_t) dargs; \
    static glatter_once_t        glatter_##name##_once = GLATTER_ONCE_INIT; \
    static glatter_##name##_t glatter_##name##_impl = (glatter_##name##_t)0; \
    static void glatter_##name##_init(void) { \
        glatter_##name##_impl = (glatter_##name##_t)glatter_get_proc_address_##family(#name); \
        if (!glatter_##name##_impl) { \
            if (GLATTER_RESOLVE_ABORT_ON_MISSING) { \
                glatter_log_printf("GLATTER: missing '%s' (aborting in debug)\n", #name); \
                abort(); \
            } \
            glatter_log_printf("GLATTER: failed to resolve '%s'\n", #name); \
        } \
    } \
    static rtype cconv glatter_##name##_thunk dargs { \
        glatter_call_once(&glatter_##name##_once, glatter_##name##_init); \
        glatter_##name##_t fn = glatter_##name##_impl; \
        if (!fn) { GLATTER_RETURN_VALUE(return_or_not, rtype, (rtype)0); } \
        return_or_not fn cargs; \
    } \
    /* Public variable keeps ABI, points permanently to the thunk (never mutated). */ \
    glatter_##name##_t glatter_##name = glatter_##name##_thunk;

#endif

#endif /* GLATTER_HEADER_ONLY */

//==================

#include <stdarg.h>
#include <stdio.h>

/* Internal: format varargs and route to glatter_log(...) */
GLATTER_INLINE_OR_NOT
void glatter_vlog_line_(const char* prefix, const char* fmt, va_list ap)
{
    char buf[1024];
    /* Diagnostic logging: truncation is acceptable; vsnprintf prevents overflow. */
    int n = vsnprintf(buf, sizeof(buf), fmt ? fmt : "", ap);
    (void)n; /* truncation ignored intentionally */
    if (prefix && *prefix) glatter_log(prefix);
    glatter_log(buf);
}

GLATTER_INLINE_OR_NOT
void glatter_dbg_enter(const char* file, int line, const char* apiname,
                       const char* fmt, ...)
{
    glatter_pre_callback(file, line);
    glatter_log_printf("GLATTER: in '%s'(%d):\n", file, line);
    glatter_log_printf("GLATTER: %s", apiname);

    if (fmt && *fmt) {
        va_list ap; va_start(ap, fmt);
        glatter_vlog_line_("", fmt, ap);
        va_end(ap);
    }

    glatter_log("\n");
}

GLATTER_INLINE_OR_NOT
void glatter_dbg_return(const char* fmt, ...)
{
    glatter_log("GLATTER: returned ");
    if (fmt && *fmt) {
        va_list ap; va_start(ap, fmt);
        glatter_vlog_line_("", fmt, ap);
        va_end(ap);
    }
    else {
        glatter_log("(void)\n");
    }
}

#if defined(GLATTER_LOG_CALLS)

    /* Debug macros become thin function calls for debugger-friendly stepping */
    #define GLATTER_DBLOCK(file,line,name,printf_fmt,...) \
        glatter_dbg_enter((file),(line), #name, (printf_fmt), ##__VA_ARGS__)
    #define GLATTER_RBLOCK(...) \
        glatter_dbg_return(__VA_ARGS__)
#else
    #define GLATTER_DBLOCK(file, line, name, printf_fmt, ...) \
        glatter_pre_callback(file, line);
    #define GLATTER_RBLOCK(...)
#endif

#if defined (GLATTER_LOG_ERRORS)
    #define GLATTER_CHECK_ERROR(family, file, line) glatter_check_error_##family(file, line);
#else
    #define GLATTER_CHECK_ERROR(...)
#endif

#define GET_PRS(v)\
    (get_prs(sizeof(v), (const void*)(&(v))))


#if defined(__llvm__) || defined (__clang__)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wunused-value"
#elif defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wsizeof-array-argument"
#endif


#ifndef GLATTER_HEADER_ONLY
    #define GLATTER_str(s) #s
    #define GLATTER_xstr(s) GLATTER_str(s)
    #define GLATTER_PDIR(pd) platforms/pd
#endif


#if defined(GLATTER_LOG_ERRORS) || defined(GLATTER_LOG_CALLS)
    #if defined(GLATTER_GL)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GL_d_def.h)
    #endif
    #if defined(GLATTER_GLX)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLX_d_def.h)
    #endif
    #if defined(GLATTER_EGL) && GLATTER_HAS_EGL_GENERATED_HEADERS
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_EGL_d_def.h)
    #endif
    #if defined(GLATTER_WGL)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_WGL_d_def.h)
    #endif
    #if defined(GLATTER_GLU)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLU_d_def.h)
    #endif
#else
    #if defined(GLATTER_GL)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GL_r_def.h)
    #endif
    #if defined(GLATTER_GLX)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLX_r_def.h)
    #endif
    #if defined(GLATTER_EGL) && GLATTER_HAS_EGL_GENERATED_HEADERS
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_EGL_r_def.h)
    #endif
    #if defined(GLATTER_WGL)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_WGL_r_def.h)
    #endif
    #if defined(GLATTER_GLU)
        #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLU_r_def.h)
    #endif
#endif

#ifndef GLATTER_HEADER_ONLY
#if defined(GLATTER_GL)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GL_ges_decl.h)
#endif
#if defined(GLATTER_GLX)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLX_ges_decl.h)
#endif
#if defined(GLATTER_EGL) && GLATTER_HAS_EGL_GENERATED_HEADERS
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_EGL_ges_decl.h)
#endif
#if defined(GLATTER_WGL)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_WGL_ges_decl.h)
#endif
#endif

/** Returns a stable cache key for the *current thread's* GL context+display/DC.
 *  - 0 means no current context bound.
 *  - Key is process-local and not intended for persistence or IPC.
 *  - Safe on 32/64-bit; avoids out-of-range shifts; O(1).
 */
/* Unique-ish key for the *current* GL/WSI context, per platform.
   Safe on 32- and 64-bit builds (no UB shifts), cheap, and stable within a process. */
GLATTER_INLINE_OR_NOT uintptr_t glatter_current_gl_context_key_(void) {
    uintptr_t a = (uintptr_t)0, b = (uintptr_t)0;

#if defined(_WIN32)
    /* WGL: combine context and DC */
    a = (uintptr_t)wglGetCurrentContext();
    b = (uintptr_t)wglGetCurrentDC();
#elif defined(GLATTER_GLX)
    /* GLX: combine context and Display* */
    a = (uintptr_t)glXGetCurrentContext();
    b = (uintptr_t)glXGetCurrentDisplay();
#elif defined(GLATTER_EGL)
    /* EGL: combine context and Display* */
    a = (uintptr_t)eglGetCurrentContext();
    b = (uintptr_t)eglGetCurrentDisplay();
#else
    return (uintptr_t)0; /* Unknown WSI → treat as "no current context". */
#endif

    /* If both are null, no current context is bound. */
    if (((a | b) == (uintptr_t)0)) {
        return (uintptr_t)0;
    }

    /* Rotate/mix b by half the pointer width to avoid out-of-range shifts.
       HALF = (bits in uintptr_t) / 2  → 16 on 32-bit, 32 on 64-bit. */
    const unsigned HALF = (unsigned)(sizeof(uintptr_t) * 4);
    const unsigned BITS = (unsigned)(sizeof(uintptr_t) * 8);

    /* Rotations are defined because 0 < HALF < BITS on all sane targets. */
    uintptr_t brot = (uintptr_t)((b << HALF) | (b >> (BITS - HALF)));

    /* Final mix: cheap, symmetric, and stable enough for a cache key. */
    return a ^ brot;
}

#if defined(GLATTER_GL)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GL_ges_def.h)
#endif
#if defined(GLATTER_GLX)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLX_ges_def.h)
#endif
#if defined(GLATTER_EGL) && GLATTER_HAS_EGL_GENERATED_HEADERS
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_EGL_ges_def.h)
#endif
#if defined(GLATTER_WGL)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_WGL_ges_def.h)
#endif

/* Optional convenience to invalidate all families' caches available in this build. */
GLATTER_INLINE_OR_NOT void glatter_invalidate_all_extension_caches(void) {
#if defined(GLATTER_GL)
    glatter_invalidate_extension_cache_GL();
#endif
#if defined(GLATTER_GLX)
    glatter_invalidate_extension_cache_GLX();
#endif
#if defined(GLATTER_WGL)
    glatter_invalidate_extension_cache_WGL();
#endif
#if defined(GLATTER_EGL) && GLATTER_HAS_EGL_GENERATED_HEADERS
    glatter_invalidate_extension_cache_EGL();
#endif
}

#if defined(GLATTER_GL)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GL_e2s_def.h)
#endif
#if defined(GLATTER_GLX)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLX_e2s_def.h)
#endif
#if defined(GLATTER_EGL) && GLATTER_HAS_EGL_GENERATED_HEADERS
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_EGL_e2s_def.h)
#endif
#if defined(GLATTER_WGL)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_WGL_e2s_def.h)
#endif
#if defined(GLATTER_GLU)
    #include GLATTER_xstr(GLATTER_PDIR(GLATTER_PLATFORM_DIR)/glatter_GLU_e2s_def.h)
#endif

#if defined(GLATTER_EGL) && !GLATTER_HAS_EGL_GENERATED_HEADERS

GLATTER_INLINE_OR_NOT glatter_extension_support_status_EGL_t glatter_get_extension_support_EGL(void)
{
    glatter_extension_support_status_EGL_t status = {0};
    return status;
}

GLATTER_INLINE_OR_NOT const char* enum_to_string_EGL(GLATTER_ENUM_EGL e)
{
    (void)e;
    return "EGL_UNKNOWN";
}

#endif


#if defined(__llvm__) || defined (__clang__)
    #pragma clang diagnostic pop
#elif defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif


#if defined(_MSC_VER)
#  pragma warning(pop)
#endif

GLATTER_EXTERN_C_END
