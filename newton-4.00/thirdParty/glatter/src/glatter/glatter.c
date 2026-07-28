/* Always include Windows/POSIX system headers BEFORE any GL headers. */
#include <glatter/glatter_config.h>

#ifdef GLATTER_HEADER_ONLY
#error "Do not compile glatter.c when GLATTER_HEADER_ONLY is defined."
#endif


/* Tag this TU as the compiled build; headers will use this for fail-fast checks. */
#ifndef GLATTER_SEPARATE_TU
#define GLATTER_SEPARATE_TU 1
#endif

#if defined(_MSC_VER)
#pragma detect_mismatch("glatter-build-mode", "separate")
#endif

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__APPLE__) || defined(__unix__)
#include <pthread.h>
#else
#error "Unsupported platform"
#endif

/* This will include platform headers in the correct, internal order. */
#include <glatter/glatter_def.h>
#include <glatter/glatter_atomic.h>

/* C/C++ compiled mode: globals are deliberate.
 * Header-only C++ uses function-local statics; compiled mode centralizes state
 * to one TU for consistent linkage and smaller object size.
 */
#if defined(_WIN32)
INIT_ONCE glatter_thread_once = INIT_ONCE_STATIC_INIT;
DWORD     glatter_thread_id   = 0;
/* Explicit INIT keeps intent clear; static zero-init would also work but is less obvious. */
glatter_atomic_int glatter_owner_bound_explicitly   = GLATTER_ATOMIC_INT_INIT(0);
glatter_atomic_int glatter_owner_thread_initialized = GLATTER_ATOMIC_INT_INIT(0);
#elif defined(__APPLE__) || defined(__unix__)
pthread_once_t glatter_thread_once = PTHREAD_ONCE_INIT;
pthread_t      glatter_thread_id;
/* Explicit INIT keeps intent clear; static zero-init would also work but is less obvious. */
glatter_atomic_int glatter_owner_bound_explicitly   = GLATTER_ATOMIC_INT_INIT(0);
glatter_atomic_int glatter_owner_thread_initialized = GLATTER_ATOMIC_INT_INIT(0);
#else
#error "Unsupported platform"
#endif