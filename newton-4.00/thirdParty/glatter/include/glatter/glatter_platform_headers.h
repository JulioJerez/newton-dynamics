#ifndef GLATTER_PLATFORM_HEADERS_H_DEFINED
#define GLATTER_PLATFORM_HEADERS_H_DEFINED

/* Auto-select a platform bundle unless the user specifies one. */
#ifndef GLATTER_PLATFORM_DIR
# if defined(GLATTER_WINDOWS_EGL_GL)
#   define GLATTER_PLATFORM_DIR glatter_windows_egl_gl
# elif defined(GLATTER_WINDOWS_WGL_GL)
#   define GLATTER_PLATFORM_DIR glatter_windows_wgl_gl
# elif defined(GLATTER_MESA_EGL_GLES)
#   define GLATTER_PLATFORM_DIR glatter_mesa_egl_gles
# elif defined(GLATTER_MESA_GLX_GL)
#   define GLATTER_PLATFORM_DIR glatter_mesa_glx_gl
# elif defined(_WIN32)
#   define GLATTER_PLATFORM_DIR glatter_windows_wgl_gl
# elif defined(__ANDROID__) || defined(__EMSCRIPTEN__)
#   define GLATTER_PLATFORM_DIR glatter_mesa_egl_gles
# else
#   define GLATTER_PLATFORM_DIR glatter_mesa_glx_gl
# endif
#endif

// WARNING: This file is parsed by glatter.py. Keep the structure simple:
// - Each platform lives in a single #if / #elif block.
// - Inside that block there must be a line:
//     #define GLATTER_PLATFORM_DIR <platform_dir_name>
// - Put one #include per line, without trailing comments.

/* Ensure Mesa-supplied gl.h headers use our packaged extension headers. */
#if !defined(GL_GLEXT_LEGACY)
#define GL_GLEXT_LEGACY 1
#endif

/* -----------------------------------------------------------
   Platform selection
   One of the following platform macros must be defined in glatter_config.h:
     - GLATTER_WINDOWS_WGL_GL
     - GLATTER_WINDOWS_EGL_GL
     - GLATTER_MESA_GLX_GL
     - GLATTER_MESA_EGL_GLES

   When consumers bypass glatter_config.h (e.g., by defining
   GLATTER_CONFIG_H_DEFINED and providing configuration flags via the
   compiler command line) we still want the convenience macros that
   glatter_config.h would normally emit.  In particular the GLES
   profiles imply the use of the Mesa EGL + GLES platform.  Mirror that
   logic here so that the public headers can be used without pulling in
   glatter_config.h while still getting the correct platform selection.
   ----------------------------------------------------------- */

#if (defined(GLATTER_EGL_GLES_1_1) || defined(GLATTER_EGL_GLES2_2_0) || \
     defined(GLATTER_EGL_GLES_3_0) || defined(GLATTER_EGL_GLES_3_1)  || \
     defined(GLATTER_EGL_GLES_3_2)) && !defined(GLATTER_MESA_EGL_GLES)
#define GLATTER_MESA_EGL_GLES
#endif

#if !defined(GLATTER_HAS_EGL_GENERATED_HEADERS)
#if defined(__has_include)
#if __has_include("platforms/glatter_mesa_egl_gles/glatter_EGL_ges_decl.h") || \
    __has_include("glatter/platforms/glatter_mesa_egl_gles/glatter_EGL_ges_decl.h")
#define GLATTER_HAS_EGL_GENERATED_HEADERS 1
#else
#define GLATTER_HAS_EGL_GENERATED_HEADERS 0
#endif
#else
#define GLATTER_HAS_EGL_GENERATED_HEADERS 0
#endif
#endif

#if defined(GLATTER_WINDOWS_WGL_GL)

/* ---------------- Windows: WGL + desktop GL ---------------- */
#undef GLATTER_PLATFORM_DIR
#define GLATTER_PLATFORM_DIR glatter_windows_wgl_gl

/* Windows macros (WINGDIAPI, APIENTRY) come from <windows.h> */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

/* Core desktop GL for Windows */
#include "headers/windows_gl_basic/GL.h"

/* Khronos extension headers for GL and WGL */
#include "headers/khronos_gl/glext.h"
#include "headers/khronos_gl/wglext.h"

/* Optional GLU */
#if defined(GLATTER_GLU)
#include "headers/windows_gl_basic/GLU.h"
#endif

/* Sanity: no incompatible wrappers with this platform */
#if defined(GLATTER_GLX) || defined(GLATTER_EGL)
#error One of the wrappers defined is not relevant to the selected platform. Please review your glatter_config.h.
#endif

#elif defined(GLATTER_WINDOWS_EGL_GL)

/* ---------------- Windows: EGL + desktop GL ---------------- */
#undef GLATTER_PLATFORM_DIR
#define GLATTER_PLATFORM_DIR glatter_windows_egl_gl

/* Windows macros (WINGDIAPI, APIENTRY) come from <windows.h> */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

/* EGL core + extensions */
#include "headers/khronos_egl/egl.h"
#include "headers/khronos_egl/eglext.h"

/* Core desktop GL for Windows */
#include "headers/windows_gl_basic/GL.h"

/* Khronos extension headers for GL */
#include "headers/khronos_gl/glext.h"

/* Optional GLU */
#if defined(GLATTER_GLU)
#include "headers/windows_gl_basic/GLU.h"
#endif

/* Sanity: no incompatible wrappers with this platform */
#if defined(GLATTER_GLX) || defined(GLATTER_WGL)
#error One of the wrappers defined is not relevant to the selected platform. Please review your glatter_config.h.
#endif

#elif defined(GLATTER_MESA_GLX_GL)

/* ---------------- Linux/BSD: GLX + desktop GL -------------- */
#undef GLATTER_PLATFORM_DIR
#define GLATTER_PLATFORM_DIR glatter_mesa_glx_gl

/* Core desktop GL for Mesa (paths corrected for your tree) */
#include "headers/mesa_gl_basic/gl.h"

/* Desktop GL extensions */
#include "headers/khronos_gl/glext.h"

/* GLX core + extensions */
#include "headers/mesa_gl_basic/glx.h"
#include "headers/khronos_gl/glxext.h"

/* Optional GLU (your GLU lives under sgi_glu) */
#if defined(GLATTER_GLU)
#include "headers/sgi_glu/glu.h"
#endif

/* Sanity: no incompatible wrappers with this platform */
#if defined(GLATTER_WGL) || defined(GLATTER_EGL)
#error One of the wrappers defined is not relevant to the selected platform. Please review your glatter_config.h.
#endif

#elif defined(GLATTER_MESA_EGL_GLES)

/* ---------------- Any: EGL + OpenGL ES --------------------- */
#undef GLATTER_PLATFORM_DIR
#define GLATTER_PLATFORM_DIR glatter_mesa_egl_gles

#if !GLATTER_HAS_EGL_GENERATED_HEADERS
typedef struct glatter_extension_support_status_EGL
{
    int dummy;
} glatter_extension_support_status_EGL_t;
#endif

/* EGL core + extensions */
#include "headers/khronos_egl/egl.h"
#include "headers/khronos_egl/eglext.h"

/* GLES core + extensions */
#if defined(GLATTER_EGL_GLES_1_1)
#include "headers/khronos_gles/gl.h"
#include "headers/khronos_gles/glext.h"
#include "headers/khronos_gles/glplatform.h"
#else
#include "headers/khronos_gles2/gl2.h"
#include "headers/khronos_gles2/gl2ext.h"
#endif

/* (Optional) GLES3 core headers if you enable them in glatter_config.h */
#if !defined(GLATTER_EGL_GLES_1_1) && (defined(GLATTER_EGL_GLES_3_0) || \
    defined(GLATTER_EGL_GLES_3_1) || defined(GLATTER_EGL_GLES_3_2))
#include "headers/khronos_gles3/gl3.h"
#endif
#if !defined(GLATTER_EGL_GLES_1_1) && (defined(GLATTER_EGL_GLES_3_1) || \
    defined(GLATTER_EGL_GLES_3_2))
#include "headers/khronos_gles3/gl31.h"
#endif
#if !defined(GLATTER_EGL_GLES_1_1) && defined(GLATTER_EGL_GLES_3_2)
#include "headers/khronos_gles3/gl32.h"
#endif

/* Sanity: no incompatible wrappers with this platform */
#if defined(GLATTER_WGL) || defined(GLATTER_GLX)
#error One of the wrappers defined is not relevant to the selected platform. Please review your glatter_config.h.
#endif

#else
#error No supported platform selected. Define one of GLATTER_WINDOWS_WGL_GL, GLATTER_WINDOWS_EGL_GL, GLATTER_MESA_GLX_GL, or GLATTER_MESA_EGL_GLES in glatter_config.h.
#endif /* platform selection */

/* -----------------------------------------------------------
   Printf format fallbacks for 64-bit integers
   glatter.h includes <inttypes.h> before this file. If PRId64
   (and friends) are still not defined (e.g., older MSVC),
   provide sane defaults here.
   ----------------------------------------------------------- */
#ifndef PRId64
  #ifdef _MSC_VER
    #define PRId64 "I64d"
    #define PRIu64 "I64u"
    #define PRIx64 "I64x"
  #else
    #define PRId64 "lld"
    #define PRIu64 "llu"
    #define PRIx64 "llx"
  #endif
#endif

/* MSVC note:
   "%Iu" is the correct legacy printf format for size_t on MSVC (both 32/64-bit).
   VS2015+ also accepts "%zu". If you only target VS2015+, you may define
   GLATTER_FMT_ZU as "zu" before including this header. */
#ifndef GLATTER_FMT_ZU
  #ifdef _MSC_VER
    #define GLATTER_FMT_ZU "Iu"
  #else
    #define GLATTER_FMT_ZU "zu"
  #endif
#endif

/* -----------------------------------------------------------
   Enum type aliases used by enum_to_string_* helpers
   These are always available, independent of platform branch.
   You can tighten them later (e.g., EGLint for EGL) and regenerate.
   ----------------------------------------------------------- */
#ifdef GLATTER_GL
typedef GLenum GLATTER_ENUM_GL;
#else
typedef unsigned int GLATTER_ENUM_GL;
#endif

typedef unsigned int GLATTER_ENUM_GLX;
typedef unsigned int GLATTER_ENUM_WGL;

#ifdef GLATTER_GLU
typedef GLUenum GLATTER_ENUM_GLU;
#else
typedef unsigned int GLATTER_ENUM_GLU;
#endif

#ifdef GLATTER_EGL
typedef EGLint GLATTER_ENUM_EGL;
#else
typedef int GLATTER_ENUM_EGL;
#endif

#endif /* GLATTER_PLATFORM_HEADERS_H_DEFINED */
