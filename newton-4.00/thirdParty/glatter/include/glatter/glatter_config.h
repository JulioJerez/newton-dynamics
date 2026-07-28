#ifndef GLATTER_CONFIG_H_DEFINED
#define GLATTER_CONFIG_H_DEFINED

/* 0) Immutable choice constants */
#define GLATTER_PLATFORM_AUTO 0
#define GLATTER_PLATFORM_WGL  1
#define GLATTER_PLATFORM_GLX  2
#define GLATTER_PLATFORM_EGL  3

/* 1) User knobs (prefer local file if present) */
#if defined(__has_include)
# if __has_include("glatter_config_user.h")
#  include "glatter_config_user.h"
# else
#  include <glatter/glatter_config_user.h>
# endif
#else
# include <glatter/glatter_config_user.h>
#endif

/* 2) Internal derivations & defaults */
/* Sanitize user knobs: treat 0 as undefined for boolean flags */
#if defined(GLATTER_GL) && (GLATTER_GL == 0)
#  define GLATTER_CFG_USER_NO_GL 1
#  undef GLATTER_GL
#endif
#if defined(GLATTER_GLU) && (GLATTER_GLU == 0)
#  define GLATTER_CFG_USER_NO_GLU 1
#  undef GLATTER_GLU
#endif
#if defined(GLATTER_EGL) && (GLATTER_EGL == 0)
#  define GLATTER_CFG_USER_NO_EGL 1
#  undef GLATTER_EGL
#endif
#if defined(GLATTER_WGL) && (GLATTER_WGL == 0)
#  define GLATTER_CFG_USER_NO_WGL 1
#  undef GLATTER_WGL
#endif
#if defined(GLATTER_GLX) && (GLATTER_GLX == 0)
#  define GLATTER_CFG_USER_NO_GLX 1
#  undef GLATTER_GLX
#endif
#if defined(GLATTER_LOG_ERRORS) && (GLATTER_LOG_ERRORS == 0)
#  define GLATTER_CFG_USER_NO_LOG_ERRORS 1
#  undef GLATTER_LOG_ERRORS
#endif
#if defined(GLATTER_LOG_CALLS) && (GLATTER_LOG_CALLS == 0)
#  define GLATTER_CFG_USER_NO_LOG_CALLS 1
#  undef GLATTER_LOG_CALLS
#endif
#if defined(GLATTER_REQUIRE_EXPLICIT_OWNER_BIND) && (GLATTER_REQUIRE_EXPLICIT_OWNER_BIND == 0)
#  undef GLATTER_REQUIRE_EXPLICIT_OWNER_BIND
#endif

/* Threaded X11 logging: prefer explicit knob */
#if defined(GLATTER_INSTALL_X_ERROR_HANDLER)
#  if GLATTER_INSTALL_X_ERROR_HANDLER
#    undef GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER
#  else
#    define GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER 1
#  endif
#endif

/* GLES version selection maps to legacy defines */
#if defined(GLATTER_GLES_VERSION)
#  if GLATTER_GLES_VERSION == 11
#    define GLATTER_EGL_GLES_1_1 1
#  elif GLATTER_GLES_VERSION == 20
#    define GLATTER_EGL_GLES2_2_0 1
#  elif GLATTER_GLES_VERSION == 30
#    define GLATTER_EGL_GLES_3_0 1
#  elif GLATTER_GLES_VERSION == 31
#    define GLATTER_EGL_GLES_3_1 1
#  elif GLATTER_GLES_VERSION == 32
#    define GLATTER_EGL_GLES_3_2 1
#  endif
#endif

/* Platform selection maps immutable choice constants to legacy switches */
#ifndef GLATTER_PLATFORM
#  define GLATTER_PLATFORM GLATTER_PLATFORM_AUTO
#endif

#if GLATTER_PLATFORM == GLATTER_PLATFORM_WGL
#  define GLATTER_WINDOWS_WGL_GL 1
#elif GLATTER_PLATFORM == GLATTER_PLATFORM_GLX
#  define GLATTER_MESA_GLX_GL 1
#elif GLATTER_PLATFORM == GLATTER_PLATFORM_EGL
#  define GLATTER_MESA_EGL_GLES 1
#endif

/* Build-mode coherence */
/* If a TU declares itself 'separate TU' (compiled mode), reject header-only in it. */
#if defined(GLATTER_SEPARATE_TU) && defined(GLATTER_HEADER_ONLY)
#error "glatter: This translation unit is marked as compiled mode (GLATTER_SEPARATE_TU) but also enables header-only. Pick one."
#endif

/* =========================
   Zero-config defaults
   ========================= */

/* If the user didn't configure Glatter manually, pick sensible defaults. */
#if !defined(GLATTER_USER_CONFIGURED) || (GLATTER_USER_CONFIGURED == 0)

    /* Core GL wrappers are available unless explicitly disabled. */
#   if !defined(GLATTER_GL) && !defined(GLATTER_CFG_USER_NO_GL)
#       define GLATTER_GL 1
#   endif

    /* Platform defaults based on the host toolchain. */
#   if defined(_WIN32)
#       ifndef GLATTER_WINDOWS_WGL_GL
#           define GLATTER_WINDOWS_WGL_GL 1
#       endif
#       if !defined(GLATTER_WGL) && !defined(GLATTER_CFG_USER_NO_WGL)
#           define GLATTER_WGL 1
#       endif
#   elif defined(__ANDROID__) || defined(__EMSCRIPTEN__)
#       ifndef GLATTER_MESA_EGL_GLES
#           define GLATTER_MESA_EGL_GLES 1
#       endif
#       if !defined(GLATTER_EGL) && !defined(GLATTER_CFG_USER_NO_EGL)
#           define GLATTER_EGL 1
#       endif
        /* Default to the highest GLES profile shipped with the headers. */
#       ifndef GLATTER_EGL_GLES_3_2
#           define GLATTER_EGL_GLES_3_2 1
#       endif
#   else
#       ifndef GLATTER_MESA_GLX_GL
#           define GLATTER_MESA_GLX_GL 1
#       endif
#       if !defined(GLATTER_GLX) && !defined(GLATTER_CFG_USER_NO_GLX)
#           define GLATTER_GLX 1
#       endif
#   endif

#endif /* !defined(GLATTER_USER_CONFIGURED) || (GLATTER_USER_CONFIGURED == 0) */


#ifndef GLATTER_WSI_AUTO_VALUE
#define GLATTER_WSI_AUTO_VALUE 0
#define GLATTER_WSI_WGL_VALUE  1
#define GLATTER_WSI_GLX_VALUE  2
#define GLATTER_WSI_EGL_VALUE  3
#endif


/////////////////////////////////
// Explicit platform selection //
/////////////////////////////////
//
// NOTE: For GLES the platform must be specified explicitly.
//
// #define GLATTER_WINDOWS_WGL_GL
// #define GLATTER_WINDOWS_EGL_GL
// #define GLATTER_MESA_GLX_GL
// #define GLATTER_MESA_EGL_GLES
// #define GLATTER_EGL_GLES_1_1
// #define GLATTER_EGL_GLES2_2_0
// #define GLATTER_EGL_GLES_3_0
// #define GLATTER_EGL_GLES_3_1
// #define GLATTER_EGL_GLES_3_2

// If no platform is defined, it will be set according to the operating system.
/* Two-stage defaults:
 * Stage 1 normalizes user knobs, Stage 2 finalizes per-platform defaults.
 * Split for readability; behavior is deterministic.
 */
#if !defined(GLATTER_WINDOWS_WGL_GL) &&\
    !defined(GLATTER_WINDOWS_EGL_GL) &&\
    !defined(GLATTER_MESA_GLX_GL)    &&\
    !defined(GLATTER_MESA_EGL_GLES)  &&\
    !defined(GLATTER_EGL_GLES_1_1)   &&\
    !defined(GLATTER_EGL_GLES2_2_0)  &&\
    !defined(GLATTER_EGL_GLES_3_0)   &&\
    !defined(GLATTER_EGL_GLES_3_1)   &&\
    !defined(GLATTER_EGL_GLES_3_2)

#if defined(_WIN32)
#   define GLATTER_WINDOWS_WGL_GL 1
#elif defined(__linux__)
#   define GLATTER_MESA_GLX_GL 1
#endif

#endif


#if defined(GLATTER_EGL_GLES_1_1)  ||\
    defined(GLATTER_EGL_GLES2_2_0) ||\
    defined(GLATTER_EGL_GLES_3_0)  ||\
    defined(GLATTER_EGL_GLES_3_1)  ||\
    defined(GLATTER_EGL_GLES_3_2)

#   ifndef GLATTER_MESA_EGL_GLES
#       define GLATTER_MESA_EGL_GLES 1
#   endif

#endif


////////////////////////////////
// Explicit wrapper selection //
////////////////////////////////
// #define GLATTER_GL
// #define GLATTER_EGL
// #define GLATTER_WGL
// #define GLATTER_GLX
// #define GLATTER_GLU

// If no wrapper is defined, GL is set, and one of ELG/GLX/WGL depending on the platform
#if !defined(GLATTER_EGL) && !defined(GLATTER_WGL) && !defined(GLATTER_GLX) &&\
    !defined(GLATTER_GLU)

#   if !defined(GLATTER_GL) && !defined(GLATTER_MESA_EGL_GLES) && \
        !defined(GLATTER_CFG_USER_NO_GL)
#       define GLATTER_GL 1
#   endif
#   if defined(GLATTER_WINDOWS_WGL_GL) && !defined(GLATTER_CFG_USER_NO_WGL)
#       ifndef GLATTER_WGL
#           define GLATTER_WGL 1
#       endif
#   elif defined(GLATTER_WINDOWS_EGL_GL) && !defined(GLATTER_CFG_USER_NO_EGL)
#       ifndef GLATTER_EGL
#           define GLATTER_EGL 1
#       endif
#   elif defined (GLATTER_MESA_GLX_GL) && !defined(GLATTER_CFG_USER_NO_GLX)
#       ifndef GLATTER_GLX
#           define GLATTER_GLX 1
#       endif
#   elif defined(GLATTER_MESA_EGL_GLES) && !defined(GLATTER_CFG_USER_NO_EGL)
#       ifndef GLATTER_EGL
#           define GLATTER_EGL 1
#       endif
#   endif

    // #define GLATTER_GLU  // GLU is not enabled by default

#endif


//////////////////////////////////////
// Debugging functionality switches //
//////////////////////////////////////
// #define GLATTER_LOG_ERRORS
// #define GLATTER_LOG_CALLS

// Unless specified otherwise, GL errors will be logged in debug builds
#if !defined(GLATTER_LOG_ERRORS) && !defined(GLATTER_LOG_CALLS) && \
    !defined(GLATTER_CFG_USER_NO_LOG_ERRORS) && \
    !defined(GLATTER_CFG_USER_NO_LOG_CALLS)
#   ifndef NDEBUG
#       define GLATTER_LOG_ERRORS 1
#   endif
#endif


///////////////////////////////////////////////////
// X error handler switch (only relevant to GLX) //
///////////////////////////////////////////////////
// Installing the GLATTER X error handler replaces the process-wide Xlib
// handler. Define GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER to opt out and
// install your own (for example after calling XInitThreads()).
//#define GLATTER_DO_NOT_INSTALL_X_ERROR_HANDLER

////////////////////////////////////////////////////////
// Thread ownership enforcement (header-only C++) switch //
////////////////////////////////////////////////////////
// Define GLATTER_REQUIRE_EXPLICIT_OWNER_BIND to disable the automatic
// owner-thread binding performed the first time a wrapped call executes in
// header-only builds. When set, applications must call
// glatter_bind_owner_to_current_thread() on the intended render thread before
// making GL calls; otherwise the library aborts to signal the configuration
// error.
//#define GLATTER_REQUIRE_EXPLICIT_OWNER_BIND

/////////////////////////////////////
// Windows character encoding switch //
/////////////////////////////////////
// Define GLATTER_WINDOWS_MBCS to treat TCHAR as CHAR when generating bindings
// for non-UNICODE builds. The generator assumes UNICODE (TCHAR -> WCHAR) by
// default.
#if defined(_WIN32)
# if !defined(UNICODE) && !defined(_UNICODE)
#   ifndef GLATTER_WINDOWS_MBCS
#     define GLATTER_WINDOWS_MBCS 1
#   endif
# endif
#endif

/* 3) Validation checks */

#if defined(GLATTER_WGL) && GLATTER_WGL
  #if !defined(_WIN32)
    #error "GLATTER_WGL is enabled but this is not a Windows (_WIN32) build."
  #endif
#endif

#if defined(GLATTER_WINDOWS_EGL_GL)
  #if !defined(_WIN32)
    #error "GLATTER_WINDOWS_EGL_GL is enabled but this is not a Windows (_WIN32) build."
  #endif
#endif

#if defined(GLATTER_GLX) && GLATTER_GLX
  #if defined(_WIN32)
    #error "GLATTER_GLX is enabled but this is a Windows build."
  #endif
#endif

/* Ensure platform constants were not altered downstream */
#if (GLATTER_PLATFORM_AUTO != 0) || (GLATTER_PLATFORM_WGL != 1) || \
    (GLATTER_PLATFORM_GLX  != 2) || (GLATTER_PLATFORM_EGL != 3)
# error "Do not modify GLATTER_PLATFORM_* constants; set GLATTER_PLATFORM only."
#endif

#endif /* GLATTER_CONFIG_H_DEFINED */
