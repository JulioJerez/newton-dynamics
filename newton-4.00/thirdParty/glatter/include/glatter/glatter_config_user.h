#ifndef GLATTER_CONFIG_USER_H
#define GLATTER_CONFIG_USER_H

/* ===== User knobs only. No logic here. ===== */

/* APIs */
#define GLATTER_GL                           1
#define GLATTER_GLU                          0

/* GLES: one of {0, 11, 20, 30, 31, 32}. 0 disables GLES. */
#define GLATTER_GLES_VERSION                 0

/* Platform/WSI selection (use constants defined in glatter_config.h) */
#ifndef GLATTER_PLATFORM
#  define GLATTER_PLATFORM                   GLATTER_PLATFORM_AUTO
#endif

/* Logging */
#define GLATTER_LOG_ERRORS                   1
#define GLATTER_LOG_CALLS                    0

/* Thread ownership */
#define GLATTER_REQUIRE_EXPLICIT_OWNER_BIND  0

/* X11 */
#define GLATTER_INSTALL_X_ERROR_HANDLER      1

/* Optional: override the number of per-thread extension-cache slots.
   Raise only if your app rapidly swaps among >8 contexts on the same thread. */
#ifndef GLATTER_ES_CACHE_SLOTS
#define GLATTER_ES_CACHE_SLOTS               8
#endif

#endif /* GLATTER_CONFIG_USER_H */
