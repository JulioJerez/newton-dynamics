#ifndef GLATTER_MASPRINTF_H
#define GLATTER_MASPRINTF_H


#include <stdio.h>  /* needed for vsnprintf    */
#include <stdlib.h> /* needed for malloc, free */
#include <stdarg.h> /* needed for va_*         */

/*
 * Configure inline keyword usage
 */
#ifndef GLATTER_MASPRINTF_INLINE
# ifdef GLATTER_INLINE_OR_NOT
#  define GLATTER_MASPRINTF_INLINE static GLATTER_INLINE_OR_NOT
# elif defined(_MSC_VER)
#  define GLATTER_MASPRINTF_INLINE static __inline
# elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
#  define GLATTER_MASPRINTF_INLINE static inline
# else
#  define GLATTER_MASPRINTF_INLINE static
# endif
#endif

/* Note: One definition per toolchain, marked GLATTER_MASPRINTF_INLINE (static inline),
 * so no ODR/multiple-definition issues across TUs or header-only builds.
 */

/* vscprintf:
 * MSVC provides _vscprintf; others use the C99 vsnprintf(NULL, 0, ...).
 * Exactly one definition is included per toolchain, and it is inline.
 */
#ifdef _MSC_VER
#  define vscprintf _vscprintf
#else
GLATTER_MASPRINTF_INLINE int vscprintf(const char* format, va_list ap)
{
    va_list ap_copy;
    va_copy(ap_copy, ap);
    int n = vsnprintf(NULL, 0, format, ap_copy);
    va_end(ap_copy);
    return n;
}
#endif

GLATTER_MASPRINTF_INLINE char* glatter_mvasprintf(const char* format, va_list ap);
GLATTER_MASPRINTF_INLINE char* glatter_masprintf(const char* format, ...);



GLATTER_MASPRINTF_INLINE char* glatter_mvasprintf(const char* format, va_list ap)
{
    va_list ap_len;
    va_copy(ap_len, ap);
    int len = vscprintf(format, ap_len);
    va_end(ap_len);
    if (len < 0)
        return 0;

    char* str = (char*)malloc((size_t)len + 1);
    if (!str)
        return 0;

    va_list ap_copy;
    va_copy(ap_copy, ap);
    int val = vsnprintf(str, (size_t)len + 1, format, ap_copy);
    va_end(ap_copy);
    if (val < 0) {
        free(str);
        return 0;
    }
    return str;
}


GLATTER_MASPRINTF_INLINE char* glatter_masprintf(const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    char* retval = glatter_mvasprintf(format, ap);
    va_end(ap);
    return retval;
}

#endif // GLATTER_MASPRINTF_H
