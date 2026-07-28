#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include <EGL/egl.h>
#include <glatter/glatter.h>

#undef eglGetCurrentContext
#undef eglGetCurrentDisplay
#undef eglGetError
#undef glGetError

extern uintptr_t glatter_current_gl_context_key_(void);

static uintptr_t g_fake_context = (uintptr_t)0;
static uintptr_t g_fake_display = (uintptr_t)0;

EGLAPI EGLContext EGLAPIENTRY eglGetCurrentContext(void)
{
    return (EGLContext)(uintptr_t)g_fake_context;
}

EGLAPI EGLDisplay EGLAPIENTRY eglGetCurrentDisplay(void)
{
    return (EGLDisplay)(uintptr_t)g_fake_display;
}

EGLAPI EGLint EGLAPIENTRY eglGetError(void)
{
    return EGL_SUCCESS;
}

unsigned int glGetError(void)
{
    return 0u;
}

int main(void)
{
    const uintptr_t ctx_value = (uintptr_t)0x13572468u;
    const uintptr_t display_value = (uintptr_t)0xFDB97531u;

    g_fake_context = (uintptr_t)0;
    g_fake_display = (uintptr_t)0;

    if (glatter_current_gl_context_key_() != (uintptr_t)0) {
        fprintf(stderr, "expected zero key when no context\n");
        return 1;
    }

    g_fake_context = ctx_value;
    g_fake_display = display_value;

    const unsigned half = (unsigned)(sizeof(uintptr_t) * 4u);
    const unsigned bits = (unsigned)(sizeof(uintptr_t) * 8u);
    const uintptr_t expected_rotation =
        (uintptr_t)((display_value << half) | (display_value >> (bits - half)));
    const uintptr_t expected = ctx_value ^ expected_rotation;

    const uintptr_t observed = glatter_current_gl_context_key_();
    if (observed != expected) {
        fprintf(
            stderr,
            "expected key %#" PRIxPTR " but got %#" PRIxPTR "\n",
            expected,
            observed);
        return 2;
    }

    return 0;
}
