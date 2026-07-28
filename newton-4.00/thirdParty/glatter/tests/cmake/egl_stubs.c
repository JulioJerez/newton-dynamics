#include <EGL/egl.h>

__eglMustCastToProperFunctionPointerType eglGetProcAddress(const char* name)
{
    (void)name;
    return NULL;
}

EGLint eglGetError(void)
{
    return EGL_SUCCESS;
}

EGLDisplay eglGetCurrentDisplay(void)
{
    return EGL_NO_DISPLAY;
}

EGLContext eglGetCurrentContext(void)
{
    return EGL_NO_CONTEXT;
}
