#include <cstdio>
#include <cstdlib>

#define GLATTER_EGL_GLES_3_0
#define GLATTER_GL
#define GLATTER_EGL

#include "glatter/glatter.h"

#undef eglGetProcAddress

extern "C" __eglMustCastToProperFunctionPointerType eglGetProcAddress(const char*)
{
    return NULL;
}

int main()
{
    void* symbol = glatter_get_proc_address("glGetString");
    if (symbol == NULL) {
        std::fprintf(stderr, "glGetString not found\n");
        return EXIT_FAILURE;
    }

    typedef const unsigned char* (*glGetString_t)(unsigned int);
    glGetString_t glGetString_ptr = reinterpret_cast<glGetString_t>(symbol);
    const unsigned char* value = glGetString_ptr(0);

    if (value == NULL) {
        std::fprintf(stderr, "glGetString returned NULL\n");
        return EXIT_FAILURE;
    }

    std::puts(reinterpret_cast<const char*>(value));
    return EXIT_SUCCESS;
}
