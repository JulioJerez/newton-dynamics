#include <glatter/glatter.h>
#include <stdio.h>

// This test verifies the primary purpose of glatter: loading a modern,
// dynamically-resolved OpenGL function.
//
// We use glGenBuffers because it is not an OpenGL 1.1
// function and must be resolved by the loader's getProcAddress mechanism.
//
// We only need this code to COMPILE and LINK successfully in the CI.
//
int main() {
    printf("Attempting to link against a modern OpenGL function (glGenBuffers)...\n");

    // These are the arguments needed to call glGenBuffers.
    // Their values are irrelevant; they just need to satisfy the compiler.
    GLuint buffer_id = 0;
    GLsizei n = 1;

    // This is the real test. If this line compiles and links, it proves
    // the glatter macro and the underlying function pointer resolution work.
    glGenBuffers(n, &buffer_id);

    printf("Glatter modern OpenGL call integration test built successfully.\n");

    return 0;
}
