#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Provide a minimal configuration for the test. We skip the default
 * configuration header so we can compile without depending on any
 * platform-specific GL/EGL headers that may not be available.
 */
#define GLATTER_CONFIG_H_DEFINED

/*
 * Pretend the platform headers are already provided and supply the enum
 * aliases glatter_def.h expects. This keeps the test focused on the logging
 * helpers without pulling real GL headers.
 */
#define GLATTER_PLATFORM_HEADERS_H_DEFINED
typedef unsigned int GLATTER_ENUM_GL;
typedef unsigned int GLATTER_ENUM_GLX;
typedef unsigned int GLATTER_ENUM_WGL;
typedef unsigned int GLATTER_ENUM_GLU;
typedef int          GLATTER_ENUM_EGL;
char* glatter_masprintf(const char* format, ...);

#define GLATTER_MASPRINTF_H
#include <glatter/glatter_def.h>

pthread_once_t glatter_thread_once = PTHREAD_ONCE_INIT;
pthread_t      glatter_thread_id;
glatter_atomic_int            glatter_owner_bound_explicitly   = GLATTER_ATOMIC_INT_INIT(0);
glatter_atomic_int            glatter_owner_thread_initialized = GLATTER_ATOMIC_INT_INIT(0);

static char        g_last_log_buffer[1024];
static const char* g_last_log_message = NULL;

static void test_log_handler(const char* str)
{
    if (str == NULL) {
        g_last_log_message = NULL;
        return;
    }

    size_t len = strlen(str);
    if (len >= sizeof(g_last_log_buffer)) {
        len = sizeof(g_last_log_buffer) - 1;
    }

    memcpy(g_last_log_buffer, str, len);
    g_last_log_buffer[len] = '\0';
    g_last_log_message = g_last_log_buffer;
}

char* glatter_masprintf(const char* format, ...)
{
    (void)format;
    return NULL;
}

int main(void)
{
    glatter_set_log_handler(test_log_handler);

    const char* formatted = glatter_masprintf("glatter should fall back");
    const char* logged = glatter_log(formatted);

    if (formatted != NULL) {
        fprintf(stderr, "expected glatter_masprintf to fail\n");
        return 1;
    }

    if (logged != NULL) {
        fprintf(stderr, "glatter_log should return NULL when input is NULL\n");
        return 1;
    }

    if (g_last_log_message == NULL) {
        fprintf(stderr, "log handler was not invoked\n");
        return 1;
    }

    if (strcmp(g_last_log_message, "GLATTER: message formatting failed.\n") != 0) {
        fprintf(stderr, "unexpected fallback message: %s\n", g_last_log_message);
        return 1;
    }

    glatter_set_log_handler(NULL);

    if (glatter_log_handler() != glatter_default_log_handler) {
        fprintf(stderr, "NULL handler did not reset to default handler\n");
        return 1;
    }

    /* Smoke test to ensure the default handler can be invoked safely. */
    glatter_log("GLATTER: default handler smoke test.\n");

    glatter_set_log_handler(test_log_handler);
    glatter_log_printf("GLATTER: printf test %d\n", 42);

    if (strcmp(g_last_log_message, "GLATTER: printf test 42\n") != 0) {
        fprintf(stderr, "unexpected printf log message: %s\n", g_last_log_message);
        return 1;
    }

    return 0;
}
