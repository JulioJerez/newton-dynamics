#define GLATTER_EGL_GLES2_2_0
#define GLATTER_EGL
#define GLATTER_str(s) #s
#define GLATTER_xstr(s) GLATTER_str(s)
#define GLATTER_PDIR(pd) platforms/pd
#include <glatter/platforms/glatter_egl_gles2_2_0/glatter_EGL_ges_decl.h>
#include <glatter/glatter_def.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * Demonstrates that multiple translation units observe the same
 * glatter_pre_callback ownership state provided by the shared singleton.
 */

void glatter_thread_warning_on_worker(void);

static void* worker_start(void* arg)
{
    (void)arg;
    glatter_thread_warning_on_worker();
    return NULL;
}

int main(void)
{
    glatter_pre_callback(__FILE__, __LINE__);

    pthread_t worker;
    if (pthread_create(&worker, NULL, worker_start, NULL) != 0) {
        fprintf(stderr, "failed to create worker thread\n");
        return EXIT_FAILURE;
    }

    if (pthread_join(worker, NULL) != 0) {
        fprintf(stderr, "failed to join worker thread\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
