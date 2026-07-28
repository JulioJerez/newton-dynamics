#define GLATTER_EGL_GLES2_2_0
#define GLATTER_EGL
#define GLATTER_str(s) #s
#define GLATTER_xstr(s) GLATTER_str(s)
#define GLATTER_PDIR(pd) platforms/pd
#include <glatter/platforms/glatter_egl_gles2_2_0/glatter_EGL_ges_decl.h>
#include <glatter/glatter_def.h>

/*
 * Invoked on the worker thread to trigger the cached-thread warning in
 * glatter_pre_callback().
 */
void glatter_thread_warning_on_worker(void)
{
    glatter_pre_callback(__FILE__, __LINE__);
}
