#include <stddef.h>

#include <glatter/glatter.h>

static void noop(const char* message)
{
    (void)message;
}

int main(void)
{
    glatter_set_log_handler(noop);
    glatter_set_log_handler(NULL);
    return glatter_get_wsi();
}
