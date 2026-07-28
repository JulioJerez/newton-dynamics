#include <glatter/glatter.h>

static void noop_logger(const char* message)
{
    (void)message;
}

int main(void)
{
    glatter_set_log_handler(noop_logger);
    glatter_set_log_handler(NULL);
    return 0;
}
