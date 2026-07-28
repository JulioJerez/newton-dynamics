#include <glatter/glatter.h>

int helper();

static void noop_logger(const char*) {}

int main()
{
    glatter_set_log_handler(noop_logger);
    return helper();
}
