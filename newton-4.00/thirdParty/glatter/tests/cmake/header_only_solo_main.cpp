#include <glatter/glatter_solo.h>

int helper();

static void noop_logger(const char*) {}

int main()
{
    glatter_set_log_handler(noop_logger);
    glatter_set_log_handler(nullptr);
    return helper();
}
