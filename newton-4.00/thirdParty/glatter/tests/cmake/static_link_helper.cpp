#include <glatter/glatter.h>

int helper()
{
    return glatter_get_proc_address("glGetString") != nullptr;
}
