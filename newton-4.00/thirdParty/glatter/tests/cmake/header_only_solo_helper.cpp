#include <glatter/glatter_solo.h>

int helper()
{
    return glatter_get_proc_address("glGetString") != nullptr;
}
