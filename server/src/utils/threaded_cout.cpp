#include "threaded_cout.hpp"

#include <debugapi.h>

std::mutex &get_cout_mutex()
{
    static std::mutex m;
    return m;
}

void real_print(const std::string &str)
{
    OutputDebugStringA(str.c_str());
}
