#ifndef THREADED_COUT_HPP
#define THREADED_COUT_HPP

#include <sstream>
#include <mutex>

inline std::ostream&
print_one(std::ostream& os)
{
    return os;
}

template <class A0, class ...Args>
inline std::ostream&
print_one(std::ostream& os, const A0& a0, const Args& ...args)
{
    os << a0;
    return print_one(os, args...);
}

template <class ...Args>
inline std::ostream&
print_all(std::ostream& os, const Args& ...args)
{
    return print_one(os, args...);
}

std::mutex&
get_cout_mutex();

void real_print(const std::string& str);

template <class ...Args>
void
print(const Args& ...args)
{
    std::lock_guard<std::mutex> _(get_cout_mutex());
    std::stringstream ss;
    auto &os = print_all(ss, args...);
    os << std::flush;
    real_print(ss.str());
}

#endif // THREADED_COUT_HPP
