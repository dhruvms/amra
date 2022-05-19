#ifndef SMPL_DETAIL_ANSI_H
#define SMPL_DETAIL_ANSI_H

#include <smpl/console/ansi.h>

namespace smpl {
namespace console {

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
reset(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[0m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
bold(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[1m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
nobold(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[22m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
uline(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[4m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
nouline(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[24m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
xout(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[9m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
noxout(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[29m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
oline(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[53m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
nooline(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[55m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
sblink(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[5m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
fblink(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[6m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
noblink(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[25m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
neg(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[7m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
noneg(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[27m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
black(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[30m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
red(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[31m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
green(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[32m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
yellow(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[33m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
blue(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[34m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
magenta(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[35m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
cyan(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[36m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
white(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[37m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits>& o, const color_type& c)
{
    if (c.index) {
        o << "\x1B[38;5;" << std::to_string((unsigned)c.r) << 'm';
    }
    else {
        o << "\x1B[38;2;" << std::to_string((unsigned)c.r) << ';' <<
                std::to_string((unsigned)c.g) << ';' <<
                std::to_string((unsigned)c.b) << 'm';
    }
    return o;
}

inline color_type color(uint8_t index)
{
    return color_type(index);
}

inline color_type color(uint8_t r, uint8_t g, uint8_t b)
{
    return color_type(r, g, b);
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
nocolor(std::basic_ostream<CharT, Traits>& o)
{
    o << "\x1B[39m";
    return o;
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
rainbow(std::basic_ostream<CharT, Traits>& o)
{
    o << red << "r" << green << "a" << yellow << "i" << blue << "n" <<
            magenta << "b" << cyan << "o" << white << "w" << nocolor;
    return o;
}

} // namespace console
} // namespace smpl

#endif
