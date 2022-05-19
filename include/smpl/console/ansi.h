#ifndef SMPL_ANSI_H
#define SMPL_ANSI_H

#include <string>
#include <ostream>

namespace smpl {
namespace console {

namespace codes {

extern const char* reset;
extern const char* bold;
extern const char* nobold;
extern const char* uline;
extern const char* nouline;
extern const char* xout;
extern const char* noxout;
extern const char* oline;
extern const char* nooline;
extern const char* sblink;
extern const char* fblink;
extern const char* noblink;
extern const char* neg;
extern const char* noneg;
extern const char* black;
extern const char* red;
extern const char* green;
extern const char* yellow;
extern const char* blue;
extern const char* magenta;
extern const char* cyan;
extern const char* white;

} // namespace codes

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
reset(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
bold(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
nobold(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
uline(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
nouline(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
xout(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
noxout(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
oline(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
nooline(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
sblink(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
fblink(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
noblink(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
neg(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
noneg(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
black(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
red(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
green(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
yellow(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
blue(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
magenta(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
cyan(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
white(std::basic_ostream<CharT, Traits>& o);

struct color_type {
    uint8_t r, g, b;
    bool index;
    color_type(uint8_t index) : r(index), g(0), b(0), index(true) { }
    color_type(uint8_t r, uint8_t g, uint8_t b) :
        r(r), g(g), b(b), index(false)
    { }
};
template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits>& o, const color_type& c);

color_type color(uint8_t index);
color_type color(uint8_t r, uint8_t g, uint8_t b);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
nocolor(std::basic_ostream<CharT, Traits>& o);

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits>&
rainbow(std::basic_ostream<CharT, Traits>& o);

} // namespace console
} // namespace smpl

#include "detail/ansi.hpp"

#endif
