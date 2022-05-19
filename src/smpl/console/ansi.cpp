#include <smpl/console/ansi.h>

namespace smpl {
namespace console {
namespace codes {

const char* reset = "\x1B[0m";
const char* bold = "\x1B[1m";
const char* nobold = "\x1B[22m";
const char* uline = "\x1B[4m";
const char* nouline = "\x1B[24m";
const char* xout = "\x1B[9m";
const char* noxout = "\x1B[29m";
const char* oline = "\x1B[53m";
const char* nooline = "\x1B[55m";
const char* sblink = "\x1B[5m";
const char* fblink = "\x1B[6m";
const char* noblink = "\x1B[25m";
const char* neg = "\x1B[7m";
const char* noneg = "\x1B[27m";
const char* black = "\x1B[30m";
const char* red = "\x1B[31m";
const char* green = "\x1B[32m";
const char* yellow = "\x1B[33m";
const char* blue = "\x1B[34m";
const char* magenta = "\x1B[35m";
const char* cyan = "\x1B[36m";
const char* white = "\x1B[37m";

} // namespace codes
} // namespace console
} // namespace smpl
