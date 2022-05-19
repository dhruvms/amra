#ifndef SMPL_CONSOLE_STD_H
#define SMPL_CONSOLE_STD_H

// standard includes
#include <chrono>
#include <sstream>
#include <string>

// project includes
#include <smpl/time.h>

namespace smpl {
namespace console {

enum Level {
    LEVEL_DEBUG = 0,
    LEVEL_INFO,
    LEVEL_WARN,
    LEVEL_ERROR,
    LEVEL_FATAL,

    LEVEL_COUNT
};

struct Logger {
    Logger* parent;
    Level level;
};

struct LogLocation {
    Logger* logger;
    LogLocation* next;
    ::smpl::console::Level level;
    bool enabled;
    bool initialized;
};

void InitializeLogLocation(
    LogLocation* loc,
    const std::string& name,
    Level level);

#ifndef SMPL_CONSOLE_ROS
  #define SMPL_ROOT_CONSOLE_NAME "smpl"
  #ifdef SMPL_PACKAGE_NAME
    #define SMPL_CONSOLE_NAME_PREFIX "." SMPL_PACKAGE_NAME
  #else
    #define SMPL_CONSOLE_NAME_PREFIX SMPL_ROOT_CONSOLE_NAME
  #endif
#endif

void print(Level level, const char* filename, int line, const char* fmt, ...);
void print(Level level, const char* filename, int line, const std::stringstream& ss);

extern bool g_initialized;
void initialize();

} // namespace console
} // namespace smpl

#define SMPL_CONSOLE_INIT \
do { \
    if (!::smpl::console::g_initialized) { \
        ::smpl::console::initialize(); \
    } \
} while(0)

#define SMPL_LOG_DEFINE_LOCATION(cond_, level_, name_) \
    static ::smpl::console::LogLocation __sc_define_location__loc = { \
        nullptr, nullptr, ::smpl::console::LEVEL_COUNT, false, false \
    }; \
    if (!__sc_define_location__loc.initialized) { \
        InitializeLogLocation(&__sc_define_location__loc, name_, level_); \
    } \
    bool __sc_define_location__enabled = \
        __sc_define_location__loc.enabled && (cond_)

#define SMPL_LOG_COND(cond, level, name, fmt, ...) \
    SMPL_CONSOLE_INIT; \
    do { \
        SMPL_LOG_DEFINE_LOCATION(cond, level, name); \
        if (__sc_define_location__enabled) { \
            ::smpl::console::print(level, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while (0)

#define SMPL_LOG(level, name, fmt, ...) SMPL_LOG_COND(true, level, name, fmt, ##__VA_ARGS__)

#define SMPL_LOG_STREAM_COND(cond, level, name, args) \
    SMPL_CONSOLE_INIT; \
    do { \
        SMPL_LOG_DEFINE_LOCATION(cond, level, name); \
        if (__sc_define_location__enabled) { \
            std::stringstream _smpl_log_stream_ss_; \
            _smpl_log_stream_ss_ << args; \
            ::smpl::console::print(level, __FILE__, __LINE__, _smpl_log_stream_ss_); \
        } \
    } while (0)

#define SMPL_LOG_STREAM(level, name, args) SMPL_LOG_STREAM_COND(true, level, name, args)

#define SMPL_LOG_ONCE(level, name, fmt, ...) \
    do { \
        static bool hit = false; \
        if (!hit) { \
            hit = true; \
            ::smpl::console::print(level, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while (0)

#define SMPL_LOG_THROTTLE(level, name, fmt, ...) \
    do { \
        static ::smpl::clock::time_point last_hit; \
        static auto rate_dur = ::std::chrono::duration_cast<::smpl::clock::duration>( \
            ::std::chrono::duration<double>(1.0 / (double)rate)); \
        auto now = ::smpl::clock::now(); \
        if (last_hit + rate_dur <= now) { \
            last_hit = now; \
            ::smpl::console::print(level, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while (0)

    // named and not named
    // stream and not stream
    // default or (once|cond|throttle)

#if SMPL_LOG_LEVEL <= SMPL_LOG_LEVEL_DEBUG
  #define SMPL_DEBUG(fmt, ...)                              SMPL_LOG(::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_COND(cond, fmt, ...)                   SMPL_LOG_COND(cond, ::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_ONCE(fmt, ...)                         SMPL_LOG_ONCE(::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_THROTTLE(rate, fmt, ...)               SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_NAMED(name, fmt, ...)                  SMPL_LOG(::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_COND_NAMED(name, cond, fmt, ...)       SMPL_LOG_COND(cond, ::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_ONCE_NAMED(name, fmt, ...)             SMPL_LOG_ONCE(::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_THROTTLE_NAMED(name, rate, fmt, ...)   SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_DEBUG_STREAM(args)                           SMPL_LOG_STREAM(::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_DEBUG_STREAM_COND(args)                      SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_DEBUG_STREAM_ONCE(args)                      SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_DEBUG_STREAM_THROTTLE(args)                  SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_DEBUG, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_DEBUG_STREAM_NAMED(name, args)               SMPL_LOG_STREAM(::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_DEBUG_STREAM_COND_NAMED(name, args)          SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_DEBUG_STREAM_ONCE_NAMED(name, args)          SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_DEBUG_STREAM_THROTTLE_NAMED(name, args)      SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_DEBUG, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
#else
  #define SMPL_DEBUG(fmt, ...)
  #define SMPL_DEBUG_COND(cond, fmt, ...)
  #define SMPL_DEBUG_ONCE(fmt, ...)
  #define SMPL_DEBUG_THROTTLE(rate, fmt, ...)
  #define SMPL_DEBUG_NAMED(name, fmt, ...)
  #define SMPL_DEBUG_COND_NAMED(name, cond, fmt, ...)
  #define SMPL_DEBUG_ONCE_NAMED(name, fmt, ...)
  #define SMPL_DEBUG_THROTTLE_NAMED(name, rate, fmt, ...)
  #define SMPL_DEBUG_STREAM(args)
  #define SMPL_DEBUG_STREAM_COND(args)
  #define SMPL_DEBUG_STREAM_ONCE(args)
  #define SMPL_DEBUG_STREAM_THROTTLE(args)
  #define SMPL_DEBUG_STREAM_NAMED(name, args)
  #define SMPL_DEBUG_STREAM_COND_NAMED(name, args)
  #define SMPL_DEBUG_STREAM_ONCE_NAMED(name, args)
  #define SMPL_DEBUG_STREAM_THROTTLE_NAMED(name, args)
#endif

#if SMPL_LOG_LEVEL <= SMPL_LOG_LEVEL_INFO
  #define SMPL_INFO(fmt, ...)                              SMPL_LOG(::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_COND(cond, fmt, ...)                   SMPL_LOG_COND(cond, ::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_ONCE(fmt, ...)                         SMPL_LOG_ONCE(::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_THROTTLE(rate, fmt, ...)               SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_NAMED(name, fmt, ...)                  SMPL_LOG(::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_COND_NAMED(name, cond, fmt, ...)       SMPL_LOG_COND(cond, ::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_ONCE_NAMED(name, fmt, ...)             SMPL_LOG_ONCE(::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_THROTTLE_NAMED(name, rate, fmt, ...)   SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_INFO_STREAM(args)                           SMPL_LOG_STREAM(::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_INFO_STREAM_COND(args)                      SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_INFO_STREAM_ONCE(args)                      SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_INFO_STREAM_THROTTLE(args)                  SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_INFO, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_INFO_STREAM_NAMED(name, args)               SMPL_LOG_STREAM(::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_INFO_STREAM_COND_NAMED(name, args)          SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_INFO_STREAM_ONCE_NAMED(name, args)          SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_INFO_STREAM_THROTTLE_NAMED(name, args)      SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_INFO, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
#else
  #define SMPL_INFO(fmt, ...)
  #define SMPL_INFO_COND(cond, fmt, ...)
  #define SMPL_INFO_ONCE(fmt, ...)
  #define SMPL_INFO_THROTTLE(rate, fmt, ...)
  #define SMPL_INFO_NAMED(name, fmt, ...)
  #define SMPL_INFO_COND_NAMED(name, cond, fmt, ...)
  #define SMPL_INFO_ONCE_NAMED(name, fmt, ...)
  #define SMPL_INFO_THROTTLE_NAMED(name, rate, fmt, ...)
  #define SMPL_INFO_STREAM(args)
  #define SMPL_INFO_STREAM_COND(args)
  #define SMPL_INFO_STREAM_ONCE(args)
  #define SMPL_INFO_STREAM_THROTTLE(args)
  #define SMPL_INFO_STREAM_NAMED(name, args)
  #define SMPL_INFO_STREAM_COND_NAMED(name, args)
  #define SMPL_INFO_STREAM_ONCE_NAMED(name, args)
  #define SMPL_INFO_STREAM_THROTTLE_NAMED(name, args)
#endif

#if SMPL_LOG_LEVEL <= SMPL_LOG_LEVEL_WARN
  #define SMPL_WARN(fmt, ...)                              SMPL_LOG(::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_COND(cond, fmt, ...)                   SMPL_LOG_COND(cond, ::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_ONCE(fmt, ...)                         SMPL_LOG_ONCE(::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_THROTTLE(rate, fmt, ...)               SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_NAMED(name, fmt, ...)                  SMPL_LOG(::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_COND_NAMED(name, cond, fmt, ...)       SMPL_LOG_COND(cond, ::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_ONCE_NAMED(name, fmt, ...)             SMPL_LOG_ONCE(::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_THROTTLE_NAMED(name, rate, fmt, ...)   SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_WARN_STREAM(args)                           SMPL_LOG_STREAM(::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_WARN_STREAM_COND(args)                      SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_WARN_STREAM_ONCE(args)                      SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_WARN_STREAM_THROTTLE(args)                  SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_WARN, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_WARN_STREAM_NAMED(name, args)               SMPL_LOG_STREAM(::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_WARN_STREAM_COND_NAMED(name, args)          SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_WARN_STREAM_ONCE_NAMED(name, args)          SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_WARN_STREAM_THROTTLE_NAMED(name, args)      SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_WARN, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
#else
  #define SMPL_WARN(fmt, ...)
  #define SMPL_WARN_COND(cond, fmt, ...)
  #define SMPL_WARN_ONCE(fmt, ...)
  #define SMPL_WARN_THROTTLE(rate, fmt, ...)
  #define SMPL_WARN_NAMED(name, fmt, ...)
  #define SMPL_WARN_COND_NAMED(name, cond, fmt, ...)
  #define SMPL_WARN_ONCE_NAMED(name, fmt, ...)
  #define SMPL_WARN_THROTTLE_NAMED(name, rate, fmt, ...)
  #define SMPL_WARN_STREAM(args)
  #define SMPL_WARN_STREAM_COND(args)
  #define SMPL_WARN_STREAM_ONCE(args)
  #define SMPL_WARN_STREAM_THROTTLE(args)
  #define SMPL_WARN_STREAM_NAMED(name, args)
  #define SMPL_WARN_STREAM_COND_NAMED(name, args)
  #define SMPL_WARN_STREAM_ONCE_NAMED(name, args)
  #define SMPL_WARN_STREAM_THROTTLE_NAMED(name, args)
#endif

#if SMPL_LOG_LEVEL <= SMPL_LOG_LEVEL_ERROR
  #define SMPL_ERROR(fmt, ...)                              SMPL_LOG(::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_COND(cond, fmt, ...)                   SMPL_LOG_COND(cond, ::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_ONCE(fmt, ...)                         SMPL_LOG_ONCE(::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_THROTTLE(rate, fmt, ...)               SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_NAMED(name, fmt, ...)                  SMPL_LOG(::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_COND_NAMED(name, cond, fmt, ...)       SMPL_LOG_COND(cond, ::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_ONCE_NAMED(name, fmt, ...)             SMPL_LOG_ONCE(::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_THROTTLE_NAMED(name, rate, fmt, ...)   SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_ERROR_STREAM(args)                           SMPL_LOG_STREAM(::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_ERROR_STREAM_COND(args)                      SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_ERROR_STREAM_ONCE(args)                      SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_ERROR_STREAM_THROTTLE(args)                  SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_ERROR, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_ERROR_STREAM_NAMED(name, args)               SMPL_LOG_STREAM(::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_ERROR_STREAM_COND_NAMED(name, args)          SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_ERROR_STREAM_ONCE_NAMED(name, args)          SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_ERROR_STREAM_THROTTLE_NAMED(name, args)      SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_ERROR, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
#else
  #define SMPL_ERROR(fmt, ...)
  #define SMPL_ERROR_COND(cond, fmt, ...)
  #define SMPL_ERROR_ONCE(fmt, ...)
  #define SMPL_ERROR_THROTTLE(rate, fmt, ...)
  #define SMPL_ERROR_NAMED(name, fmt, ...)
  #define SMPL_ERROR_COND_NAMED(name, cond, fmt, ...)
  #define SMPL_ERROR_ONCE_NAMED(name, fmt, ...)
  #define SMPL_ERROR_THROTTLE_NAMED(name, rate, fmt, ...)
  #define SMPL_ERROR_STREAM(args)
  #define SMPL_ERROR_STREAM_COND(args)
  #define SMPL_ERROR_STREAM_ONCE(args)
  #define SMPL_ERROR_STREAM_THROTTLE(args)
  #define SMPL_ERROR_STREAM_NAMED(name, args)
  #define SMPL_ERROR_STREAM_COND_NAMED(name, args)
  #define SMPL_ERROR_STREAM_ONCE_NAMED(name, args)
  #define SMPL_ERROR_STREAM_THROTTLE_NAMED(name, args)
#endif

#if SMPL_LOG_LEVEL <= SMPL_LOG_LEVEL_FATAL
  #define SMPL_FATAL(fmt, ...)                              SMPL_LOG(::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_COND(cond, fmt, ...)                   SMPL_LOG_COND(cond, ::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_ONCE(fmt, ...)                         SMPL_LOG_ONCE(::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_THROTTLE(rate, fmt, ...)               SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_NAMED(name, fmt, ...)                  SMPL_LOG(::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_COND_NAMED(name, cond, fmt, ...)       SMPL_LOG_COND(cond, ::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_ONCE_NAMED(name, fmt, ...)             SMPL_LOG_ONCE(::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_THROTTLE_NAMED(name, rate, fmt, ...)   SMPL_LOG_THROTTLE(rate, ::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, fmt, ##__VA_ARGS__)
  #define SMPL_FATAL_STREAM(args)                           SMPL_LOG_STREAM(::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_FATAL_STREAM_COND(args)                      SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_FATAL_STREAM_ONCE(args)                      SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_FATAL_STREAM_THROTTLE(args)                  SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_FATAL, SMPL_CONSOLE_NAME_PREFIX, args)
  #define SMPL_FATAL_STREAM_NAMED(name, args)               SMPL_LOG_STREAM(::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_FATAL_STREAM_COND_NAMED(name, args)          SMPL_LOG_STREAM_COND(cond, ::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_FATAL_STREAM_ONCE_NAMED(name, args)          SMPL_LOG_STREAM_ONCE(::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
  #define SMPL_FATAL_STREAM_THROTTLE_NAMED(name, args)      SMPL_LOG_STREAM_THROTTLE(::smpl::console::LEVEL_FATAL, std::string(SMPL_CONSOLE_NAME_PREFIX) + "." + name, args)
#else
  #define SMPL_FATAL(fmt, ...)
  #define SMPL_FATAL_COND(cond, fmt, ...)
  #define SMPL_FATAL_ONCE(fmt, ...)
  #define SMPL_FATAL_THROTTLE(rate, fmt, ...)
  #define SMPL_FATAL_NAMED(name, fmt, ...)
  #define SMPL_FATAL_COND_NAMED(name, cond, fmt, ...)
  #define SMPL_FATAL_ONCE_NAMED(name, fmt, ...)
  #define SMPL_FATAL_THROTTLE_NAMED(name, rate, fmt, ...)
  #define SMPL_FATAL_STREAM(args)
  #define SMPL_FATAL_STREAM_COND(args)
  #define SMPL_FATAL_STREAM_ONCE(args)
  #define SMPL_FATAL_STREAM_THROTTLE(args)
  #define SMPL_FATAL_STREAM_NAMED(name, args)
  #define SMPL_FATAL_STREAM_COND_NAMED(name, args)
  #define SMPL_FATAL_STREAM_ONCE_NAMED(name, args)
  #define SMPL_FATAL_STREAM_THROTTLE_NAMED(name, args)
#endif

#endif
