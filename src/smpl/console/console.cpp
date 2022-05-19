#include <smpl/console/detail/console_std.h>

// standard includes
#include <stdarg.h>
#include <string.h>
#include <iostream>
#include <mutex>
#include <unordered_map>

#include <boost/program_options.hpp>

// project includes
#include <smpl/console/ansi.h>
#include <smpl/console/nonstd.h>

namespace smpl {
namespace console {

bool g_initialized = false;

static bool g_unbuffered = false;
static bool g_colored = false;
static bool g_show_locations = false;

static std::mutex g_init_mutex;
static std::mutex g_locations_mutex;

// map (fully-qualified logger name) -> (logger)
static std::unordered_map<std::string, Logger> g_loggers;

Logger* GetLogger(const std::string& name)
{
    auto lit = g_loggers.find(name);
    if (lit == end(g_loggers)) {
        bool inserted;
        std::tie(lit, inserted) = g_loggers.insert(std::make_pair(name, Logger()));

        // find or create the parent logger
        Logger* parent;
        std::string::size_type pos = name.size();
        auto dotpos = name.find_last_of('.', pos);
        if (dotpos == std::string::npos) {
            parent = &g_loggers[""];
        } else {
            parent = GetLogger(name.substr(0, dotpos));
        }

        lit->second.parent = parent;
        lit->second.level = parent->level;
    }

    return &lit->second;
}

void initialize()
{
    std::unique_lock<std::mutex> lock(g_init_mutex);

    if (g_initialized) {
        return;
    }

    // create the root logger
    g_loggers[""] = Logger{ nullptr, LEVEL_INFO };

    const char* config_filepath = getenv("SMPL_CONSOLE_CONFIG_FILE");
    if (!config_filepath) {
        g_initialized = true;
        return;
    }

    // parse the config file
    namespace po = boost::program_options;

    po::options_description ops;
    ops.add_options()
            ("format.unbuffered", po::value<bool>(&g_unbuffered)->default_value(false))
            ("format.colored", po::value<bool>(&g_colored)->default_value(false))
            ("format.show_locations", po::value<bool>(&g_show_locations)->default_value(false))
            ;

    bool allow_unregistered = true;
    auto pops = po::parse_config_file<char>(
            config_filepath, ops, allow_unregistered);

    po::variables_map vm;
    po::store(pops, vm);
    po::notify(vm);

    for (auto& op : pops.options) {
        if (op.unregistered) {
            auto& levelstr = op.value.back();
            Level level = LEVEL_COUNT;
            if (levelstr == "INFO") {
                level = LEVEL_INFO;
            } else if (levelstr == "DEBUG") {
                level = LEVEL_DEBUG;
            } else if (levelstr == "WARN") {
                level = LEVEL_WARN;
            } else if (levelstr == "ERROR") {
                level = LEVEL_ERROR;
            } else if (levelstr == "FATAL") {
                level = LEVEL_FATAL;
            }

            if (level != LEVEL_COUNT) { // format correct
                Logger* logger = GetLogger(op.string_key);
                logger->level = level;
            }
        }
    }

    g_initialized = true;
}

void InitializeLogLocation(
    LogLocation* loc,
    const std::string& name,
    Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);

    if (loc->initialized) {
        return;
    }

    loc->logger = GetLogger(name);
    loc->level = level;
    loc->enabled = level >= loc->logger->level;
    loc->initialized = true;
}

void print(Level level, const char* filename, int line, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    FILE* f;
    if (level >= LEVEL_ERROR) {
        f = stderr;
    } else {
        f = stdout;
    }

    if (g_colored) {
        switch (level) {
        case LEVEL_DEBUG:
            fprintf(f, "%s", codes::green);
            break;
        case LEVEL_INFO:
            fprintf(f, "%s", codes::white);
            break;
        case LEVEL_WARN:
            fprintf(f, "%s", codes::yellow);
            break;
        case LEVEL_ERROR:
            fprintf(f, "%s", codes::red);
            break;
        case LEVEL_FATAL:
            fprintf(f, "%s", codes::red);
            break;
        default:
            break;
        }
    }

    switch (level) {
    case LEVEL_DEBUG:
        fprintf(f, "[DEBUG] ");
        break;
    case LEVEL_INFO:
        fprintf(f, "[INFO]  ");
        break;
    case LEVEL_WARN:
        fprintf(f, "[WARN]  ");
        break;
    case LEVEL_ERROR:
        fprintf(f, "[ERROR] ");
        break;
    case LEVEL_FATAL:
        fprintf(f, "[FATAL] ");
        break;
    default:
        break;
    }

    vfprintf(f, fmt, args);

    // print file and line
    if (g_show_locations) {
        const char* base = strrchr(filename, '\\');
        if (base) {
            fprintf(f, " [%s:%d]", base + 1, line);
        } else {
            fprintf(f, " [%s:%d]", filename, line);
        }
    }

    if (g_colored) {
        fprintf(f, "%s", codes::reset);
    }

    fprintf(f, "\n");

    if (g_unbuffered) {
        fflush(f);
    }

    va_end(args);
}

void print(Level level, const char* filename, int line, const std::stringstream& ss)
{
    auto& o = (level >= LEVEL_ERROR) ? std::cerr : std::cout;

    if (g_colored) {
        switch (level) {
        case LEVEL_DEBUG:
            o << green;
            break;
        case LEVEL_INFO:
            o << white;
            break;
        case LEVEL_WARN:
            o << yellow;
            break;
        case LEVEL_ERROR:
            o << red;
            break;
        case LEVEL_FATAL:
            o << red;
            break;
        default:
            break;
        }
    }

    switch (level) {
    case LEVEL_DEBUG:
        o << "[DEBUG] ";
        break;
    case LEVEL_INFO:
        o << "[INFO]  ";
        break;
    case LEVEL_WARN:
        o << "[WARN]  ";
        break;
    case LEVEL_ERROR:
        o << "[ERROR] ";
        break;
    case LEVEL_FATAL:
        o << "[FATAL] ";
        break;
    default:
        break;
    }

    o << ss.str();

    if (g_show_locations) {
        o << " [" << filename << ':' << line << ']';
    }

    if (g_colored) {
        o << reset;
    }

    o << '\n';
    if (g_unbuffered) {
        o << std::flush;
    }
}

} // namespace console
} // namespace smpl
