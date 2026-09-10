#include "snowclear/logging.hpp"

#include <cstdio>
#include <mutex>

namespace snowclear {
namespace {

std::mutex& sink_mutex() {
    static std::mutex m;
    return m;
}

LogSink& sink() {
    static LogSink s;
    return s;
}

// Default sink: mirrors the previous ros1_compat behaviour (Info/Debug to
// stdout, Warn/Error to stderr, flushed per line) so that output captured by
// the experiment tooling is byte-identical to before.
void default_sink(LogLevel level, const std::string& message) {
    const char* name = "INFO";
    FILE* stream = stdout;
    switch (level) {
        case LogLevel::kDebug: name = "DEBUG"; break;
        case LogLevel::kInfo:  name = "INFO";  break;
        case LogLevel::kWarn:  name = "WARN";  stream = stderr; break;
        case LogLevel::kError: name = "ERROR"; stream = stderr; break;
    }
    std::fprintf(stream, "[%s] %s\n", name, message.c_str());
    std::fflush(stream);
}

}  // namespace

void set_log_sink(LogSink new_sink) {
    std::lock_guard<std::mutex> lock(sink_mutex());
    sink() = std::move(new_sink);
}

void log_message(LogLevel level, const char* fmt, ...) {
    char stack_buffer[1024];
    va_list args;
    va_start(args, fmt);
    const int needed = std::vsnprintf(stack_buffer, sizeof(stack_buffer), fmt, args);
    va_end(args);

    std::string message;
    if (needed < 0) {
        message = "<log formatting failed>";
    } else if (static_cast<size_t>(needed) < sizeof(stack_buffer)) {
        message.assign(stack_buffer, static_cast<size_t>(needed));
    } else {
        message.resize(static_cast<size_t>(needed));
        va_start(args, fmt);
        std::vsnprintf(&message[0], static_cast<size_t>(needed) + 1, fmt, args);
        va_end(args);
    }

    LogSink copy;
    {
        std::lock_guard<std::mutex> lock(sink_mutex());
        copy = sink();
    }
    if (copy) {
        copy(level, message);
    } else {
        default_sink(level, message);
    }
}

}  // namespace snowclear
