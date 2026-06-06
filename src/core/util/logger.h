#pragma once
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>

namespace mmbp::log {

enum Level { DEBUG = 0, INFO = 1, WARN = 2, ERR = 3 };

using LogCallback = void (*)(int level, const char *message, void *userData);

struct Logger {
  std::atomic<Level> minLevel{WARN};
  LogCallback callback = nullptr;
  void *userData = nullptr;
  std::mutex cbMutex;

  void log(Level level, const char *fmt, ...) {
    if (level < minLevel.load(std::memory_order_relaxed))
      return;
    char buf[2048];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n < 0)
      return;
    if (n >= (int)sizeof(buf)) {
      const char trunc[] = "...[truncated]";
      size_t tlen = sizeof(trunc) - 1;
      if (sizeof(buf) > tlen + 1) {
        std::memcpy(buf + sizeof(buf) - 1 - tlen, trunc, tlen);
        buf[sizeof(buf) - 1] = '\0';
      }
    }
    LogCallback cb;
    void *ud;
    {
      std::lock_guard<std::mutex> lock(cbMutex);
      cb = callback;
      ud = userData;
    }
    if (cb)
      cb(static_cast<int>(level), buf, ud);
  }

#define MMBP_LOG_METHOD(NAME, LVL)                                             \
  void NAME(const char *fmt, ...) {                                            \
    if (LVL < minLevel.load(std::memory_order_relaxed))                        \
      return;                                                                  \
    char buf[2048];                                                            \
    va_list args;                                                              \
    va_start(args, fmt);                                                       \
    int n = vsnprintf(buf, sizeof(buf), fmt, args);                            \
    va_end(args);                                                              \
    if (n < 0)                                                                 \
      return;                                                                  \
    if (n >= (int)sizeof(buf)) {                                               \
      const char t[] = "...[truncated]";                                       \
      size_t tl = sizeof(t) - 1;                                               \
      if (sizeof(buf) > tl + 1) {                                              \
        std::memcpy(buf + sizeof(buf) - 1 - tl, t, tl);                        \
        buf[sizeof(buf) - 1] = '\0';                                           \
      }                                                                        \
    }                                                                          \
    LogCallback cb;                                                            \
    void *ud;                                                                  \
    {                                                                          \
      std::lock_guard<std::mutex> lock(cbMutex);                               \
      cb = callback;                                                           \
      ud = userData;                                                           \
    }                                                                          \
    if (cb)                                                                    \
      cb(static_cast<int>(LVL), buf, ud);                                      \
  }

  MMBP_LOG_METHOD(debug, DEBUG)
  MMBP_LOG_METHOD(info, INFO)
  MMBP_LOG_METHOD(warn, WARN)
  MMBP_LOG_METHOD(error, ERR)

#undef MMBP_LOG_METHOD
};

inline Logger &getLogger() {
  static Logger instance;
  return instance;
}

inline void setLogCallback(LogCallback cb, void *userData) {
  auto &g = getLogger();
  std::lock_guard<std::mutex> lock(g.cbMutex);
  g.callback = cb;
  g.userData = userData;
}

inline void setLogLevel(Level level) {
  getLogger().minLevel.store(level, std::memory_order_relaxed);
}

#define g_logger (mmbp::log::getLogger())

} // namespace mmbp::log
