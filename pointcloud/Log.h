//
// Created by nyx on 12/19/18.
//

#pragma once

#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

namespace std {
    template<typename T>
    auto operator<<(ostream& os, const std::vector<T>& vec) -> ostream&;
}    // namespace std

namespace rock::log_parser::log {
    static auto stream_flag = false;

    namespace impl {
        enum class Module : uint32_t { SLAM = 4, PERCEPTOR = 25 };

        enum class Level : uint32_t { Fatal = 2, Error = 3, Warning = 4, Info = 6, Debug = 7, Verbose = 8 };

        struct Log {
            using logger_type = int (*)(Module, Level, int, char const*, char const*, ...);
            using writer_type = int (*)(Module, char const*, ...);

            using level_type = Level;

            static logger_type logger;
            static writer_type writer;

            static level_type level;

            template<typename... Args>
            static void record(level_type level, char const* func, uint32_t line, char const* format, Args... args) {
                if (level > Log::level)
                    return;
                logger(Module::PERCEPTOR, level, line, func, format, args...);
            }

            template<typename... Args>
            static void record(char const* format, Args... args) {
                writer(Module::PERCEPTOR, format, args...);
            }

            Log() = delete;
        };
    }    // namespace impl

#define PARSER_FATAL(fmt, ...)                                                                                        \
    rock::log_parser::log::impl::Log::record(rock::log_parser::log::impl::Level::Fatal, __func__, __LINE__, fmt "\n", \
                                             ##__VA_ARGS__)
#define PARSER_ERROR(fmt, ...)                                                                                        \
    rock::log_parser::log::impl::Log::record(rock::log_parser::log::impl::Level::Error, __func__, __LINE__, fmt "\n", \
                                             ##__VA_ARGS__)

#define PARSER_WARN(fmt, ...)                                                                                 \
    rock::log_parser::log::impl::Log::record(rock::log_parser::log::impl::Level::Warning, __func__, __LINE__, \
                                             fmt "\n", ##__VA_ARGS__)

#define PARSER_INFO(fmt, ...)                                                                                        \
    rock::log_parser::log::impl::Log::record(rock::log_parser::log::impl::Level::Info, __func__, __LINE__, fmt "\n", \
                                             ##__VA_ARGS__)

#define PARSER_DEBUG(fmt, ...)                                                                                        \
    rock::log_parser::log::impl::Log::record(rock::log_parser::log::impl::Level::Debug, __func__, __LINE__, fmt "\n", \
                                             ##__VA_ARGS__)

}    // namespace rock::log_parser::log
