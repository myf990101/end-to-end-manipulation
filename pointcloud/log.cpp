#include "Log.h"

namespace rock::log_parser::log::impl {

Log::logger_type Log::logger = nullptr;
Log::writer_type Log::writer = nullptr;
Log::level_type   Log::level  = Log::level_type::Info;  // ✅ 使用 level_type

} // namespace rock::log_parser::log::impl
