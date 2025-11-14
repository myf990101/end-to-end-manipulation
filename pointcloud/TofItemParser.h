#pragma once

#include <fstream>

#include "log_type.hpp"
#include "tof_log_type.hpp"
#include "Log.h"

namespace rock::log_parser {
    auto parse_tof(std::ifstream& fs, LogData& data, bool old_bin10) -> ParseResult;
}    // namespace rock::log_parser