//
// Created by ray on 2024/1/19.
//

#pragma once

#include "BaseLogParser.hpp"
#include "TofItemParser.h"

namespace rock::log_parser {
    class TofLogParser : public BaseLogParser {
    public:
        struct TofDataPos {
            TofDataPos() = default;
            TofDataPos(std::streampos& p, timestamp& t) : pos{p}, time{t} {}
            std::streampos pos;
            timestamp      time;
        };

    public:
        TofLogParser(const std::string& path, const LogType& type, int buf_capacity, bool old_bin10 = false,
                     bool (*filter_func_ptr)(LogData const&) = nullptr) :
                BaseLogParser(type, buf_capacity, filter_func_ptr),
                old_bin10_(old_bin10) {
            std::vector<cv::String> result;
            try {
                cv::glob(path, result, true);
            } catch (...) {
                PARSER_ERROR("fail to read files of %s", path.c_str());
                result.clear();
            }

            if (result.empty()) {
                PARSER_ERROR("No tof file found!");
                read_done_ = true;
                return;
            }
            read_done_ = false;
            fs_        = std::ifstream(result.front());
        }

        auto produce_one() -> log_data_ptr override {
            if (read_done_) {
                return nullptr;
            }

            while (!fs_.eof()) {
                LogData data{};

                auto pos = fs_.tellg();
                auto r   = parse_tof(fs_, data, old_bin10_);

                if (r == ParseResult::Success) {
                    all_data_.emplace_back(pos, data.timestamp);
                    return std::make_shared<LogData>(std::move(data));
                } else if (r == ParseResult::Ignore) {
                    continue;
                } else if (r == ParseResult::Error) {
                    //PARSER_ERROR
                    break;
                }
            }

            read_done_ = true;
            return nullptr;
        }

        void seek_start(timestamp time_start) override {
            // binary file, cannot find accurate time to filter
        }

        void set_timestamp(timestamp time) override {
            if (read_done_) {
                return;
            }

            for (auto i = all_data_.size() - 1; i > 0; --i) {
                const auto& data = all_data_.at(i);
                if (data.time > time) {
                    continue;
                }

                all_data_.resize(i + 1);
                fs_.seekg(all_data_.back().pos);
                return;
            }

            all_data_.clear();
            fs_.seekg(0);
        }

    private:
        std::ifstream          fs_;
        std::deque<TofDataPos> all_data_;
        bool                   old_bin10_;
    };
}    // namespace rock::log_parser