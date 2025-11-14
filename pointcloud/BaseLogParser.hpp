//
// Created by ray on 2024/1/18.
//

#pragma once

#include <boost/circular_buffer.hpp>
#include <cassert>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include "log_type.hpp"
#include "Log.h"

namespace rock::log_parser {
    using timestamp = double;

    class BaseLogParser {
    public:
        using log_data_ptr = std::shared_ptr<LogData>;

    public:
        BaseLogParser(const LogType& type, int buf_capacity, bool (*filter_func_ptr)(LogData const&) = nullptr) :
                type_(type),
                read_done_(true),
                filter_func_ptr_(filter_func_ptr) {
            // 由于日志可能时间戳乱序，因此对每个日志读取时，需要排序
            // 于是每个日志维护一个buffer，buffer剩一半读满，读满排序
            // 只要乱序的那行日志不跳到capacity/2以外，就可以保证这个log_local_buffer_输出的日志永远有序
            log_local_buffer_.set_capacity(buf_capacity);
        }

        auto read_front() -> log_data_ptr {
            if (is_end()) {
                return nullptr;
            }

            if (log_local_buffer_.size() * 2 < log_local_buffer_.capacity() && !read_done_) {
                do_produce();
            }

            return log_local_buffer_.empty() ? nullptr : log_local_buffer_.front();
        }

        auto is_end() -> bool {
            return read_done_ && log_local_buffer_.empty();
        };

        virtual auto produce_one() -> log_data_ptr = 0;    // 负责读取一行日志，并且标记是否读取完成

        void consume_front() {
            assert(!log_local_buffer_.empty());
            log_local_buffer_.pop_front();
        }

        void reset() {
            log_local_buffer_.clear();
            do_produce();
        }

        virtual void seek_start(timestamp time_start) = 0;

        virtual void set_timestamp(timestamp time) = 0;

        virtual ~BaseLogParser() {}

    private:
        void do_produce() {
            while (!log_local_buffer_.full() && !read_done_) {
                auto one = produce_one();
                if (one) {
                    if (filter_func_ptr_ && filter_func_ptr_(*one)) {
                        continue;
                    }

                    log_local_buffer_.push_back(one);
                }
            }

            std::stable_sort(log_local_buffer_.begin(), log_local_buffer_.end(),
                             [](const log_data_ptr& l, const log_data_ptr& r) { return l->timestamp < r->timestamp; });
        }

    protected:
        LogType type_;
        bool    read_done_ = true;

    private:
        boost::circular_buffer<log_data_ptr> log_local_buffer_;
        bool (*filter_func_ptr_)(LogData const&);
    };
}    // namespace rock::log_parser
