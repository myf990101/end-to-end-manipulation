#include "TofItemParser.h"

#include <fstream>

#include "log_type.hpp"
#include "tof_log_type.hpp"
#include "Log.h"

namespace rock::log_parser {
    auto parse_tof_v1(std::ifstream& fs, LogData& data) -> ParseResult {
        PARSER_ERROR("not support Coral log old.");
        return ParseResult::Error;
    }

    auto parse_tof_v2(std::ifstream& fs, LogData& data) -> ParseResult {
        using log_point_t       = Bin10PointWithIndex;
        using log_point_cloud_t = Bin10HeaderToFWithStructure;

        log_point_cloud_t buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(log_point_cloud_t))) {
            return ParseResult::Error;
        }

        std::vector<log_point_t> raw(buf.pointCount);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(log_point_t) * raw.size()))) {
            return ParseResult::Error;
        }

        data.timestamp = static_cast<double>(buf.timestamp) / 1000;
        data.type      = LogDataType::Tof;
        data.sensor    = static_cast<int32_t>(buf.isLeftToF ? Tof::Sensor::Left : Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure(buf.dataType);
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];
            point.x     = raw[i].x / 1000;
            point.y     = raw[i].y / 1000;
            point.z     = raw[i].z / 1000;
            point.i     = 1;
            point.index = static_cast<int32_t>(raw[i].index);
        }
        return ParseResult::Success;
    }

    auto parse_tof_v3(std::ifstream& fs, LogData& data) -> ParseResult {
        using log_point_t       = Bin10PointToFCompactWithOrder;
        using log_point_cloud_t = Bin10HeaderToFPerc;

        log_point_cloud_t buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(log_point_cloud_t))) {
            return ParseResult::Error;
        }

        std::vector<log_point_t> raw(buf.pointCount);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(log_point_t) * raw.size()))) {
            return ParseResult::Error;
        }

        if (buf.isPercPoint == 1) {
            return ParseResult::Ignore;
        }

        data.timestamp = static_cast<double>(buf.timestamp) / 1000;
        data.type      = LogDataType::Tof;
        data.sensor    = static_cast<int32_t>(buf.isLeftToF ? Tof::Sensor::Left : Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure(buf.dataType);
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];
            point.x     = static_cast<float>(raw[i].x) / 1000;
            point.y     = static_cast<float>(raw[i].y) / 1000;
            point.z     = static_cast<float>(raw[i].z) / 1000;
            point.i     = 1;
            point.index = static_cast<int32_t>(raw[i].origIdx);
        }

        return ParseResult::Success;
    }

    auto parse_tof_v4(std::ifstream& fs, LogData& data) -> ParseResult {
        using log_point_t       = Bin10PointToFCompactWithConfidence;
        using log_point_cloud_t = Bin10HeaderToFWithExpose;

        log_point_cloud_t buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(log_point_cloud_t))) {
            return ParseResult::Error;
        }

        std::vector<log_point_t> raw(buf.pointCount);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(log_point_t) * raw.size()))) {
            return ParseResult::Error;
        }

        data.timestamp = static_cast<double>(buf.timestamp) / 1000;
        data.type      = LogDataType::Tof;
        data.sensor    = static_cast<int32_t>(buf.tofId == 0   ? Tof::Sensor::Front
                                              : buf.tofId == 1 ? Tof::Sensor::Left
                                              : buf.tofId == 2 ? Tof::Sensor::Top
                                                               : Tof::Sensor::Unknown);

        if (data.sensor == static_cast<int32_t>(Tof::Sensor::Unknown)) {
            PARSER_ERROR("%f bad type id %d", data.timestamp, buf.tofId);
        }

        data.tof.exposure = Tof::Exposure(buf.dataType);
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];
            point.x     = static_cast<float>(raw[i].x) / 1000;
            point.y     = static_cast<float>(raw[i].y) / 1000;
            point.z     = static_cast<float>(raw[i].z) / 1000;
            point.i     = raw[i].confidence;

            point.amp_con.confidence = raw[i].confidence;
            point.amp_con.amplitude  = raw[i].amplitude;
        }
        return ParseResult::Success;
    }

    auto parse_tof_v5(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderToFWithClass;
        using Point = Bin10PointToFCompact;
        Head buf;

        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }

        data.timestamp = static_cast<double>(buf.timestamp) / 1000;
        data.type      = LogDataType::Tof;
        data.sensor    = static_cast<int32_t>(Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure::HdrFlood;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        std::vector<Point> raw(data.tof.size);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(Point) * data.tof.size))) {
            PARSER_WARN("err read points");
            return ParseResult::Error;
        }

        auto type_id       = 0;
        auto type_id_count = 0;

        while (type_id_count >= buf.classifyCount[type_id]) {
            ++type_id;
        }

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];

            point.x = static_cast<float>(raw[i].x) / 1000.f;
            point.y = static_cast<float>(raw[i].y) / 1000.f;
            point.z = static_cast<float>(raw[i].z) / 1000.f;
            point.i = type_id;

            while (++type_id_count >= buf.classifyCount[type_id]) {
                ++type_id;
                type_id_count = -1;
            }
        }
        return ParseResult::Success;
    }

    auto parse_tof_v6(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderToFWithClusterId;
        using Point = Bin10PointToFWithClusterId;
        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }

        data.timestamp = static_cast<double>(buf.timestamp) / 1000;
        data.type      = LogDataType::Tof;
        data.sensor    = static_cast<int32_t>(Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure::HdrFlood;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        std::vector<Point> raw(data.tof.size);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(Point) * data.tof.size))) {
            PARSER_WARN("err read points");
            return ParseResult::Error;
        }

        auto type_id       = 0;
        auto type_id_count = 0;

        while (type_id_count >= buf.classifyCount[type_id]) {
            ++type_id;
        }

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];

            point.x     = static_cast<float>(raw[i].x) / 1000.f;
            point.y     = static_cast<float>(raw[i].y) / 1000.f;
            point.z     = static_cast<float>(raw[i].z) / 1000.f;
            point.i     = type_id;
            point.index = raw[i].clusterId;

            while (++type_id_count >= buf.classifyCount[type_id]) {
                ++type_id;
                type_id_count = -1;
            }
        }
        return ParseResult::Success;
    }

    auto parse_tof_v7(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderToFCompact;
        using Point = Bin10PointToFCompact;
        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }
        data.timestamp = static_cast<double>(buf.timestamp) / 1000;

        data.type   = LogDataType::Tof;
        data.sensor = static_cast<int32_t>(Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure::HdrFlood;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        std::vector<Point> raw(data.tof.size);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(Point) * data.tof.size))) {
            PARSER_WARN("err read points");
            return ParseResult::Error;
        }

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];

            point.x = static_cast<float>(raw[i].x) / 1000.f;
            point.y = static_cast<float>(raw[i].y) / 1000.f;
            point.z = static_cast<float>(raw[i].z) / 1000.f;
            point.i = 1;
        }
        return ParseResult::Success;
    }

    auto parse_tof_v8(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderToFWithCorrectZ;
        using Point = Bin10PointToFWithCorrectZ;
        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }
        data.timestamp = static_cast<double>(buf.timestamp) / 1000;

        data.type   = LogDataType::Tof;
        data.sensor = static_cast<int32_t>(Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure::HdrFlood;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        auto classify = 0;

        std::vector<int> classify_count(8);

        for (auto i = 0; i < 8; ++i) {
            classify_count[i] = buf.classifyCount[i];
        }

        for (auto& point : data.tof.cloud) {
            Point raw;
            if (!fs.read(reinterpret_cast<char*>(&raw), static_cast<std::streamsize>(sizeof(Point)))) {
                PARSER_WARN("err read points");
                return ParseResult::Error;
            }

            point.x     = static_cast<float>(raw.x) / 1000.f;
            point.y     = static_cast<float>(raw.y) / 1000.f;
            point.z     = static_cast<float>(raw.z) / 1000.f;
            point.index = raw.clusterId;

            if (buf.sizeofPoint > 10) {
                auto cur = fs.tellg();
                cur += buf.sizeofPoint - 10;
                fs.seekg(cur);
            }

            while (classify < 8 && classify_count[classify] <= 0) {
                ++classify;
            }
            --classify_count[classify];

            point.i = classify;

            if (classify == 2) {
                point.z = raw.correctZ / 1000.f;
            }
        }
        return ParseResult::Success;
    }

    auto parse_tof_v8_old(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderToFWithCorrectZ;
        using Point = Bin10PointToFWithCorrectZ;
        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }
        data.timestamp = static_cast<double>(buf.timestamp) / 1000;

        data.type   = LogDataType::Tof;
        data.sensor = static_cast<int32_t>(Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure::HdrFlood;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        std::vector<Point> raw(data.tof.size);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(Point) * data.tof.size))) {
            PARSER_WARN("err read points");
            return ParseResult::Error;
        }

        auto type_id       = 0;
        auto type_id_count = 0;

        while (type_id_count >= buf.classifyCount[type_id]) {
            ++type_id;
        }

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];

            point.x     = static_cast<float>(raw[i].x) / 1000.f;
            point.y     = static_cast<float>(raw[i].y) / 1000.f;
            point.z     = static_cast<float>(raw[i].z) / 1000.f;
            point.index = raw[i].clusterId;
            point.i     = type_id;

            while (++type_id_count >= buf.classifyCount[type_id]) {
                ++type_id;
                type_id_count = -1;
            }
        }
        return ParseResult::Success;
    }

    auto parse_tof_wire(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderToFLooseWire;
        using Point = Bin10PointToFCompact;
        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }
        data.timestamp = static_cast<double>(buf.timestamp) / 1000;

        data.type   = LogDataType::Tof;
        data.sensor = static_cast<int32_t>(Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure::Wire;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        std::vector<Point> raw(data.tof.size);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(Point) * data.tof.size))) {
            PARSER_WARN("err read points");
            return ParseResult::Error;
        }

        auto type_id       = 0;
        auto type_id_count = 0;

        while (type_id_count >= buf.classiyCount[type_id]) {
            ++type_id;
        }

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];

            point.x = static_cast<float>(raw[i].x) / 1000.f;
            point.y = static_cast<float>(raw[i].y) / 1000.f;
            point.z = static_cast<float>(raw[i].z) / 1000.f;
            point.i = type_id;

            while (++type_id_count >= buf.classiyCount[type_id]) {
                ++type_id;
                type_id_count = -1;
            }
        }
        return ParseResult::Success;
    }

    auto parse_tof_flag(std::ifstream& fs, LogData& data) -> ParseResult {
        using log_head_t  = Bin10HeaderOptionalItem;
        using log_point_t = uint8_t;
        log_head_t buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }

        double timestamp = static_cast<double>(buf.timestamp) / 1000.0;
        assert(std::abs(timestamp - data.timestamp) <= 0.001);
        assert(data.tof.size == buf.dataLen);

        std::vector<log_point_t> raw(buf.dataLen);
        if (!fs.read(reinterpret_cast<char*>(raw.data()), sizeof(log_point_t) * buf.dataLen)) {
            PARSER_WARN("err read data");
            return ParseResult::Error;
        }

        for (auto i = 0; i < raw.size(); ++i) {
            data.tof.cloud[i].flag = raw[i];
        }

        return ParseResult::Success;
    }

    auto parse_tof_v9(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderToFSpecialPerc;
        using Point = Bin10PointToFCompact;

        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }
        data.timestamp = static_cast<double>(buf.timestamp) / 1000;

        data.type   = LogDataType::Tof;
        data.sensor = static_cast<int32_t>(Tof::Sensor::Front);

        data.tof.exposure = Tof::Exposure::HdrFlood;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        for (auto& point : data.tof.cloud) {
            Point raw;
            if (!fs.read(reinterpret_cast<char*>(&raw), static_cast<std::streamsize>(sizeof(Point)))) {
                PARSER_WARN("err read points");
                return ParseResult::Error;
            }

            point.x = static_cast<float>(raw.x) / 1000.f;
            point.y = static_cast<float>(raw.y) / 1000.f;
            point.z = static_cast<float>(raw.z) / 1000.f;
            point.i = buf.frameType;
        }
        return ParseResult::Success;
    }

    auto parse_tof_3dmap(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10Header3DMap;
        using Point = Bin10Point3DMap;
        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }
        data.timestamp = static_cast<double>(buf.timestamp) / 1000;

        data.type   = LogDataType::Tof3DMap;
        data.sensor = static_cast<int32_t>(buf.tofId);

        data.tof.exposure = static_cast<Tof::Exposure>(buf.dataType);
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);
        std::vector<Point> raw(data.tof.size);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(Point) * data.tof.size))) {
            PARSER_WARN("err read points");
            return ParseResult::Error;
        }

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];

            point.x = static_cast<float>(raw[i].x) / 1000.f;
            point.y = static_cast<float>(raw[i].y) / 1000.f;
            point.z = static_cast<float>(raw[i].z) / 1000.f;
            point.i = static_cast<float>(raw[i].confidence);
        }

        return ParseResult::Success;
    }
    auto parse_tof_perc_top_itof(std::ifstream& fs, LogData& data) -> ParseResult {
        using Head  = Bin10HeaderPercTopIToF;
        using Point = Bin10PointToFCompact;
        Head buf;
        if (!fs.read(reinterpret_cast<char*>(&buf), sizeof(buf))) {
            PARSER_WARN("fail read head");
            return ParseResult::Error;
        }
        data.timestamp = static_cast<double>(buf.timestamp) / 1000;

        data.type   = LogDataType::Tof;
        data.sensor = static_cast<int32_t>(Tof::Sensor::Top);

        data.tof.exposure = Tof::Exposure::Flood;
        data.tof.dim      = 3;
        data.tof.size     = buf.pointCount;
        data.tof.cloud.resize(data.tof.size);

        auto noise_count  = buf.classifyCount[0];
        auto arm_count    = buf.classifyCount[0] + buf.classifyCount[1];
        auto object_count = buf.classifyCount[0] + buf.classifyCount[1] + buf.classifyCount[2];

        std::vector<Point> raw(data.tof.size);
        if (!fs.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(sizeof(Point) * data.tof.size))) {
            PARSER_WARN("err read points");
            return ParseResult::Error;
        }

        for (auto i = 0; i < data.tof.cloud.size(); ++i) {
            auto& point = data.tof.cloud[i];

            point.x = static_cast<float>(raw[i].x) / 1000.f;
            point.y = static_cast<float>(raw[i].y) / 1000.f;
            point.z = static_cast<float>(raw[i].z) / 1000.f;

            if (i < noise_count) {
                point.i = 1;
                continue;
            }

            if (i < arm_count) {
                point.i = 2;
                continue;
            }

            if (i < object_count) {
                point.i = 3;
                continue;
            }

            point.i = 1;
        }

        return ParseResult::Success;
    }

    // if fail, the ifstream may meet EOF
    template<typename T>
    bool peek(std::ifstream& fs, T& data) {
        auto pos_before_read = fs.tellg();
        if (!fs.read((char*)(&data), sizeof(data)))
            return false;
        fs.seekg(pos_before_read);
        return true;
    }

    auto parse_tof(std::ifstream& fs, LogData& data, bool old_bin10) -> ParseResult {
        assert(fs);
        Bin10Type data_type;

        if (!peek<Bin10Type>(fs, data_type)) {
            return ParseResult::Error;
        }

        // step1: read data
        if (data_type == Bin10Type::Bin10Type_ToFWithStructure) {
            if (parse_tof_v2(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_ToFPerc) {
            if (parse_tof_v3(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_ToFWithExpose) {
            if (parse_tof_v4(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_TofWithClass) {
            if (parse_tof_v5(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_TofWithClusterId) {
            if (parse_tof_v6(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_TofLooseWire) {
            if (parse_tof_v7(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_TofWithCorrectZ) {
            if ((old_bin10 ? parse_tof_v8_old(fs, data) : parse_tof_v8(fs, data)) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_TofLooseWireWithClassify) {
            if (parse_tof_wire(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_SpecialPerc) {
            if (parse_tof_v9(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_3DMap) {
            if (parse_tof_3dmap(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else if (data_type == Bin10Type::Bin10Type_PercTopIToF) {
            if (parse_tof_perc_top_itof(fs, data) == ParseResult::Error)
                return ParseResult::Error;
        } else {
            PARSER_ERROR("bad data type %08X", data_type);
            return ParseResult::Error;
        }

        // step2: if next is opt data, append it

        if (peek(fs, data_type) && data_type == Bin10Type::Bin10Type_OptionalItem) {
            // append flag to data
            if (parse_tof_flag(fs, data) == ParseResult::Error) {
                return ParseResult::Error;
            }
        }

        return ParseResult::Success;
    }
}    // namespace rock::log_parser