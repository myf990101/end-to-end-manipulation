#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>

#pragma pack(push, 1)

namespace rock::log_parser {
    enum Bin10Type : uint32_t {
        Bin10Type_Old                      = 0xF0DC0132U,
        Bin10Type_OldLeft                  = 0xF1DC0132U,
        Bin10Type_WidthBrightness          = 0xF2DC0132U,
        Bin10Type_FlagStatus               = 0xF3DC0132U,
        Bin10Type_PixelXY                  = 0xF4DC0132U,
        Bin10Type_ToF                      = 0xF5DC0132U,
        Bin10Type_ToFLeft                  = 0xF6DC0132U,
        Bin10Type_FlagStatusPixelXY        = 0xF7DC0132U,
        Bin10Type_ToFWithStructure         = 0xF8DC0132U,
        Bin10Type_ToFPerc                  = 0xF9DC0132U,
        Bin10Type_ToFWithExpose            = 0xFADC0132U,
        Bin10Type_ToFSplitPerc             = 0xFBDC0132U,
        Bin10Type_TofWithClass             = 0xFCDC0132U,
        Bin10Type_TofWithClusterId         = 0xFDDC0132U,
        Bin10Type_TofLooseWire             = 0xFEDC0132U,
        Bin10Type_TofWithCorrectZ          = 0xFFDC0132U,
        Bin10Type_TofLooseWireWithClassify = 0x00DC0132U,
        Bin10Type_OptionalItem             = 0x01DC0132U,
        Bin10Type_FrameFlagPixelXY         = 0x02DC0132U,
        Bin10Type_FrameFlagStatusPixelXY   = 0x03DC0132U,
        Bin10Type_SpecialPerc              = 0x11DC0132U,
        Bin10Type_3DMap                    = 0x22DC0132U,
        Bin10Type_PercTopIToF              = 0x33DC0132U,
    };

    struct Bin10PointBase {
        float x;
        float y;
        float z;
    };
    static_assert(sizeof(Bin10PointBase) == 12, "Bin10PointBase size error");

    struct Bin10PointWidthBrightness : public Bin10PointBase {
        uint8_t width;
        uint8_t brightness;
    };
    static_assert(sizeof(Bin10PointWidthBrightness) == 14, "Bin10PointWidthBrightness size error");

    struct Bin10PointFlagStatus : public Bin10PointWidthBrightness {
        uint8_t flag;
        uint8_t status;
    };
    static_assert(sizeof(Bin10PointFlagStatus) == 16, "Bin10PointFlagStatus size error");

    struct Bin10PointPixelXY : public Bin10PointWidthBrightness {
        float pixelX;
        float pixelY;
    };
    static_assert(sizeof(Bin10PointPixelXY) == 22, "Bin10PointPixelXY size error");

    struct Bin10PointFlagStatusPixelXY : public Bin10PointFlagStatus {
        float pixelX;
        float pixelY;
    };
    static_assert(sizeof(Bin10PointFlagStatusPixelXY) == 24, "Bin10PointFlagStatusPixelXY size error");

    struct Bin10PointWithIndex : public Bin10PointBase {
        uint16_t index;
    };
    static_assert(sizeof(Bin10PointWithIndex) == 14, "Bin10PointWithIndex size error");

    struct Bin10Pos {
        int32_t x;
        int32_t y;
        double  theta;
    };
    static_assert(sizeof(Bin10Pos) == 16, "Bin10Pos size error");

    struct Bin10PointToFCompact {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    static_assert(sizeof(Bin10PointToFCompact) == 6, "Bin10PointToFCompact size error");

    struct Bin10PointToFCompactWithOrder : public Bin10PointToFCompact {
        uint16_t origIdx;
    };
    static_assert(sizeof(Bin10PointToFCompactWithOrder) == 8, "Bin10PointToFCompactWithOrder size error");

    struct Bin10PointToFWithClusterId : public Bin10PointToFCompact {
        uint16_t clusterId;
    };
    static_assert(sizeof(Bin10PointToFWithClusterId) == 8, "Bin10PointToFWithClusterId size error");

    struct Bin10PointToFWithCorrectZ : public Bin10PointToFCompact {
        int16_t  correctZ;
        uint16_t clusterId;
    };
    static_assert(sizeof(Bin10PointToFWithCorrectZ) == 10, "Bin10PointToFWithCorrectZ size error");

    struct Bin10PointToFCompactWithConfidence : public Bin10PointToFCompact {
        uint16_t amplitude;
        uint8_t  row;
        uint8_t  col;
        float    confidence;
    };
    static_assert(sizeof(Bin10PointToFCompactWithConfidence) == 14, "Bin10PointToFCompactWithConfidence size error");

    struct BadBin10PointToFCompactWithConfidence : public Bin10PointToFCompact {
        uint16_t amplitude;
        uint16_t row;
        uint16_t col;
        float    confidence;
    };
    static_assert(sizeof(BadBin10PointToFCompactWithConfidence) == 16,
                  "BadBin10PointToFCompactWithConfidence size error");

    struct Bin10Point3DMap : public Bin10PointToFCompact {
        uint16_t confidence;
    };
    static_assert(sizeof(Bin10Point3DMap) == 8, "Bin10Point3DMap size error");

    struct Bin10HeaderBase {
        Bin10Type type;
        uint32_t  timestamp;
        Bin10Pos  robotPos;
        Bin10Pos  odoPos;
        float     yawSpeed;
    };
    static_assert(sizeof(Bin10HeaderBase) == 44, "Bin10HeaderBase size error");

    struct Bin10HeaderOld : public Bin10HeaderBase {
        int32_t        pointCount;
        Bin10PointBase data[0];
    };
    static_assert(sizeof(Bin10HeaderOld) == 48, "Bin10HeaderOld size error");

    struct Bin10HeaderStructLight : public Bin10HeaderBase {
        float          temperature;
        int32_t        pointCount;
        Bin10PointBase data[0];
    };
    static_assert(sizeof(Bin10HeaderStructLight) == 52, "Bin10HeaderStructLight size error");

    struct Bin10FlagHeaderStructLight : public Bin10HeaderBase {
        float          temperature;
        uint8_t        frameFlag;
        int32_t        pointCount;
        Bin10PointBase data[0];
    };
    static_assert(sizeof(Bin10FlagHeaderStructLight) == 53, "Bin10FlagHeaderStructLight size error");

    struct Bin10HeaderToF : public Bin10HeaderBase {
        uint16_t       dataType;
        int32_t        pointCount;
        Bin10PointBase data[0];
    };
    static_assert(sizeof(Bin10HeaderToF) == 50, "Bin10HeaderToF size error");

    struct Bin10HeaderToFWithStructure : public Bin10HeaderBase {
        uint16_t            dataType;    //0=flood, 1=short-spot, 2=long-spot
        uint16_t            isLeftToF;
        int32_t             pointCount;
        Bin10PointWithIndex data[0];
    };
    static_assert(sizeof(Bin10HeaderToFWithStructure) == 52, "Bin10HeaderToFWithStructure size error");

    struct Bin10HeaderToFPerc : public Bin10HeaderBase {
        uint32_t                      dataType    : 2;    //0=flood, 1=short-spot, 2=long-spot
        uint32_t                      isLeftToF   : 1;
        uint32_t                      isPercPoint : 1;    // produced by 'perception' module
        uint32_t                      reserved    : 28;
        int32_t                       pointCount;
        Bin10PointToFCompactWithOrder points[0];
    };
    static_assert(sizeof(Bin10HeaderToFPerc) == 52, "Bin10HeaderToFPerc error");

    struct Bin10HeaderToFWithExpose : public Bin10HeaderBase {
        uint32_t                           dataType : 4;    //3=hdr-flood, 4=hdr-spot
        uint32_t                           tofId    : 4;    //Front Left Top
        uint32_t                           reserved : 24;
        uint32_t                           exposeTime;
        int32_t                            pointCount;
        Bin10PointToFCompactWithConfidence points[0];
    };
    static_assert(sizeof(Bin10HeaderToFWithExpose) == 56, "Bin10HeaderToFWithExpose error");

    struct Bin10HeaderToFCompact : public Bin10HeaderBase {
        int32_t              pointCount;
        Bin10PointToFCompact points[0];
    };
    static_assert(sizeof(Bin10HeaderToFCompact) == 48, "Bin10HeaderToFCompact error");

    struct Bin10HeaderToFWithClass : public Bin10HeaderBase {
        uint16_t             classifyCount[4];
        uint32_t             reserved[6];
        int32_t              pointCount;
        Bin10PointToFCompact points[0];
    };
    static_assert(sizeof(Bin10HeaderToFWithClass) == 80, "Bin10HeaderToFWithClass error");

    struct Bin10HeaderToFWithClusterId : public Bin10HeaderBase {
        uint16_t                   classifyCount[4];
        uint32_t                   reserved[6];
        int32_t                    pointCount;
        Bin10PointToFWithClusterId points[0];
    };
    static_assert(sizeof(Bin10HeaderToFWithClusterId) == 80, "Bin10HeaderToFWithClusterId error");

    struct Bin10HeaderToFWithCorrectZ : public Bin10HeaderBase {
        uint16_t                  classifyCount[8];
        uint16_t                  reserved[7];
        uint8_t                   subVersion;
        uint8_t                   sizeofPoint;
        int32_t                   pointCount;
        Bin10PointToFWithCorrectZ points[0];
    };
    static_assert(sizeof(Bin10HeaderToFWithCorrectZ) == 80, "Bin10HeaderToFWithCorrectZ error");

    struct Bin10HeaderToFLooseWire : public Bin10HeaderBase {
        uint16_t             classiyCount[8];
        uint16_t             reserved[8];
        int32_t              pointCount;
        Bin10PointToFCompact points[0];
    };
    static_assert(sizeof(Bin10HeaderToFCompact) == 48, "Bin10HeaderToFCompact error");

    struct Bin10HeaderOptionalItem {
        Bin10Type type;
        uint32_t  timestamp;
        uint16_t  itemType;
        uint16_t  version;
        uint32_t  dataLen;
        char      data[0];
    };
    static_assert(sizeof(Bin10HeaderOptionalItem) == 16, "Bin10HeaderOptionalItem error");

    struct Bin10HeaderToFSpecialPerc : public Bin10HeaderBase {
        uint8_t              frameType;
        uint8_t              reserved[31];
        uint32_t             pointCount;
        Bin10PointToFCompact points[0];
    };
    static_assert(sizeof(Bin10HeaderToFSpecialPerc) == 80, "Bin10HeaderToFSpecialPerc error");

    struct Bin10Header3DMap : public Bin10HeaderBase {
        uint32_t        dataType : 4;
        uint32_t        tofId    : 4;
        uint32_t        reserved : 24;
        uint32_t        pointCount;
        Bin10Point3DMap points[0];
    };
    static_assert(sizeof(Bin10Header3DMap) == 52, "Bin10Header3DMap size error");

    struct Bin10HeaderPercTopIToF : public Bin10HeaderBase {
        static const size_t  NUM_CLASSIFY = 4;
        uint16_t             classifyCount[NUM_CLASSIFY];
        uint16_t             reserved[15 - NUM_CLASSIFY];
        uint8_t              subVersion;
        uint8_t              sizeofPoint;
        int32_t              pointCount;
        Bin10PointToFCompact points[0];
    };
    static_assert(sizeof(Bin10HeaderPercTopIToF) == 80, "Bin10HeaderPercTopIToF error");
}    // namespace rock::log_parser

#pragma pack(pop)
