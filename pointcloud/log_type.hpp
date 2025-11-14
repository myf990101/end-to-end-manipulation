#pragma once
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace rock::log_parser {
    enum class ParseResult {
        Success,    // success
        Error,      // error happened
        Ignore,     // ignore this result
    };

    enum class LogType : int32_t {
        PlayerSensor = 0,
        RockSensor   = 1,
        Command      = 2,
        Trajectory   = 3,
        Camera       = 4,
        Tof          = 5,
        Icon         = 6,
        Map          = 7,
        Nav          = 8,
        WireRect     = 9,
        WireMask     = 10,
        AvoidCube    = 11,
        ObjRect      = 12,
    };

    enum class LogDataType : int32_t {
        None = 0,
        Position2D,
        Position3D,
        Laser,
        MultiLaser,
        Bumper,
        MapStatus,
        Cmd_Reset,
        Cmd_Pause,
        Cmd_Lock,
        Cmd_Load,
        Cmd_SetPose,
        Cmd_Relocate,
        Cmd_GlobalRelocate,
        Cmd_RangeRelocate,
        GlobalRelocatePrepare,
        RelocateResult,
        Estimate,
        OdoGyro,
        Camera_Image,
        SlamPose,
        SlamTrajectory,
        Cmd_GlobalMovingRelocate,
        TrackingControl,
        Cmd_SetMapList,
        Cmd_Get_Prepare_Result,
        GlobalMovingRelocatePrepare,
        Cmd_SaveMap,
        Cmd_RotateMap,
        NavNormalStage,
        Tof,
        Cmd_SaveMessUpMap,
        Cmd_ActionType,
        Cmd_Pause_Moving_Relocate,
        Slip,
        NotSlip,
        Cmd_Top_Covered,
        Cmd_CheckMessUp,
        Cmd_FastMapRotate,
        Cmd_CalibMouse,
        DropLaser,
        Lds_Up,
        Lds_Down,
        Cliff,
        Map,
        Icon,
        Nav,
        WireRect,
        WireMask,
        Tof3DMap,
        AvoidCube,
        ObjRect,
        DropTof,
        Cmd_ClearPartShadowmap,
        DropGyroOdo,
        InitPose,
        Cmd_UpdateMapFast,
    };

    struct Point3D {
        float x;
        float y;
        float z;
    };

    using Acc3D = Point3D;

    struct EulerAng {
        float roll;
        float pitch;
        float yaw;
    };

    using EulerAngVel = EulerAng;

    struct Pose3D {
        Point3D  point;
        EulerAng euler;
    };

    struct Pose2D {
        float x;
        float y;
        float theta;
    };

    struct Point2D {
        float x;
        float y;
    };

    struct Quadrilateral {
        Point2D left_bottom;
        Point2D left_top;
        Point2D right_top;
        Point2D right_bottom;
    };

    struct Wheel {
        int32_t left;
        int32_t right;
    };

    struct Velocity {
        float v;
        float w;
    };

    struct Beam {
        float    rho;
        float    theta;
        uint16_t intensity;
        uint16_t flag;
    };

    struct Motion2D {
        Pose2D  pos;
        Pose2D  vel;
        uint8_t stall;
    };

    struct Motion3D {
        Pose3D  pos;
        Pose3D  vel;
        uint8_t stall;
    };

    struct Laser {
        uint32_t          id;
        uint32_t          subtype;
        float             min_angle;
        float             max_angle;
        float             resolution;
        float             max_range;
        std::vector<Beam> scan;

        void filter() {}    // depressed
    };

    struct MultiLaser {
        using Scan = std::vector<Beam>;
        uint32_t          id;
        uint32_t          subtype;
        float             min_angle;
        float             max_angle;
        float             resolution;
        float             max_range;
        std::vector<Scan> scans;

        void filter() {}    // depressed
    };

    struct Bumper {
        double timestamp;
        bool   left, front, right;
    };

    struct MapStatus {
        bool lock;
    };

    struct Command {
        enum class ActionType {
            SlamNormalClean = 0,
            SlamBuildMapClean,
            SlamFastBuildMap,
            SlamZonedClean,
            SlamSegmentClean,
            SlamSpot,
            SlamBackToDock,
            SlamBackToWash,
        };

        enum class LockReason {
            WheelLifting = 0,
            RobotNotFlat,
            ArmInMotion,
            MopPressureIncreased,
            MopPressureDecreased,
            Default = 99,
        };

        std::string            map;
        Pose2D                 pose;
        Quadrilateral          quad;
        bool                   succeeded;
        std::array<int32_t, 6> map_list;
        int32_t                map_id;
        int32_t                map_status;
        ActionType             action_type;
        LockReason             lock_reason;
    };

    struct Mouse {
        int32_t  x;
        int32_t  y;
        uint16_t image_quality;
        uint8_t  shutter;
        uint8_t  lt_src;
        uint8_t  init_ok;
        uint8_t  frame_avg;
        uint8_t  qrd;
        uint8_t  dirty;
        int32_t  mouse_valid_cnt;
    };

    /**
     * x,y : x轴位移，对底层DeltaX累计值
     * image_quality 每帧信号质量
     * shutter;   //0x15寄存器的值，代表曝光的一个参数
     * lt_src    //当前光源，0代表ld，1代表led
     * frame_avg;   // 0x61寄存器，Average brightness of a frame
     * qrd;    //0x6f 寄存器的值，具体代表什么未知，手册里没有
     * dirty
     * //根据信号质量低于某一阈值（比光源切换的阈值要高），并且结合stOPTRpt.u8Qrd，判断出的数据是否dirty的一个标志位，底层仅判断没有用，具体规律不详
     * mouse_valid_cnt //每次切光源后这个值为2，之后每次读取数据--，直到1.
     * 最开始应该是为了切光源后数据有效与否做的。底层工厂的时候有用。
     */

    struct OdoGyro {
        Pose2D      pose;
        Pose2D      odo_gyro_pose;
        Pose2D      revised_pose;
        Acc3D       acceleration;
        EulerAng    euler;
        EulerAngVel angular_vel;
        Velocity    actual_velocity;
        Velocity    desired_velocity;
        Wheel       wheel;
        Mouse       mouse;
    };

    struct Camera {
        std::string name;

        bool is_bmp() const {
            return name.substr(name.length() - 4) == ".bmp";
        }

        bool is_pgm() const {
            return name.substr(name.length() - 4) == ".pgm";
        }
    };

    struct Tof {
        enum class Exposure : int32_t {
            Unknown   = -1,
            Flood     = 0,
            ShortSpot = 1,
            LongSpot  = 2,
            HdrFlood  = 3,
            HdrSpot   = 4,
            Merged    = 5,
            Wire      = 6
        };

        enum class Sensor : int32_t { Unknown = -1, Front = 0, Left = 1, Top = 2, Merged = 3 };

        struct Point;

        Exposure           exposure;
        Pose2D             slam_pose;
        Pose2D             odom_pose;
        uint32_t           dim;
        uint32_t           size;
        std::vector<Point> cloud;
    };

    struct Tof::Point {
        struct AmplitudeConfidence {
            uint32_t amplitude;
            float    confidence;
        };

        float    x;
        float    y;
        float    z;
        float    i;
        uint16_t flag = 0;
        union {
            AmplitudeConfidence amp_con;
            int                 index;
            // other_type_t  other_type_data;
        };
    };

    struct SlamPose {
        Pose2D  current;
        Pose2D  motion;
        uint8_t stall;
    };

    struct MapList {
        std::array<int32_t, 6> map_list;
    };

    struct Cliff {
        bool FL = false;
        bool FR = false;
        bool CL = false;
        bool CR = false;
        bool BL = false;
        bool BR = false;
    };

    struct Map {
        std::string location;
    };

    struct Icon {
        float x;
        float y;
        int   remove_id;
    };

    struct Nav {
        struct TofSLC {
            int32_t            version;
            int32_t            sensor;
            std::vector<float> gnd;
            float              yaw;
            std::vector<float> trans;
            std::vector<float> gnd_rect;
        };

        enum class Type : int32_t {
            Unknown      = -1,
            Shadow_laser = 0,
            Obs_map      = 1,
            Obj_map      = 2,
            Det_map      = 3,
            Camera2d     = 4,
            Polygons     = 5,
            TofSLC       = 6
        };

        union {
            std::vector<Point3D> points;
            std::vector<TofSLC>  TofSLCs;
        };
    };

    struct WireRect {
        std::vector<Point3D> points;
    };

    struct WireMask {
        std::vector<uint32_t> mask;
    };

    struct AvoidCube {
        std::vector<float> cube;
        std::vector<float> seed;
        int                armPosition;
        int                armworkstatus;
    };

    struct ObjRect {
        std::vector<float> obj;
    };

    // todo: we may need constructor for each kind of type
    // the book "C++ primer" introduce the good way to construct/assign/destroy a union
    // if not treated correctly, some memory error may happen
    struct LogData {
        double      timestamp;
        LogDataType type;
        int32_t     sensor;
        union {
            Motion2D   position_2d;
            Motion3D   position_3d;
            Laser      laser;
            MultiLaser multi_laser;
            Bumper     bumper;
            MapStatus  map_status;
            Command    command;
            OdoGyro    odo_gyro;
            Camera     camera;
            Tof        tof;
            SlamPose   slam_pose;
            Cliff      cliff;
            Map        map;
            Icon       icon;
            Nav        nav;
            WireRect   rect;
            WireMask   mask;
            AvoidCube  avoid_cube;
            ObjRect    obj;
            double     droptime;
        };

        LogData() : timestamp{}, type{LogDataType::None}, sensor{}, tof{} {}

        LogData(LogData&& data) noexcept : timestamp(data.timestamp), type(data.type), sensor(data.sensor), tof{} {
            switch (type) {
                case LogDataType::Position2D:
                    position_2d = data.position_2d;
                    break;
                case LogDataType::Position3D:
                    position_3d = data.position_3d;
                    break;
                case LogDataType::Laser:
                    laser = std::move(data.laser);
                    break;
                case LogDataType::MultiLaser:
                    multi_laser = std::move(data.multi_laser);
                    break;
                case LogDataType::Bumper:
                    bumper = data.bumper;
                    break;
                case LogDataType::MapStatus:
                    map_status = data.map_status;
                    break;
                case LogDataType::Cmd_SetPose:
                case LogDataType::GlobalRelocatePrepare:
                case LogDataType::GlobalMovingRelocatePrepare:
                case LogDataType::Cmd_Get_Prepare_Result:
                case LogDataType::TrackingControl:
                case LogDataType::RelocateResult:
                case LogDataType::Estimate:
                case LogDataType::SlamTrajectory:
                case LogDataType::Cmd_SetMapList:
                case LogDataType::Cmd_Load:
                case LogDataType::Cmd_SaveMap:
                case LogDataType::Cmd_RotateMap:
                case LogDataType::NavNormalStage:
                case LogDataType::Cmd_ClearPartShadowmap:
                    ::new (&command) Command;
                    command = data.command;
                    break;
                case LogDataType::OdoGyro:
                    odo_gyro = data.odo_gyro;
                    break;
                case LogDataType::Camera_Image:
                    camera = std::move(data.camera);
                    break;
                case LogDataType::Tof:
                case LogDataType::Tof3DMap:
                    tof = std::move(data.tof);
                    break;
                case LogDataType::SlamPose:
                    slam_pose = data.slam_pose;
                    break;
                case LogDataType::Cliff:
                    cliff = data.cliff;
                    break;
                case LogDataType::Map:
                    map = data.map;
                    break;
                case LogDataType::Icon:
                    icon = data.icon;
                    break;
                case LogDataType::WireRect:
                    rect = std::move(data.rect);
                    break;
                case LogDataType::WireMask:
                    mask = std::move(data.mask);
                    break;
                case LogDataType::AvoidCube:
                    avoid_cube = std::move(data.avoid_cube);
                    break;
                case LogDataType::ObjRect:
                    obj = std::move(data.obj);
                    break;
                case LogDataType::DropTof:
                    droptime = data.droptime;
                    break;
                default:
                    break;
            }
        }

        ~LogData() {
            if (type == LogDataType::Laser) {
                laser.~Laser();
            }

            if (type == LogDataType::GlobalRelocatePrepare) {
                command.~Command();
            }

            if (type == LogDataType::Camera_Image) {
                camera.~Camera();
            }

            if (type == LogDataType::Tof || type == LogDataType::Tof3DMap) {
                tof.~Tof();
            }

            if (type == LogDataType::MultiLaser) {
                multi_laser.~MultiLaser();
            }
        }

        auto match(LogDataType const& data_type, int32_t sensor_id = 0) const -> bool {
            return type == data_type && sensor == sensor_id;
        }
    };

    struct LogPath {
        LogType     type;
        std::string path;
        bool (*filter_func_ptr)(LogData const&) = nullptr;
    };
}    // namespace rock::log_parser
