#include <fstream>
#include <string>
#include <filesystem>
#include "TofLogParser.hpp"      // 含 parse_tof()
#include "pcd_writer.hpp"  // 含 save_pcd()

using rock::log_parser::LogData;
using rock::log_parser::parse_tof;
using rock::log_parser::ParseResult;
using tof_t = rock::log_parser::Tof;
int main(int argc, char** argv) {
    if (argc < 2) return 1;

    std::ifstream fs(argv[1], std::ios::binary);
    if (!fs) return 1;

    std::filesystem::path out_dir = std::filesystem::path(argv[1]).stem().string() + "_pcd";
    std::filesystem::create_directories(out_dir);

    int frame_idx = 0;
    bool old_bin10 = false;

    while (fs && !fs.eof()) {
        rock::log_parser::LogData data;
        if (rock::log_parser::parse_tof(fs, data, old_bin10) != rock::log_parser::ParseResult::Success)
            break;
        
        if (data.tof.exposure == tof_t::Exposure::HdrFlood) {
            //  转换 TOF 点云为 PCL 风格
        std::vector<PointXYZI> points = TofToXYZI(data);
        std::cout << "[DEBUG] data.sensor = " << data.sensor << std::endl;
        std::string filename = (out_dir / ("frame_" + std::to_string(frame_idx++) + ".pcd")).string();
        save_pcd(filename, points);
        }
    }

    return 0;
}
