#include "pcd_writer.hpp"
#include <iostream>


void save_pcd(const std::string& filename, const std::vector<PointXYZI>& points) {
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    // 简单 PCD 文件头
    ofs << "# .PCD v0.7 - Point Cloud Data file\n";
    ofs << "FIELDS x y z intensity\n";
    ofs << "SIZE 4 4 4 4\n";
    ofs << "TYPE F F F F\n";
    ofs << "COUNT 1 1 1 1\n";
    ofs << "WIDTH " << points.size() << "\n";
    ofs << "HEIGHT 1\n";
    ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
    ofs << "POINTS " << points.size() << "\n";
    ofs << "DATA ascii\n";

    for (auto& p : points) {
        ofs << p.x << " " << p.y << " " << p.z << " " << p.intensity << " \n";
    }

    ofs.close();
}
std::vector<PointXYZI> TofToXYZI(const rock::log_parser::LogData& data) {
    std::vector<PointXYZI> points;
    if (data.type != rock::log_parser::LogDataType::Tof)
        return points;

    points.reserve(data.tof.cloud.size());
    for (const auto& p : data.tof.cloud) {
        points.push_back({p.x, p.y, p.z, p.i});
    }
    return points;
}

