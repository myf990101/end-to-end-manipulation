#pragma once
#include <string>
#include <vector>
#include <fstream>
#include "TofLogParser.hpp" 

struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;  // 对应 tof::Point::i
};

void save_pcd(const std::string& filename, const std::vector<PointXYZI>& points);
std::vector<PointXYZI> TofToXYZI(const rock::log_parser::LogData& data);