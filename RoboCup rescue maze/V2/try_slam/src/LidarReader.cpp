#include "LidarReader.hpp"
#include <sstream>

LidarReader::LidarReader(SerialBus& bus_) : bus(bus_) {}

bool LidarReader::readScan(std::vector<LidarPoint>& scan) {
    scan.clear();
    std::string line;
    double last_angle = -1;

    for (;;) {
        if (!bus.readLine(line)) return false;
        std::stringstream ss(line);
        double angle, distance;
        char sep;
        if (!(ss >> angle >> sep >> distance)) continue;
        scan.push_back({angle, distance});
        if (angle < last_angle) break;
        last_angle = angle;
        if (scan.size() > 4000) break;
    }
    return true;
}
