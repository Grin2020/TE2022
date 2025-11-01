#pragma once
#include <vector>
#include "SerialBus.hpp"

struct LidarPoint {
    double angle;    
    double distance; 
};

class LidarReader {
public:
    LidarReader(SerialBus& bus);
    bool readScan(std::vector<LidarPoint>& scan);

private:
    SerialBus& bus;
};
