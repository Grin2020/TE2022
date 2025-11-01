#include "SLAM.hpp"
#include <cmath>

SLAM::SLAM(LidarReader& lidar_, Map& map_, MotorController& motor_, const RobotConfig& cfg)
: lidar(lidar_), map(map_), motor(motor_), config(cfg), pose(0,0,0) {}

Eigen::Vector2d SLAM::polarToCartesian(double angle, double distance) {
    double rad = angle*M_PI/180.0;
    return Eigen::Vector2d(distance*cos(rad), distance*sin(rad));
}

void SLAM::scanMatch(const std::vector<LidarPoint>& newScan) {
    if(lastScan.empty()){ lastScan = newScan; return; }

    std::vector<Eigen::Vector2d> prevPts,newPts;
    for(auto& p:lastScan) prevPts.push_back(polarToCartesian(p.angle,p.distance));
    for(auto& p:newScan) newPts.push_back(polarToCartesian(p.angle,p.distance));

    Eigen::Vector2d delta(0,0);
    int n = std::min(prevPts.size(),newPts.size());
    for(int i=0;i<n;++i) delta += (prevPts[i]-newPts[i]);
    delta/=n;

    pose(0)+=delta(0);
    pose(1)+=delta(1);

    lastScan = newScan;
}

Eigen::Vector3d SLAM::getPose() const { return pose; }

void SLAM::step() {
    std::vector<LidarPoint> scan;
    if(!lidar.readScan(scan)) return;

    scanMatch(scan);

    std::vector<Eigen::Vector2d> points;
    for(auto& p:scan) points.push_back(polarToCartesian(p.angle,p.distance));
    map.update(points, pose);

    motor.setSpeed(config.maxSpeed/2, config.maxSpeed/2);
}
