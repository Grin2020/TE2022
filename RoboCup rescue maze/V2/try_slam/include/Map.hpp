#pragma once
#include <vector>
#include <Eigen/Dense>

class Map {
public:
    Map(int width, int height, double resolution);
    void update(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector3d& robotPose);
    void inflateObstacles(double robotWidth, double robotLength);
    bool isFree(int x, int y) const;
    Eigen::Vector2d gridToWorld(int gx, int gy) const;
    std::pair<int,int> worldToGrid(double x, double y) const;

    int width, height;
    double resolution;

private:
    std::vector<std::vector<int>> grid;
};
