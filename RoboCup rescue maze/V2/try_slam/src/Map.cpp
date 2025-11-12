#include "Map.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

Map::Map(int w, int h, double res) : width(w), height(h), resolution(res) {
    grid.resize(width, std::vector<int>(height, 0));
}

/*void Map::update(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector3d& robotPose) {
    for (auto& pt : points) {
        double x_world = robotPose(0) + pt(0)*cos(robotPose(2)) - pt(1)*sin(robotPose(2));
        double y_world = robotPose(1) + pt(0)*sin(robotPose(2)) + pt(1)*cos(robotPose(2));
        int xi = std::clamp(int(x_world / resolution), 0, width-1);
        int yi = std::clamp(int(y_world / resolution), 0, height-1);
        grid[xi][yi] = 1;
    }
}*/


void Map::update(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector3d& robotPose) {
    int updatedPoints = 0;
    for (auto& pt : points) {
        // Преобразуем из координат лидара в мировые
        double x_world = robotPose(0) + pt(0) * cos(robotPose(2)) - pt(1) * sin(robotPose(2));
        double y_world = robotPose(1) + pt(0) * sin(robotPose(2)) + pt(1) * cos(robotPose(2));

        // --- фильтруем шум ---
        if (std::isnan(x_world) || std::isnan(y_world)) continue;
        if (fabs(x_world) > width * resolution || fabs(y_world) > height * resolution) continue;

        // Проверяем границы карты
        int xi = int((x_world + width * resolution / 2) / resolution);
        int yi = int((y_world + height * resolution / 2) / resolution);

        if (xi < 0 || yi < 0 || xi >= width || yi >= height)
            continue;

        grid[xi][yi] = 1; // занятая ячейка
        updatedPoints++;
    }

    std::cout << "[DEBUG] Updated points in map: " << updatedPoints << std::endl;
}


void Map::inflateObstacles(double robotWidth, double robotLength) {
    int rx = int(robotWidth / resolution);
    int ry = int(robotLength / resolution);

    std::vector<std::vector<int>> newGrid = grid;
    for (int x = 0; x < width; ++x)
        for (int y = 0; y < height; ++y)
            if (grid[x][y] == 1)
                for (int dx=-rx; dx<=rx; ++dx)
                    for (int dy=-ry; dy<=ry; ++dy){
                        int nx = x+dx, ny=y+dy;
                        if(nx>=0 && nx<width && ny>=0 && ny<height)
                            newGrid[nx][ny] = 1;
                    }
    grid = newGrid;
}

bool Map::isFree(int x, int y) const {
    if(x<0 || x>=width || y<0 || y>=height) return false;
    return grid[x][y]==0;
}

Eigen::Vector2d Map::gridToWorld(int gx, int gy) const {
    return Eigen::Vector2d((gx+0.5)*resolution, (gy+0.5)*resolution);
}

std::pair<int,int> Map::worldToGrid(double x, double y) const {
    return {int(x/resolution), int(y/resolution)};
}
