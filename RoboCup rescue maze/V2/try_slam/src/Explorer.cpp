#include "Explorer.hpp"
#include <queue>
#include <cmath>
#include <algorithm>

Explorer::Explorer(Map& map_, SLAM& slam_, MotorController& motor_, const RobotConfig& cfg)
: map(map_), slam(slam_), motor(motor_), config(cfg) {}

void Explorer::step() {
    if(currentPath.empty()){
        Eigen::Vector2d target = findNextFrontier();
        currentPath = planPath(slam.getPose().head<2>(), target);
    }
    moveAlongPath();
}

Eigen::Vector2d Explorer::findNextFrontier() {
    auto pose = slam.getPose().head<2>();
    auto [gx, gy] = map.worldToGrid(pose(0), pose(1));
    std::queue<std::pair<int,int>> q;
    std::vector<std::vector<bool>> visited(map.width, std::vector<bool>(map.height,false));
    q.push({gx,gy});
    visited[gx][gy]=true;

    while(!q.empty()){
        auto [x,y]=q.front(); q.pop();
        for(int dx=-1;dx<=1;++dx)
            for(int dy=-1;dy<=1;++dy){
                int nx=x+dx, ny=y+dy;
                if(nx<0||ny<0||nx>=map.width||ny>=map.height) continue;
                if(visited[nx][ny]) continue;
                visited[nx][ny]=true;
                if(map.isFree(nx,ny)) q.push({nx,ny});
                else return map.gridToWorld(nx,ny);
            }
    }
    return slam.getPose().head<2>();
}

std::vector<Eigen::Vector2d> Explorer::planPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal){
    auto [sx,sy]=map.worldToGrid(start(0),start(1));
    auto [gx,gy]=map.worldToGrid(goal(0),goal(1));

    struct Node { int x,y; double g,h; Node* parent; };
    auto cmp = [](Node* a, Node* b){ return (a->g + a->h) > (b->g + b->h); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
    std::vector<std::vector<bool>> closed(map.width, std::vector<bool>(map.height,false));

    Node* startNode = new Node{sx,sy,0,std::hypot(gx-sx,gy-sy),nullptr};
    open.push(startNode);

    Node* goalNode=nullptr;
    while(!open.empty()){
        Node* n=open.top(); open.pop();
        if(closed[n->x][n->y]) { delete n; continue; }
        closed[n->x][n->y]=true;
        if(n->x==gx && n->y==gy){ goalNode=n; break; }

        for(int dx=-1;dx<=1;++dx)
            for(int dy=-1;dy<=1;++dy){
                if(dx==0 && dy==0) continue;
                int nx=n->x+dx, ny=n->y+dy;
                if(nx<0||ny<0||nx>=map.width||ny>=map.height) continue;
                if(closed[nx][ny] || !map.isFree(nx,ny)) continue;
                Node* child = new Node{nx,ny,n->g+std::hypot(dx,dy),std::hypot(gx-nx,gy-ny),n};
                open.push(child);
            }
    }

    std::vector<Eigen::Vector2d> path;
    while(goalNode){
        path.push_back(map.gridToWorld(goalNode->x,goalNode->y));
        goalNode=goalNode->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void Explorer::moveAlongPath(){
    if(currentPath.empty()) return;
    Eigen::Vector2d target=currentPath.front();
    Eigen::Vector2d pos=slam.getPose().head<2>();
    Eigen::Vector2d dir=target-pos;
    double angle_to_target=atan2(dir(1),dir(0));
    double angle_error=angle_to_target-slam.getPose()(2);

    double left, right;

    if(std::abs(angle_error) > config.tankTurnThreshold){
        // Танковый разворот
        left  = (angle_error>0)? -config.maxSpeed/2 : config.maxSpeed/2;
        right = -left;
    } else {
        // PID-контроль курса
        integral += angle_error;
        double derivative = angle_error - last_error;
        last_error = angle_error;
        double correction = config.pid_kp*angle_error + config.pid_ki*integral + config.pid_kd*derivative;

        double baseSpeed = config.maxSpeed/2;
        left  = std::clamp(baseSpeed - correction, -config.maxSpeed, config.maxSpeed);
        right = std::clamp(baseSpeed + correction, -config.maxSpeed, config.maxSpeed);
    }

    motor.setSpeed(left,right);

    if(dir.norm()<0.05) currentPath.erase(currentPath.begin());
}
