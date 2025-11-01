#include "SerialBus.hpp"
#include "LidarReader.hpp"
#include "MotorController.hpp"
#include "Map.hpp"
#include "SLAM.hpp"
#include "Explorer.hpp"
#include "RobotConfig.hpp"

int main(){
    RobotConfig cfg;
    cfg.width = 0.25;     
    cfg.length = 0.3;     
    cfg.wheelBase = 0.15; 
    cfg.maxSpeed = 100;   

    cfg.pid_kp = 2.0;
    cfg.pid_ki = 0.0;
    cfg.pid_kd = 0.5;
    cfg.tankTurnThreshold = 0.35; // рад ≈ 20°

    SerialBus bus("/dev/ttyAMA0",115200);
    LidarReader lidar(bus);
    MotorController motor(bus,cfg);
    Map map(500,500,0.05);
    map.inflateObstacles(cfg.width, cfg.length);
    SLAM slam(lidar,map,motor,cfg);
    Explorer explorer(map,slam,motor,cfg);

    bus.writeDebug("Start exploration");

    for(;;){
        slam.step();
        explorer.step();
    }
}
