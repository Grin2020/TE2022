#pragma once
#include <asio.hpp>
#include <string>
#include <mutex>

class SerialBus {
public:
    SerialBus(const std::string& portName, unsigned int baud);

    bool readLine(std::string& line);       
    void writeMotor(const std::string& line); 
    void writeDebug(const std::string& line); 

private:
    asio::io_service io;
    asio::serial_port port;
    asio::streambuf buf;
    std::mutex writeMutex;
};
