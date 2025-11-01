#include "SerialBus.hpp"

SerialBus::SerialBus(const std::string& portName, unsigned int baud)
    : port(io, portName)
{
    port.set_option(asio::serial_port_base::baud_rate(baud));
}

bool SerialBus::readLine(std::string& line) {
    try {
        asio::read_until(port, buf, '\n');
        std::istream is(&buf);
        std::getline(is, line);
        return true;
    } catch (...) {
        return false;
    }
}

void SerialBus::writeMotor(const std::string& line) {
    std::lock_guard<std::mutex> lock(writeMutex);
    asio::write(port, asio::buffer("M;" + line + "\n"));
}

void SerialBus::writeDebug(const std::string& line) {
    std::lock_guard<std::mutex> lock(writeMutex);
    asio::write(port, asio::buffer("D;" + line + "\n"));
}
