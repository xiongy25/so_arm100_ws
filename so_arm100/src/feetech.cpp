#include "motors/feetech.hpp"
#include <stdexcept>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

class FeetechMotorsBus::Impl {
public:
    int fd_;
    std::map<std::string, std::pair<int, std::string>> motors_;
    
    Impl(const std::string& port, 
         const std::map<std::string, std::pair<int, std::string>>& motors)
        : port_(port), motors_(motors), fd_(-1) {}
    
    ~Impl() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    void connect() {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open port: " + port_);
        }

        struct termios tty;
        if (::tcgetattr(fd_, &tty) != 0) {
            throw std::runtime_error("Failed to get port attributes");
        }

        ::cfsetospeed(&tty, B1000000);
        ::cfsetispeed(&tty, B1000000);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
            throw std::runtime_error("Failed to set port attributes");
        }

        // Clear any existing data
        ::tcflush(fd_, TCIOFLUSH);
    }

    void disconnect() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    // Helper function to send command and read response
    std::vector<uint8_t> sendCommand(const std::vector<uint8_t>& cmd) {
        // Write command
        if (::write(fd_, cmd.data(), cmd.size()) != static_cast<ssize_t>(cmd.size())) {
            throw std::runtime_error("Failed to write command");
        }
        
        // Wait for response
        ::usleep(5000);  // 5ms delay
        
        // Read response
        std::vector<uint8_t> response(128);
        ssize_t n = ::read(fd_, response.data(), response.size());
        if (n < 0) {
            throw std::runtime_error("Failed to read response");
        }
        response.resize(n);
        return response;
    }

    std::vector<int> read(const std::string& param_name) {
        std::vector<int> values;
        for (const auto& [name, info] : motors_) {
            values.push_back(read(param_name, name));
        }
        return values;
    }

    int read(const std::string& param_name, const std::string& motor_name) {
        auto it = motors_.find(motor_name);
        if (it == motors_.end()) {
            throw std::runtime_error("Unknown motor: " + motor_name);
        }

        int id = it->second.first;
        
        // Construct read command
        std::vector<uint8_t> cmd;
        if (param_name == "Present_Position") {
            cmd = {0xFF, 0xFF, static_cast<uint8_t>(id), 0x04, 0x02, 0x24, 0x02, 0x00};  // Read position command
            // Calculate checksum
            uint8_t checksum = 0;
            for (size_t i = 2; i < cmd.size(); i++) {
                checksum += cmd[i];
            }
            cmd.push_back(~checksum);
        } else if (param_name == "Moving") {
            cmd = {0xFF, 0xFF, static_cast<uint8_t>(id), 0x04, 0x02, 0x2E, 0x01, 0x00};  // Read moving status
            uint8_t checksum = 0;
            for (size_t i = 2; i < cmd.size(); i++) {
                checksum += cmd[i];
            }
            cmd.push_back(~checksum);
        } else {
            throw std::runtime_error("Unsupported parameter: " + param_name);
        }

        auto response = sendCommand(cmd);
        
        // Parse response
        if (response.size() >= 6 && response[0] == 0xFF && response[1] == 0xFF) {
            if (param_name == "Present_Position") {
                return (response[5] << 8) | response[4];  // Combine high and low bytes
            } else if (param_name == "Moving") {
                return response[4];
            }
        }
        
        throw std::runtime_error("Invalid response");
    }

    void write(const std::string& param_name, const std::vector<int>& values) {
        if (values.size() != motors_.size()) {
            throw std::runtime_error("Number of values does not match number of motors");
        }

        size_t i = 0;
        for (const auto& [name, _] : motors_) {
            write(param_name, values[i++], name);
        }
    }

    void write(const std::string& param_name, int value, const std::string& motor_name) {
        auto it = motors_.find(motor_name);
        if (it == motors_.end()) {
            throw std::runtime_error("Unknown motor: " + motor_name);
        }

        int id = it->second.first;
        
        // Construct write command
        std::vector<uint8_t> cmd;
        if (param_name == "Goal_Position") {
            cmd = {0xFF, 0xFF, static_cast<uint8_t>(id), 0x05, 0x03, 0x1E, 
                  static_cast<uint8_t>(value & 0xFF),  // Low byte
                  static_cast<uint8_t>((value >> 8) & 0xFF)};  // High byte
        } else if (param_name == "Torque_Enable") {
            cmd = {0xFF, 0xFF, static_cast<uint8_t>(id), 0x04, 0x03, 0x28, 
                  static_cast<uint8_t>(value ? 1 : 0)};
        } else if (param_name == "P_Coefficient") {
            cmd = {0xFF, 0xFF, static_cast<uint8_t>(id), 0x04, 0x03, 0x0E, 
                  static_cast<uint8_t>(value)};
        } else {
            throw std::runtime_error("Unsupported parameter: " + param_name);
        }
        
        // Calculate checksum
        uint8_t checksum = 0;
        for (size_t i = 2; i < cmd.size(); i++) {
            checksum += cmd[i];
        }
        cmd.push_back(~checksum);
        
        // Send command and get response
        auto response = sendCommand(cmd);
        
        // Check response
        if (response.size() < 6 || response[0] != 0xFF || response[1] != 0xFF) {
            throw std::runtime_error("Invalid response");
        }
    }

private:
    std::string port_;
};

FeetechMotorsBus::FeetechMotorsBus(
    const std::string& port,
    const std::map<std::string, std::pair<int, std::string>>& motors)
    : impl_(new Impl(port, motors)) {}

FeetechMotorsBus::~FeetechMotorsBus() = default;

void FeetechMotorsBus::connect() { impl_->connect(); }
void FeetechMotorsBus::disconnect() { impl_->disconnect(); }
std::vector<int> FeetechMotorsBus::read(const std::string& param_name) { return impl_->read(param_name); }
int FeetechMotorsBus::read(const std::string& param_name, const std::string& motor_name) { return impl_->read(param_name, motor_name); }
void FeetechMotorsBus::write(const std::string& param_name, const std::vector<int>& values) { impl_->write(param_name, values); }
void FeetechMotorsBus::write(const std::string& param_name, int value, const std::string& motor_name) { impl_->write(param_name, value, motor_name); }
