#ifndef MOTORS_FEETECH_HPP_
#define MOTORS_FEETECH_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>

class FeetechMotorsBus {
public:
    FeetechMotorsBus(const std::string& port, 
                     const std::map<std::string, std::pair<int, std::string>>& motors);
    ~FeetechMotorsBus();

    void connect();
    void disconnect();

    // Read a parameter from all motors
    std::vector<int> read(const std::string& param_name);
    
    // Read a parameter from a specific motor
    int read(const std::string& param_name, const std::string& motor_name);
    
    // Write a parameter to all motors
    void write(const std::string& param_name, const std::vector<int>& values);
    
    // Write a parameter to a specific motor
    void write(const std::string& param_name, int value, const std::string& motor_name);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif  // MOTORS_FEETECH_HPP_
