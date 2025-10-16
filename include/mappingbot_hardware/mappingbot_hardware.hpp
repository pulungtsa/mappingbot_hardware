
#ifndef __MAPPINGBOT_HARDWARE__MAPPINGBOT_HARDWARE_H__
#define __MAPPINGBOT_HARDWARE__MAPPINGBOT_HARDWARE_H__

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <vector>
#include <limits>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "mappingbot_hardware/mappingbot_serial_port.hpp"
#include "mappingbot_hardware/mappingbot_hardware_compiler.h"

#define ENCODER_MIN 0
#define ENCODER_MAX 9000
#define ENCODER_LOW_WRAP_FACTOR 0.3
#define ENCODER_HIGH_WRAP_FACTOR 0.7

#define TICKS_PER_ROTATION 90

namespace mappingbot_hardware
{
    class MappingbotHardware
        : public hardware_interface::SystemInterface
    {
    // public methods (Humble-compatible)
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MappingbotHardware)

        MAPPINGBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        MAPPINGBOT_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        MAPPINGBOT_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        MAPPINGBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        MAPPINGBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        MAPPINGBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        MAPPINGBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


    private:
        hardware_interface::HardwareInfo info_;
        std::vector<uint8_t> motor_ids_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;
        std::vector<double> velocity_commands_;
        std::vector<double> velocity_commands_saved_;
        std::shared_ptr<MappingbotSerialPort> serial_port_;
        std::string serial_port_name_;
        void on_encoder_update(int16_t right, int16_t left);

        // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_pub[2];
        rclcpp::Time last_read;
        // std_msgs::msg::Float64 pos;

        // Last known encoder values
        int16_t last_wheelcountR;
        int16_t last_wheelcountL;
        // Count of full encoder wraps
        int multR;
        int multL;
        // Thresholds for calculating the wrap
        int low_wrap = ENCODER_LOW_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
        int high_wrap = ENCODER_HIGH_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;

    };
    
}

#endif // __MAPPINGBOT_HARDWARE__MAPPINGBOT_HARDWARE_H__