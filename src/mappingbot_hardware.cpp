#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"



#include "mappingbot_hardware/mappingbot_hardware.hpp"
#include "mappingbot_hardware/mappingbot_config.h"

PLUGINLIB_EXPORT_CLASS(
    mappingbot_hardware::MappingbotHardware,
    hardware_interface::SystemInterface
)   

using namespace mappingbot_hardware;

hardware_interface::CallbackReturn MappingbotHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SystemInterface::on_init(info) != 
        hardware_interface::CallbackReturn::SUCCESS) 
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
 
    // battVoltage_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
    // board_temperatures_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
    // cfg_.device = info_.hardware_parameters["serial_port_device"];
    // cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    serial_port_name_ = info_.hardware_parameters["serial_port"];
    motor_ids_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // TODO : Cara configure dan masukin data batteryVolt dan board temp
    // for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    //     if (sensor.name["battVoltgae_"].empty()) {
    //         RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), "Motor id not defined for join %s", joint.name.c_str());
    //         return hardware_interface::CallbackReturn::ERROR;
    //     }

    //     if (sensor.name["board_temperatures_"].empty()) {
    //         RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), "Motor id not defined for join %s", joint.name.c_str());
    //         return hardware_interface::CallbackReturn::ERROR;
    //     }
    // }

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // DiffBotSystem has exactly two states and one command interface on each joint
        auto motor_id_it = joint.parameters.find("motor_id");
        if (motor_id_it == joint.parameters.end() || motor_id_it->second.empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), 
                        "Motor id not defined for joint %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MappingbotHardware"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    
    for (size_t i = 0; i < info_.joints.size(); i++) {
        motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
        // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
    }

    // serial_port_->open(cfg_.device);
    // serial_port_ = std::make_shared<MappingbotSerialPort>();
    // if (serial_port_->open(cfg_.device) != CallbackReturn::SUCCESS) {
    //     RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Mappingbot hardware failed to open serial port");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    // vel_pub_[0]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/velocity", 10);
    // vel_pub_[1]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/velocity", 10);
    // cmd_pub_[0]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/cmd", 10);
    // cmd_pub_[1]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/cmd", 10);
    // voltage_pub_   = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/battery_voltage", 10);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MappingbotHardware::export_state_interfaces()
{
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (size_t i = 0; i < info_.sensors.size(); i++) {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[i].name, "batteryVoltage", &battVoltage_[i]));
    // }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MappingbotHardware::export_command_interfaces()
{
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn MappingbotHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Hardware activated!");
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Mappingbot hardware starting ...please wait..");

    // for (auto i = 0; i <= hw_start_sec_; i++)
    // {
    //     rclcpp::sleep_for(std::chrono::seconds(1));
    //     RCLCPP_INFO(
    //     rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    // }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(position_states_[i])) {
            position_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        velocity_commands_saved_[i] = velocity_commands_[i];
    }

    serial_port_ = std::make_shared<MappingbotSerialPort>();
    if (!serial_port_->open(serial_port_name_)) {
        RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"),
                    "Mappingbot hardware failed to open serial port");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // status_ = hardware_interface::status::STARTED;

    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Mappingbot hardware System Successfully started!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MappingbotHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Hardware deactivated!");
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Mappingbot hardware is stopping ...");

    // for (auto i = 0; i <= hw_stop_sec_; i++)
    // {
    //     rclcpp::sleep_for(std::chrono::seconds(1));
    //     RCLCPP_INFO(
    //     rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    // }

    if (serial_port_->is_open()) {
        serial_port_->close();
        serial_port_.reset();
    }

    // status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Mappingbot hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MappingbotHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)

{
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Reading...");
    // TODO : buat dua sistem, jika pake simulation gazebo dan jika pake robot real
    // if (start() != hardware_interface::CallbackReturn::SUCCESS) {
    if (!serial_port_ || !serial_port_->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("MappingbotHardware"),
                     "Serial port not open during read()");
        return hardware_interface::return_type::ERROR;
    }

    serial_port_->read_frames();

    velocity_states_[0] = 1 * (abs((double)serial_port_->velL)); // data in rpm unit
    velocity_states_[1] = 1 * (abs((double)serial_port_->velR)); // data in rpm unit

    // position_states_[0] = serial_port_->wheelL_hall;
    // position_states_[1] = serial_port_->wheelR_hall;

    on_encoder_update (serial_port_->wheelL_hall, serial_port_->wheelR_hall);

    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Posisi Kiri: %f", position_states_[0]);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Posisi Kanan: %f", position_states_[1]);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Kecepatan Kiri: %f", velocity_states_[0]);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Kecepatan Kanan: %f", velocity_states_[1]);
    
    // fprintf(stderr, "velL : %f\n", velocity_states_[0]);
    // fprintf(stderr, "velR : %f\n", velocity_states_[1]);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MappingbotHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Writing...");   
    // if (start() != hardware_interface::CallbackReturn::SUCCESS) {
    if (!serial_port_ || !serial_port_->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("MappingbotHardware"),
                     "Serial port not open during write()");
        return hardware_interface::return_type::ERROR;
    }

    // Convert PID outputs in RAD/S to RPM
    double set_cmd_left = velocity_commands_[0] / 0.10472;
    double set_cmd_right = velocity_commands_[1] / 0.10472;

    // double set_cmd_left = 33.6;
    // double set_cmd_right = -33.6;
    // Calculate steering from difference of left and right
    // set_speed[0] = left wheel
    // set_speed[1] = right wheel
    // const double speed = (set_speed[0] + set_speed[1])/2.0;
    // const double steer = (set_speed[0] - speed)*2.0;

    // const double set_cmd_left = -210.0;
    // const double set_cmd_right = 210.0;
    
    serial_port_->write_frame(set_cmd_left, set_cmd_right);

    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Kecepatan Kiri r/s: %f", velocity_commands_[0]);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Kecepatan Kanan r/s: %f", velocity_commands_[1]);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Posisi x: %f", msg->pose.pose.position.x);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Posisi y: %f", msg->pose.pose.position.y);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Kecepatan Kiri rpm: %f", set_cmd_left);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Kecepatan Kanan rpm: %f", set_cmd_right);
    // RCLCPP_INFO(rclcpp::get_logger("MappingbotHardware"), "Motor successfully written!");
    return hardware_interface::return_type::OK;
}

void MappingbotHardware::on_encoder_update (int16_t right, int16_t left){
    double posL = 0.0, posR = 0.0;

    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < low_wrap && last_wheelcountR > high_wrap)
        multR++;
    else if (right > high_wrap && last_wheelcountR < low_wrap)
        multR--;
    posR = right + multR*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountR = right;

    if (left < low_wrap && last_wheelcountL > high_wrap)
        multL++;
    else if (left > high_wrap && last_wheelcountL < low_wrap)
        multL--;
    posL = left + multL*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountL = left;

    static double lastPosL = 0.0, lastPosR = 0.0;
    static double lastPubPosL = 0.0, lastPubPosR = 0.0;
    static bool nodeStartFlag = true;
    rclcpp::Clock clock;
    auto now = clock.now();

    if ((now - last_read).seconds() > 0.2
		&& std::abs(posL) < 5 && std::abs(posR) < 5) {
        lastPosL = posL;
        lastPosR = posR;
    }
    double posLDiff = 0;
    double posRDiff = 0;

    // If node is just starting, keep odom at zeros
    if (nodeStartFlag) {
        nodeStartFlag = false;
    } else {
        posLDiff = posL - lastPosL;
        posRDiff = posR - lastPosR;
    }

    lastPubPosL += posLDiff;
    lastPubPosR += posRDiff;
    lastPosL = posL;
    lastPosR = posR;

    // Convert position in accumulated ticks to position in radians
    position_states_[1] = 2.0*M_PI * lastPubPosL / (double)TICKS_PER_ROTATION;
    position_states_[0] = 2.0*M_PI * lastPubPosR / (double)TICKS_PER_ROTATION;
}

