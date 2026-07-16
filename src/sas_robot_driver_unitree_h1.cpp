/*
# (C) Copyright 2024-2026 Adorno-Lab software developments
#
#    This file is part of sas_robot_driver_unitree_h1.
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#   Based on https://ros2-tutorial.readthedocs.io/en/latest/cpp/cpp_node.html
#
# ################################################################*/

#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1.hpp>

#include "DriverUnitreeH1.hpp"
#include <iostream>
#include <memory>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>


//using std::placeholders::_1;

namespace sas
{

class RobotDriverUnitreeH1::Impl
{

public:
   std::shared_ptr<DriverUnitreeH1> unitree_h1_driver_;
   Impl()
   {

   };



};


void RobotDriverUnitreeH1::_initial_settings()
{
    impl_ = std::make_unique<RobotDriverUnitreeH1::Impl>();

    DriverUnitreeH1::MODE mode;
    if (configuration_.mode == "VelocityControl")
    {
        mode = DriverUnitreeH1::MODE::VelocityControl;
    }else{
        mode = DriverUnitreeH1::MODE::None;
    }

    std::vector<DriverUnitreeH1::CUSTOM_FLAGS> custom_flags;
    if (configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO)
        custom_flags.push_back(DriverUnitreeH1::CUSTOM_FLAGS::FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO);


    // I need to use the parameters of the configuration structure!
    impl_->unitree_h1_driver_ = std::make_shared<DriverUnitreeH1>(break_loops_,
                                                                  mode, // Driver mode
                                                                  DriverUnitreeH1::LEVEL::HIGH,       // Level mode
                                                                  true,   //verbosity
                                                                  2000,   // TIMEOUT in ms
                                                                  configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE, // LIE DOWN ROBOT WHEN DEINITIALIZE
                                                                  configuration_.ROBOT_IP,  // Target IP   //192.168.123.10 for low-level mode
                                                                  configuration_.ROBOT_PORT,// Target port  //8007 for low-level mode
                                                                  8090,
                                                                  custom_flags);

    // For backward compatibility
    publisher_FR_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/FR_joint_states",1);
    publisher_FL_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/FL_joint_states",1);
    publisher_RR_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/RR_joint_states",1);
    publisher_RL_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/RL_joint_states",1);

    publisher_rpy_angles_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix_ + "/get/rpy_angles", 1);
    publisher_IMU_state_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_prefix_ + "/get/IMU_state", 1);
    publisher_pose_state_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix_ + "/get/pose_state", 1);
    publisher_high_level_velocities_state_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_prefix_ + "/get/twist_state", 1);

    publisher_battery_state_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(topic_prefix_ + "/get/battery_state", 1);

    publisher_IMU_orientation_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix_ + "/get/imu_orientation",1);
    publisher_last_IMU_orientation_when_robot_stopped_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix_ + "/get/last_imu_orientation_when_robot_stopped",1);

    subscriber_target_holonomic_velocities_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_prefix_ + "/set/holonomic_target_velocities",
        1,
        std::bind(&RobotDriverUnitreeH1::_callback_target_holonomic_velocities, this, std::placeholders::_1)
        );

    subscriber_target_twist_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic_prefix_ + "/set/target_twist",
        1,
        std::bind(&RobotDriverUnitreeH1::_callback_target_twist, this, std::placeholders::_1)
        );

    /*
    subscriber_target_twist_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic_prefix_ + "/set/target_twist",
        1,
        std::bind(&RobotDriverUnitreeH1::_callback_target_twist, this, std::placeholders::_1)
        );

    subscriber_stand_commands_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_prefix_ + "/set/stand_commands",
        1,
        std::bind(&RobotDriverUnitreeH1::_callback_stand_commands, this, std::placeholders::_1)
        );

    subscriber_mode_switch_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        topic_prefix_ + "/set/mode",
        1,
        std::bind(&RobotDriverUnitreeH1::_callback_mode_switch, this, std::placeholders::_1)
        );
*/


/*
    subscriber_shutdown_signal_ = node_->create_subscription<sas_msgs::msg::Bool>(
        topic_prefix_ + "/set/shutdown", 1,
        std::bind(&RobotDriverUnitreeH1::_callback_shutdown_signal_,  this, std::placeholders::_1)
        );
*/

    subscriber_emergency_stop_device_signal_ = node_->create_subscription<sas_msgs::msg::Bool>(
        "/sas/set/shutdown", 1,
        std::bind(&RobotDriverUnitreeH1::_callback_emergency_stop_device_signal,  this, std::placeholders::_1)
        );

    // set the callback here
    set_control_loop_callback([this]() {

        _read_joint_states_and_publish();
        _read_imu_state_and_publish();
        _read_battery_state();
        _read_twist_state_and_publish();
        _set_target_velocities_from_subscriber();
        _read_rpy_angles_state_and_publish();

        if (shutdown_signal_)
        {
            throw std::runtime_error("The shutdown signal was received!");
            *st_break_loops_ = true; // Signal shutdown
        }
    });
}

RobotDriverUnitreeH1::RobotDriverUnitreeH1(std::shared_ptr<Node> &node,
                                           const RobotDriverUnitreeH1Configuration &configuration,
                                           std::atomic_bool *break_loops)
    :LeggedRobotDriver{break_loops},
    st_break_loops_{break_loops},
    topic_prefix_{configuration.robot_name},
    configuration_{configuration},
    node_{node},
    timer_period_{0.002},
    print_count_{0},
    clock_{0.002},
    shutdown_signal_{false}
{
    _initial_settings();
}

RobotDriverUnitreeH1::RobotDriverUnitreeH1(std::shared_ptr<Node> &node,
                                           const RobotDriverUnitreeH1Configuration &configuration,
                                           const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :LeggedRobotDriver{shutdown_signaler},
    topic_prefix_{configuration.robot_name},
    configuration_{configuration},
    node_{node},
    timer_period_{0.002},
    print_count_{0},
    clock_{0.002},
    shutdown_signal_{false}
{
    _initial_settings();
}


/**
 * @brief Get the joint positions of all four legs of the Unitree H1 robot
 *
 * Retrieves the current joint angles for all 12 degrees of freedom (3 per leg)
 * and concatenates them into a single state vector.
 *
 * @return VectorXd A 12-element vector with joint positions in the format:
 *         [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
 *          RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]
 */
VectorXd RobotDriverUnitreeH1::get_joint_positions()
{
    const VectorXd qFR = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::FR);
    const VectorXd qFL = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::FL);
    const VectorXd qRR = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::RR);
    const VectorXd qRL = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::RL);
    VectorXd qlegs = VectorXd(qFR.size() + qFL.size() + qRR.size() + qRL.size());
    qlegs << qFR, qFL, qRR, qRL;
    return qlegs;
}

/**
 * @brief Get the joint velocities of all four legs of the Unitree H1 robot
 *
 * Retrieves the current joint angular velocities for all 12 degrees of freedom (3 per leg)
 * and concatenates them into a single state vector.
 *
 * @return VectorXd A 12-element vector with joint velocities in the format:
 *         [FR_hip_dot, FR_thigh_dot, FR_calf_dot, FL_hip_dot, FL_thigh_dot, FL_calf_dot,
 *          RR_hip_dot, RR_thigh_dot, RR_calf_dot, RL_hip_dot, RL_thigh_dot, RL_calf_dot]
 */
VectorXd RobotDriverUnitreeH1::get_joint_velocities()
{
    const VectorXd qFR_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::FR);
    const VectorXd qFL_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::FL);
    const VectorXd qRR_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::RR);
    const VectorXd qRL_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::RL);
    VectorXd qlegs_dot = VectorXd(qFR_dot.size() + qFL_dot.size() + qRR_dot.size() + qRL_dot.size());
    qlegs_dot << qFR_dot, qFL_dot, qRR_dot, qRL_dot;
    return qlegs_dot;
}

/**
 * @brief Get the estimated joint torques of all four legs of the Unitree H1 robot
 *
 * Retrieves the current estimated joint torques for all 12 degrees of freedom (3 per leg)
 * and concatenates them into a single state vector.
 *
 * @return VectorXd A 12-element vector with joint torques in the format:
 *         [tFR_hip, tFR_thigh, tFR_calf, tFL_hip, tFL_thigh, tFL_calf,
 *          tRR_hip, tRR_thigh, tRR_calf, tRL_hip, tRL_thigh, tRL_calf]
 */
VectorXd RobotDriverUnitreeH1::get_joint_torques()
{
    const VectorXd tFR = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::FR);
    const VectorXd tFL = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::FL);
    const VectorXd tRR = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::RR);
    const VectorXd tRL = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::RL);
    VectorXd tlegs = VectorXd(tFR.size() + tFL.size() + tRR.size() + tRL.size());
    tlegs << tFR, tFL, tRR, tRL;
    return tlegs;
}

void RobotDriverUnitreeH1::set_target_joint_positions([[maybe_unused]] const VectorXd &desired_joint_positions_rad)
{
    throw std::runtime_error("RobotDriverUnitreeH1::set_target_joint_positions Not implemented");
}


void RobotDriverUnitreeH1::connect()
{
    impl_->unitree_h1_driver_->connect();
}

void RobotDriverUnitreeH1::disconnect()
{
    impl_->unitree_h1_driver_->disconnect();
}

void RobotDriverUnitreeH1::initialize()
{
    impl_->unitree_h1_driver_->initialize();
}

void RobotDriverUnitreeH1::deinitialize()
{
    impl_->unitree_h1_driver_->deinitialize();
}

void RobotDriverUnitreeH1::set_target_twist(const DQ &twist)
{
    const VectorXd twist_vec = twist.vec6();
    //    0  1  2  3  4  5
    //   wx wy wz vx vy vz
    double vx = twist_vec(3);
    double vy = twist_vec(4);
    double wz = twist_vec(2);
    impl_->unitree_h1_driver_->set_high_level_speed(vx,vy,wz);
}

void RobotDriverUnitreeH1::set_target_base_orientation([[maybe_unused]] const DQ &r)
{
    throw std::runtime_error("RobotDriverUnitreeH1::set_target_base_orientation() Not implemented");
}

void RobotDriverUnitreeH1::set_target_base_height([[maybe_unused]] const double &base_height)
{
    throw std::runtime_error("RobotDriverUnitreeH1::set_target_base_height Not implemented");
}


void RobotDriverUnitreeH1::_read_joint_states_and_publish()
{
    sensor_msgs::msg::JointState ros_msg_FR;
    sensor_msgs::msg::JointState ros_msg_FL;
    sensor_msgs::msg::JointState ros_msg_RR;
    sensor_msgs::msg::JointState ros_msg_RL;

    ros_msg_FR.header.stamp =  node_->get_clock()->now();
    ros_msg_FL.header.stamp =  node_->get_clock()->now();
    ros_msg_RR.header.stamp =  node_->get_clock()->now();
    ros_msg_RL.header.stamp =  node_->get_clock()->now();

    VectorXd qFR     = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::FR);
    VectorXd qFR_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::FR);
    VectorXd qFR_tau = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::FR);

    VectorXd qFL     = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::FL);
    VectorXd qFL_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::FL);
    VectorXd qFL_tau = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::FL);

    VectorXd qRR     = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::RR);
    VectorXd qRR_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::RR);
    VectorXd qRR_tau = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::RR);

    VectorXd qRL     = impl_->unitree_h1_driver_->get_joint_positions(DriverUnitreeH1::BRANCH::RL);
    VectorXd qRL_dot = impl_->unitree_h1_driver_->get_joint_velocities(DriverUnitreeH1::BRANCH::RL);
    VectorXd qRL_tau = impl_->unitree_h1_driver_->get_joint_estimated_torques(DriverUnitreeH1::BRANCH::RL);

    if (qFR.size() > 0)
        ros_msg_FR.position = vectorxd_to_std_vector_double(qFR);
    if (qFR_dot.size() > 0)
        ros_msg_FR.velocity = vectorxd_to_std_vector_double(qFR_dot);
    if (qFR_tau.size() > 0)
        ros_msg_FR.effort = vectorxd_to_std_vector_double(qFR_tau);

    if (qFL.size() > 0)
        ros_msg_FL.position = vectorxd_to_std_vector_double(qFL);
    if (qFL_dot.size() > 0)
        ros_msg_FL.velocity = vectorxd_to_std_vector_double(qFL_dot);
    if (qFL_tau.size() > 0)
        ros_msg_FL.effort = vectorxd_to_std_vector_double(qFL_tau);

    if (qRR.size() > 0)
        ros_msg_RR.position = vectorxd_to_std_vector_double(qRR);
    if (qRR_dot.size() > 0)
        ros_msg_RR.velocity = vectorxd_to_std_vector_double(qRR_dot);
    if (qRR_tau.size() > 0)
        ros_msg_RR.effort = vectorxd_to_std_vector_double(qRR_tau);


    if (qRL.size() > 0)
        ros_msg_RL.position = vectorxd_to_std_vector_double(qRL);
    if (qRL_dot.size() > 0)
        ros_msg_RL.velocity = vectorxd_to_std_vector_double(qRL_dot);
    if (qRL_tau.size() > 0)
        ros_msg_RL.effort = vectorxd_to_std_vector_double(qRL_tau);

    publisher_FR_joint_states_->publish(ros_msg_FR);
    publisher_FL_joint_states_->publish(ros_msg_FL);
    publisher_RR_joint_states_->publish(ros_msg_RR);
    publisher_RL_joint_states_->publish(ros_msg_RL);

}

void RobotDriverUnitreeH1::_read_imu_state_and_publish()
{
    sensor_msgs::msg::Imu ros_msg_imu;
    ros_msg_imu.header.stamp = node_->get_clock()->now();

    geometry_msgs::msg::PoseStamped ros_msg_pose;
    ros_msg_pose.header.stamp = node_->get_clock()->now();


    DQ rIMU_stopped = impl_->unitree_h1_driver_->get_last_IMU_orientation_when_robot_stopped();

    DQ orientation = impl_->unitree_h1_driver_->get_IMU_orientation();
    VectorXd vec_orientation = orientation.vec4();
    ros_msg_imu.orientation.w = vec_orientation(0);
    ros_msg_imu.orientation.x = vec_orientation(1);
    ros_msg_imu.orientation.y = vec_orientation(2);
    ros_msg_imu.orientation.z = vec_orientation(3);

    VectorXd vec_angular_velocity = impl_->unitree_h1_driver_->get_IMU_gyroscope().vec3();
    ros_msg_imu.angular_velocity.x = vec_angular_velocity(0);
    ros_msg_imu.angular_velocity.y = vec_angular_velocity(1);
    ros_msg_imu.angular_velocity.z = vec_angular_velocity(2);

    VectorXd vec_acceleration = impl_->unitree_h1_driver_->get_IMU_accelerometer().vec3();
    ros_msg_imu.linear_acceleration.x = vec_acceleration(0);
    ros_msg_imu.linear_acceleration.y = vec_acceleration(1);
    ros_msg_imu.linear_acceleration.z = vec_acceleration(2);

    VectorXd vec_position = impl_->unitree_h1_driver_->get_IMU_pose().translation().vec3();
    ros_msg_pose.pose.position.x = vec_position(0);
    ros_msg_pose.pose.position.y = vec_position(1);
    ros_msg_pose.pose.position.z = vec_position(2);

    ros_msg_pose.pose.orientation.w = vec_orientation(0);
    ros_msg_pose.pose.orientation.x = vec_orientation(1);
    ros_msg_pose.pose.orientation.y = vec_orientation(2);
    ros_msg_pose.pose.orientation.z = vec_orientation(3);

    publisher_IMU_state_->publish(ros_msg_imu);
    publisher_pose_state_->publish(ros_msg_pose);


    publisher_IMU_orientation_->publish(sas::dq_to_geometry_msgs_pose_stamped(orientation));

    publisher_last_IMU_orientation_when_robot_stopped_->publish(sas::dq_to_geometry_msgs_pose_stamped(rIMU_stopped));
}

void RobotDriverUnitreeH1::_read_twist_state_and_publish()
{
    geometry_msgs::msg::TwistStamped ros_msg_twist;
    ros_msg_twist.header.stamp = node_->get_clock()->now();

    VectorXd vec_angular_velocity = impl_->unitree_h1_driver_->get_high_level_angular_velocity().vec3();
    ros_msg_twist.twist.angular.x = vec_angular_velocity(0);
    ros_msg_twist.twist.angular.y = vec_angular_velocity(1);
    ros_msg_twist.twist.angular.z = vec_angular_velocity(2);

    VectorXd vec_acceleration = impl_->unitree_h1_driver_->get_high_level_linear_velocity().vec3();
    ros_msg_twist.twist.linear.x = vec_acceleration(0);
    ros_msg_twist.twist.linear.y = vec_acceleration(1);
    ros_msg_twist.twist.linear.z = vec_acceleration(2);

    publisher_high_level_velocities_state_->publish(ros_msg_twist);

}

void RobotDriverUnitreeH1::_read_rpy_angles_state_and_publish()
{
    std_msgs::msg::Float64MultiArray msg;
    Eigen::Vector3d rpy_angles = impl_->unitree_h1_driver_->get_IMU_rpy_angles();

    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "rpy";
    msg.layout.dim[0].size = 3;
    msg.layout.dim[0].stride = 3;


    msg.data.clear();
    msg.data.reserve(3);
    msg.data.push_back(rpy_angles.x());
    msg.data.push_back(rpy_angles.y());
    msg.data.push_back(rpy_angles.z());

    // Publish the message
    publisher_rpy_angles_->publish(msg);
}

void RobotDriverUnitreeH1::_read_battery_state()
{
    sensor_msgs::msg::BatteryState ros_msg_battery;
    ros_msg_battery.header.stamp = node_->get_clock()->now();
    int battery_level = impl_->unitree_h1_driver_->get_state_of_charge();

    ros_msg_battery.percentage = 0.01*battery_level;
    publisher_battery_state_->publish(ros_msg_battery);
}



bool RobotDriverUnitreeH1::_should_shutdown() const
{
    return (*st_break_loops_);
}



void RobotDriverUnitreeH1::_set_target_velocities_from_subscriber()
{
    if (new_target_twist_available_)
    {
        //    0  1  2  3  4  5
        //   wx wy wz vx vy vz
        double vx = target_twist_(3);
        double vy = target_twist_(4);
        double wz = target_twist_(2);

        impl_->unitree_h1_driver_->set_high_level_speed(vx,vy,wz);

        new_target_twist_available_ = false;
    }
}


/*
void RobotDriverUnitreeH1::_set_target_stand_commands_from_subscriber()
{
    if (new_stand_commands_available_)
    {
        impl_->unitree_h1_driver_->set_forced_stand_commands(target_stand_commands_(0),
                                                             target_stand_commands_(1),
                                                             target_stand_commands_(2),
                                                             target_stand_commands_(3));
        new_stand_commands_available_ = false;
    }
}
*/


/*

void RobotDriverUnitreeH1::control_loop()
{
    try{

        clock_.init();
        impl_->unitree_h1_driver_->connect();
        impl_->unitree_h1_driver_->initialize();



        while(!_should_shutdown())
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);

            if (shutdown_signal_)
            {
                throw std::runtime_error("The shutdown signal was received!");
                *st_break_loops_ = true; // Signal shutdown
            }

            _read_joint_states_and_publish();
            _read_imu_state_and_publish();
            _read_battery_state();
            _read_twist_state_and_publish();
            _set_target_velocities_from_subscriber();
            _set_target_stand_commands_from_subscriber();
            _read_rpy_angles_state_and_publish();


            if (is_watchdog_enabled())
            {
                if (!watchdog_started_)
                {   // This portion of code is executed only one time
                    // Initialize the watchdog.
                    //double watchdog_period;
                    //double watchdog_maximum_acceptable_delay;

                    // If the "watchdog_period_in_seconds" is not defined, we use a default value.
                    //get_ros_optional_parameter(node_, "watchdog_period_in_seconds", watchdog_period, 1.0);

                    //RCLCPP_INFO_STREAM(node_->get_logger(), "::Watchdog initialized with a " << watchdog_period << " second period");
                    RCLCPP_INFO_STREAM(node_->get_logger(), "::Watchdog initialized with a " << watchdog_period_in_seconds_  << " second period");
                    // If the elapsed time between the triggers is higher than the watchdog period, an exception is thrown


                    // If the "watchdog_maximum_acceptable_delay" is not defined, we use a default value.
                    //get_ros_optional_parameter(node_, "watchdog_maximum_acceptable_delay", watchdog_maximum_acceptable_delay_, 1.0);
                    RCLCPP_INFO_STREAM(node_->get_logger(), "::Watchdog initialized with a maximum acceptable delay of " << watchdog_maximum_acceptable_delay_in_seconds_<< " seconds");
                    // If the time difference between the time point of signal that was sent (using the client computer's clock) and the time point
                    // when the watchdog signal was received (using the computer's clock on which the server is running) is higher than the watchdog_maximum_acceptable_delay,
                    // an exception is thrown by the robot driver.


                    const std::chrono::nanoseconds period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(watchdog_period_in_seconds_));
                    watchdog_started_ = true;
                    _watchdog_set_maximum_acceptable_delay(watchdog_maximum_acceptable_delay_in_seconds_);

                    //-----------------------------------------------------------------------------------------/
                    _watchdog_start(period);
                    //--- For developers: Do not put more code after this point---//
                }
            }

            rclcpp::spin_some(node_);
        }
        impl_->unitree_h1_driver_->set_high_level_speed(0,0,0);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::Exception caught::" << e.what());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::Unexpected error or exception caught");
    }
}
*/



void RobotDriverUnitreeH1::_callback_target_holonomic_velocities(const std_msgs::msg::Float64MultiArray &msg)
{
    VectorXd target_holonomic_velocities  = std_vector_double_to_vectorxd(msg.data);
    new_target_velocities_available_ = true;
    new_target_twist_available_ = true;
    target_twist_ <<0,                                // Wx
                    0,                                // Wy
                    target_holonomic_velocities(2),   // Wz
                    target_holonomic_velocities(0),   // Vx
                    target_holonomic_velocities(1),   // Vy
                    0;                                // Vz
}




void RobotDriverUnitreeH1::_callback_target_twist(const geometry_msgs::msg::TwistStamped& msg)
{
    target_twist_ <<msg.twist.angular.x,
                    msg.twist.angular.y,
                    msg.twist.angular.z,
                    msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z;

    new_target_twist_available_ = true;
}


/*
void RobotDriverUnitreeH1::_callback_stand_commands(const std_msgs::msg::Float64MultiArray &msg)
{
    target_stand_commands_  = std_vector_double_to_vectorxd(msg.data);
    new_stand_commands_available_ = true;
}
*/

/*
void RobotDriverUnitreeH1::_callback_mode_switch(const std_msgs::msg::Int32MultiArray& msg)
{
    if (msg.data.size() >= 1)
    {
        auto target_mode = static_cast<DriverUnitreeH1::HIGH_LEVEL_MODE>(msg.data[0]);
        auto current_target_mode = impl_->unitree_h1_driver_->get_target_high_mode();

        // Only request if mode is different
        if (target_mode != current_target_mode)
        {
            impl_->unitree_h1_driver_->request_change_in_high_level_control(target_mode);

            RCLCPP_INFO(node_->get_logger(), "Mode switched to: %s",
                        impl_->unitree_h1_driver_->high_level_mode_to_string(target_mode).c_str());
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(), "Ignoring mode switch to same mode: %s",
                         impl_->unitree_h1_driver_->high_level_mode_to_string(target_mode).c_str());
        }
    }
}

*/

/*
void RobotDriverUnitreeH1::_callback_shutdown_signal_(const sas_msgs::msg::Bool &msg)
{
    // Only update this member if it was never set to true.
    // In other words, the driver is shut down if at least one received message is true.
    if (shutdown_signal_ == false)
        shutdown_signal_ = msg.data;
}
*/

void RobotDriverUnitreeH1::_callback_emergency_stop_device_signal(const sas_msgs::msg::Bool& msg)
{
    // Only update this member if it was never set to true.
    // In other words, the driver is shut down if at least one received message is true.
    if (shutdown_signal_ == false)
        shutdown_signal_ = msg.data;
}






RobotDriverUnitreeH1::~RobotDriverUnitreeH1()
{
    *st_break_loops_ = true;
    impl_->unitree_h1_driver_->deinitialize();
}

}
