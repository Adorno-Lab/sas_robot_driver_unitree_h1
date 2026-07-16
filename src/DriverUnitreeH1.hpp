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
#
# ################################################################
*/

#pragma once
#include <dqrobotics/DQ.h>
#include <memory>




using namespace DQ_robotics;
using namespace Eigen;

class DriverUnitreeH1
{
public:
    enum class HIGH_LEVEL_MODE{
        IDLE_DEFAULT_STAND, // 0. idle, default stand
        FORCED_STAND,        // 1. force stand (controlled by dBodyHeight + ypr)
        TARGET_VELOCITY_WALKING, // 2. target velocity walking (controlled by velocity + yawSpeed)
        PATH_MODE_WALKING,     // 4. path mode walking (reserve for future release)
        POSITION_STAND_DOWN,   // 5. position stand down.
        POSITION_STAND_UP,    // 6. position stand up
        DAMPING_MODE,         // 7. damping mode
        RECOVERY_STAND        // 9. recovery stand
    };

    enum class GAIT_TYPE{ //uint8_t gaitType;			   // 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
        IDLE,
        TROT,
        TROT_RUNNING,
        CLIMB_STAIR,
        TROT_OBSTACLE
    };

protected:
    std::atomic_bool* st_break_loops_;
private:
    enum class STATUS{
        IDLE,
        CONNECTED,
        INITIALIZED,
        DEINITIALIZED,
        DISCONNECTED,
    };
    STATUS current_status_{STATUS::IDLE};
    std::string status_msg_;


    const std::unordered_map<HIGH_LEVEL_MODE, uint8_t> high_level_mode_map_ =
        {
        {HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND,      0},
        {HIGH_LEVEL_MODE::FORCED_STAND,            1},
        {HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, 2},
        {HIGH_LEVEL_MODE::PATH_MODE_WALKING,       4},
        {HIGH_LEVEL_MODE::POSITION_STAND_DOWN,     5},
        {HIGH_LEVEL_MODE::POSITION_STAND_UP,       6},
        {HIGH_LEVEL_MODE::DAMPING_MODE,            7},
        {HIGH_LEVEL_MODE::RECOVERY_STAND,          9},
        };
    const std::unordered_map<uint8_t, HIGH_LEVEL_MODE> high_level_mode_map_inv_ =
        {
        {0, HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND     },
        {1, HIGH_LEVEL_MODE::FORCED_STAND           },
        {2, HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING},
        {4, HIGH_LEVEL_MODE::PATH_MODE_WALKING      },
        {5, HIGH_LEVEL_MODE::POSITION_STAND_DOWN    },
        {6, HIGH_LEVEL_MODE::POSITION_STAND_UP      },
        {7, HIGH_LEVEL_MODE::DAMPING_MODE           },
        {9, HIGH_LEVEL_MODE::RECOVERY_STAND         },
        };


     const std::unordered_map<uint8_t, GAIT_TYPE> gait_type_map_inv_ =
        {
        {0, GAIT_TYPE::IDLE},
        {1, GAIT_TYPE::TROT},
        {2, GAIT_TYPE::TROT_RUNNING},
        {3, GAIT_TYPE::CLIMB_STAIR},
        {3, GAIT_TYPE::TROT_OBSTACLE},
        };


    HIGH_LEVEL_MODE current_high_level_mode_; // This information cames from the Unitree SDK High State
    GAIT_TYPE current_gait_type_;// This information cames from the Unitree SDK High State

    HIGH_LEVEL_MODE target_high_level_mode_;
    bool mode_change_in_progress_;
    void _command_in_high_level_mode(const HIGH_LEVEL_MODE& high_level_mode,
                                     const double& forward_vel,
                                     const double& side_vel,
                                     const double& yaw_speed,
                                     const double& roll_angle = 0,
                                     const double& pitch_angle = 0,
                                     const double& yaw_angle = 0,
                                     const double& body_height = 0);



    void _finish_high_level_motion();
    void _stop_robot_in_high_level_motion();
    void _command_robot_in_high_level_motion();
    void _check_high_level_mode_request();
    void _prepare_the_robot_for_high_level_motion();
    bool robot_is_prepared_for_high_level_motion_{false};


//void _set_high_level_mode(const HIGH_LEVEL_MODE& high_level_mode);

public:
    enum class MODE{
        None,
        PositionControl,
        VelocityControl,
        ForceControl,
    };

    enum class LEVEL{HIGH, LOW};
    LEVEL level_;
    enum class BRANCH{FR, FL, RR, RL};


public:
    enum class CUSTOM_FLAGS
    {
        FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO,
    };
    std::vector<CUSTOM_FLAGS> custom_flags_;

private:

    class Impl;
    std::shared_ptr<Impl> impl_;




    double target_high_level_forward_speed_{0};
    double target_high_level_side_speed_{0};
    double target_high_level_yaw_speed_{0};

    double target_high_level_roll_angle_{0};
    double target_high_level_pitch_angle_{0};
    double target_high_level_yaw_angle_{0};
    double target_high_level_bodyheight_{0}; //delta

    std::string ip_ {"0.0.0"};
    int port_{0};

    void _show_status();
    bool verbosity_;
    int timeout_in_milliseconds_;
    bool LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_;

    MODE mode_{MODE::None};

    float dt_{0.002};
    unsigned long long motiontime_{0};
    unsigned long long frozen_time_in_request_check_{0};
    bool frozen_time_in_request_check_was_set_{false};
    uint32_t tick_{0}; //real-time from motion controller

    unsigned long long frozen_time_high_level_stop_motion_{0};
    bool frozen_time_high_level_stop_motion_was_set_{false};

    unsigned long long frozen_time_high_level_motion_preparation_{0};
    bool frozen_time_high_level_motion_preparation_was_set_{false};

    int state_of_charge_{0}; // Battery status (0-100%)

    bool communication_established_{false};
    std::vector<unsigned long long> upd_status_{0,0,0,0,0,0,0};


    //std::atomic<bool> finish_control_loop_{false};
    //std::atomic<bool> finish_echo_robot_state_{false};

    std::atomic<bool> finish_motion_to_deinitialize_{false};
    std::atomic<bool> the_robot_is_ready_to_deinitialize_{false};

    DQ last_IMU_orientation_when_robot_stopped_{1};

    DQ IMU_orientation_{1};
    DQ IMU_gyroscope_{0};
    DQ IMU_accelerometer_{0};
    Vector3d IMU_rpy_ = Vector3d::Zero();

    DQ odometry_position_{0};
    double body_height_{0};

    DQ high_level_linear_velocity_{0};
    DQ high_level_angular_velocity_{0};


    void _update_data_from_robot_state();

    template<typename T>
    void _update_joint_data(const T& state);

    template<typename T>
    void _update_IMU_data(const T& state);

    template<typename T>
    void _update_battery_data(const T& state);


    //-------------------------------------------------------------
    //---------------Robot state attributes------------------------
    //----joint positions--(unit: radian)
    VectorXd qFR_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd qFL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRR_ = (VectorXd(3) << 0,0,0).finished();

    //----joint velocities--(unit: radian/second)
    VectorXd qFR_dot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qFL_dot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRL_dot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRR_dot_  = (VectorXd(3) << 0,0,0).finished();

    //----joint accelerations-- (unit: radian/second^2)
    VectorXd qFR_ddot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qFL_ddot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRL_ddot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRR_ddot_  = (VectorXd(3) << 0,0,0).finished();

    //----estimated output joint torques (unit: N.m)
    VectorXd tauFR_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd tauFL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd tauRL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd tauRR_ = (VectorXd(3) << 0,0,0).finished();

    //----motor temperatures
    VectorXd temperatureFR_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd temperatureFL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd temperatureRL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd temperatureRR_ = (VectorXd(3) << 0,0,0).finished();
    //--------------------------------------------------------------
    //--------------------------------------------------------------
    double speed_threshold_to_force_stand_mode_ = 0.15;



    void _robot_control();
    void _robot_update();
    void _UDPRecv();
    void _UDPSend();

    void _update_udp_status();

    void _set_driver_mode(const MODE& mode, const LEVEL& level);
    void _initialize_high_cmd_variable();


    static bool are_approximately_equal(const double& a, const double& b, const double& epsilon = 1e-6) ;
    bool status_velocities_{false};



public:
    DriverUnitreeH1() = delete;
    DriverUnitreeH1(const DriverUnitreeH1&) = delete;
    DriverUnitreeH1& operator= (const DriverUnitreeH1&) = delete;
    DriverUnitreeH1(std::atomic_bool* st_break_loops,
                    const MODE& mode = MODE::None,
                    const LEVEL& level = LEVEL::HIGH,
                    const bool& verbosity = true,
                    const int& TIMEOUT_IN_MILLISECONDS = 2000,
                    const bool& LIE_DOWN_ROBOT_WHEN_DEINITIALIZE = true,
                    const std::string &TARGET_IP = "192.168.123.220", // For low-level use "192.168.123.10",
                    const int& TARGET_PORT = 8082,              //For low-level use 8007
                    const int& LOCAL_PORT = 8090,
                    const std::vector<CUSTOM_FLAGS>& custom_flags = std::vector<CUSTOM_FLAGS>{});

    std::string get_target_ip() const;
    int get_target_port() const;
    int get_motiontime() const;
    uint32_t get_realtime_controller() const;
    int get_state_of_charge() const;
    std::string get_status_message() const;

    std::vector<unsigned long long> get_udp_status();
    bool get_connection_status();

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    std::tuple<VectorXd, VectorXd, VectorXd, VectorXd> get_leg_joint_positions() const;

    VectorXd get_joint_positions(const BRANCH& branch) const;
    VectorXd get_joint_velocities(const BRANCH& branch) const;
    VectorXd get_joint_accelerations(const BRANCH& branch) const;
    VectorXd get_joint_estimated_torques(const BRANCH& branch) const;
    VectorXd get_joint_temperatures(const BRANCH& branch) const;

    DQ get_IMU_orientation() const;
    DQ get_last_IMU_orientation_when_robot_stopped() const;
    Vector3d get_IMU_rpy_angles() const;
    DQ get_IMU_gyroscope() const;
    DQ get_IMU_accelerometer() const;
    DQ get_IMU_pose() const;
    VectorXd get_mobile_platform_configuration_from_IMU_pose() const;
    DQ get_high_level_angular_velocity() const;
    DQ get_high_level_linear_velocity() const;

    DQ get_odometry_position() const;
    double get_body_height() const;



    void set_high_level_forward_speed(const double& forward_speed = 0);
    void set_high_level_yaw_speed(const double& yaw_speed = 0);
    void set_high_level_forward_and_yaw_speed(const double& forward_speed = 0,
                                              const double& yaw_speed = 0);

    void set_high_level_speed(const double& forward_speed = 0,
                              const double& side_speed = 0,
                              const double& yaw_speed = 0);

    void set_forced_stand_commands(const double& roll_angle=0,
                                   const double& pitch_angle=0,
                                   const double& yaw_angle=0,
                                   const double& bodyheight=0);


    double get_high_level_forward_speed_reference() const;
    double get_high_level_yaw_speed_reference() const;

    void show_high_mode() const;
    unsigned long long get_motion_time() const;

    void request_change_in_high_level_control(const HIGH_LEVEL_MODE& mode);

    HIGH_LEVEL_MODE get_current_high_mode() const;
    HIGH_LEVEL_MODE get_target_high_mode() const;
    GAIT_TYPE get_current_gait_type() const;

    std::string high_level_mode_to_string(const HIGH_LEVEL_MODE& mode) const;
    std::string gait_type_to_string(const GAIT_TYPE& gait_type) const;




};


