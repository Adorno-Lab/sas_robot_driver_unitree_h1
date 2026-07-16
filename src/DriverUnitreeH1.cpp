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
#   Contributor: Daniel S. J. Derwent, email: daniel.derwent@manchester.ac.uk
#
# ################################################################*/

#include "DriverUnitreeH1.hpp"
#include <unitree_legged_sdk/unitree_legged_sdk.h>


bool flag_in_custom_flags(const DriverUnitreeH1::CUSTOM_FLAGS& flag,
                          const std::vector<DriverUnitreeH1::CUSTOM_FLAGS>& flags)
{
    return std::count(flags.begin(), flags.end(), flag) > 0;
}

class DriverUnitreeH1::Impl
{
public:
    std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safe_;
    std::shared_ptr<UNITREE_LEGGED_SDK::UDP>    udp_;
    UNITREE_LEGGED_SDK::LowCmd low_cmd_;
    UNITREE_LEGGED_SDK::LowState low_state_;

    UNITREE_LEGGED_SDK::HighCmd high_cmd_;
    UNITREE_LEGGED_SDK::HighState high_state_;

    // TODO: Update this to use the indices for the H1. Consider a broader re-work because its hard to see where
    // the waist joint would go in this system.
    std::vector<int> LA_index_ = {UNITREE_LEGGED_SDK::LA_0, UNITREE_LEGGED_SDK::LA_1, UNITREE_LEGGED_SDK::LA_2};
    std::vector<int> RA_index_ = {UNITREE_LEGGED_SDK::RA_0, UNITREE_LEGGED_SDK::RA_1, UNITREE_LEGGED_SDK::RA_2};
    std::vector<int> LL_index_ = {UNITREE_LEGGED_SDK::LL_0, UNITREE_LEGGED_SDK::LL_1, UNITREE_LEGGED_SDK::LL_2};
    std::vector<int> RL_index_ = {UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2};

    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_control_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_echo_state_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSend_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpRecv_;
    Impl()
    {

    };
};


/**
 * @brief DriverUnitreeH1::DriverUnitreeH1 constructor of the class
 * @param st_break_loops Use this flag to break internal loops. This is useful to stop the robot using a signal
 *              interruption by the user.
 * @param mode  The operation mode. Select the control strategy to command the robot.
 * @param level The control level to be used. HIGH or LOW.
 *              The HIGH level is used to send task space commands
 *              to the robot. The robot constraints are handled by the own robot  using a Unitree internal controller.
 *              The LOW mode is used to send joint position, velocity or torque commands. In this case, you must take into account
 *              all constraints in your controler (joint limits, control input limits, robot balance, self-collision avoidance, etc).
 * @param verbosity Use true (default) to display more information in the terminal.
 * @param TIMEOUT_IN_MILLISECONDS The max time in milliseconds to establish the communication to the robot before to throw an exception.
 * @param LIE_DOWN_ROBOT_WHEN_DEINITIALIZE In the B1 driver, this flag is used to put the robot on the ground when the driver is deinitialized.
 *                                         there is not currently an equivalent for the H1, so this flag is deprecated for now.
 * @param TARGET_IP The IP address of the H1 robot to perform the communication. You can use a
 *                  LAN cable connection or a WiFI network to establish the communication.
 *
 * @param TARGET_PORT The default port of the H1 robot.
 * @param LOCAL_PORT The communication port of the PC that is running the code.
 * @param custom_flags Additional flags to modify the robot behavior.
 *
 *
 *
 *
 * Example:
 *
 *          // This class follows the SmartArmStack driver principles, in which four methods are required to
 *          // start and finish the robot communication.
 *
            DriverUnitreeH1 H1(&kill_this_process,
                            DriverUnitreeH1::MODE::VelocityControl, // Driver mode
                            DriverUnitreeH1::LEVEL::HIGH,       // Level mode
                            true,   //verbosity
                            2000,   // TIMEOUT in ms
                            "192.168.123.220",  // Target IP   //192.168.123.10 for low-level mode
                            8082,              // Target port  //8007 for low-level mode
                            8090);             // Local port


            H1.connect();     // First method to be called.
            H1.initialize();  // Second method to be called. It is required to connect before to initialize.

            //Your code here
            H1.set_high_level_speed(-0.03, 0, 0.0);  //0.03 is the minimum value in forward speed.
            //

            H1.deinitialize();  // Third method to be called.
            H1.disconnect();    // Fourth method to be called. It is required to deinitialize before to disconnect.

 */

 // Modified to not take LIE_DOWN_ROBOT_WHEN_DEINITIALIZE and always set it as false until we figure out an H1 
 // alternative to this parameter
DriverUnitreeH1::DriverUnitreeH1(std::atomic_bool *st_break_loops,
                                           const MODE &mode,
                                           const LEVEL &level,
                                           const bool &verbosity,
                                           const int &TIMEOUT_IN_MILLISECONDS,
                                        //    const bool &LIE_DOWN_ROBOT_WHEN_DEINITIALIZE,
                                           const string &TARGET_IP,
                                           const int &TARGET_PORT,
                                           const int &LOCAL_PORT,
                                           const std::vector<CUSTOM_FLAGS>& custom_flags):
    st_break_loops_{st_break_loops},
    target_high_level_mode_{HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING},
    mode_change_in_progress_{false},
    custom_flags_{custom_flags},
    ip_{TARGET_IP},
    port_{TARGET_PORT},
    verbosity_{verbosity},
    timeout_in_milliseconds_{TIMEOUT_IN_MILLISECONDS},
    LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_{false} // ALways false until we figure out an H1 alternative to this parameter
{
    impl_        = std::make_shared<DriverUnitreeH1::Impl>();
    impl_->safe_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::H1);
    impl_->udp_  = std::make_shared<UNITREE_LEGGED_SDK::UDP>
        (level == LEVEL::LOW ? UNITREE_LEGGED_SDK::LOWLEVEL : UNITREE_LEGGED_SDK::HIGHLEVEL,LOCAL_PORT,TARGET_IP.c_str(), TARGET_PORT);


    current_status_ = STATUS::IDLE;
    status_msg_ = std::string("Idle.");
    _set_driver_mode(mode, level);

    if (level == LEVEL::LOW)
        impl_->udp_->InitCmdData(impl_->low_cmd_);
    else
        impl_->udp_->InitCmdData(impl_->high_cmd_);


    UNITREE_LEGGED_SDK::InitEnvironment();

    impl_->loop_control_    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("control_loop", dt_, boost::bind(&DriverUnitreeH1::_robot_control,this));
    impl_->loop_echo_state_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("echo_state_loop", dt_, boost::bind(&DriverUnitreeH1::_robot_update,this));
    impl_->loop_udpSend_    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("udp_send", dt_, 3,  boost::bind(&DriverUnitreeH1::_UDPSend, this));
    impl_->loop_udpRecv_    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("udp_recv", dt_, 3,  boost::bind(&DriverUnitreeH1::_UDPRecv, this));

    ip_ = TARGET_IP;
    port_ = TARGET_PORT;
    std::cerr<<"ROBOT_IP: "<<ip_<<std::endl;
    std::cerr<<"ROBOT_PORT: "<<port_<<std::endl;


}

/**
 * @brief DriverUnitreeH1::get_target_ip returns the Robot IP (TARGET_IP) address. For high-level mode,
 *                  the IP address is usually "192.168.123.220" and "192.168.123.10" for low-level mode.
 * @return The robot IP address.
 */
std::string DriverUnitreeH1::get_target_ip() const
{
    return ip_;
}


/**
 * @brief DriverUnitreeH1::get_target_port returns the robot port (target port). Usually this value
 *                  corresponds to 8082 for high-level mode and 8007 for low-level driver.
 * @return
 */
int DriverUnitreeH1::get_target_port() const
{
    return port_;
}


/**
 * @brief DriverUnitreeH1::get_motiontime returns the elapsed time in the control loop thread.
 * @return
 */
int DriverUnitreeH1::get_motiontime() const
{
    return motiontime_;
}

/**
 * @brief DriverUnitreeH1::get_realtime_controller returns the elapsed time in the low-level controller.
 * @return
 */
uint32_t DriverUnitreeH1::get_realtime_controller() const
{
    return tick_;
}

/**
 * @brief DriverUnitreeH1::get_state_of_charge returns the state of charge of the H1 battery.
 * @return A value from 0-100%
 */
int DriverUnitreeH1::get_state_of_charge() const
{
    return state_of_charge_;
}

/**
 * @brief DriverUnitreeH1::get_status_message returns the status message of the driver.
 * @return
 */
std::string DriverUnitreeH1::get_status_message() const
{
    return status_msg_;
}


/**
 * @brief DriverUnitreeH1::get_udp_status returns the UDP communication status.
 * @return a 7-dimensional vector containing the UDP communication status. The vector containts
 *      the following ordered data:
 *
 *       unsigned long long TotalCount;	  // total loop count
 *       unsigned long long SendCount;	  // total send count
 *       unsigned long long RecvCount;	  // total receive count
 *       unsigned long long SendError;	  // total send error
 *       unsigned long long FlagError;	  // total flag error
 *       unsigned long long RecvCRCError;  // total reveive CRC error
 *       unsigned long long RecvLoseError; // total lose package count
 */
std::vector<unsigned long long> DriverUnitreeH1::get_udp_status()
{
    return upd_status_;
}

/**
 * @brief DriverUnitreeH1::get_connection_status returns a flag that represents the connection status.
 * @return The connection status flag. Returns true if the connection was successful. False otherwise.
 */
bool DriverUnitreeH1::get_connection_status()
{
    return communication_established_;
}

/**
 * @brief DriverUnitreeH1::_UDPRecv receives data from the UDP communication. This callback method is used by loop_udpRecv_.
 */
void DriverUnitreeH1::_UDPRecv()
{
    impl_->udp_->Recv();
    _update_udp_status();
}

/**
 * @brief DriverUnitreeH1::_UDPSend sends data using the UDP communication. This callback method is used by loop_udpSend_.
 */
void DriverUnitreeH1::_UDPSend()
{
    impl_->udp_->Send();
    _update_udp_status();
}

/**
 * @brief DriverUnitreeH1::_update_udp_status updates the UDP status if the data it is initialized.
 *
 */
void DriverUnitreeH1::_update_udp_status()
{
    upd_status_.at(0) = impl_->udp_->udpState.TotalCount >0 ? impl_->udp_->udpState.TotalCount : 0;
    upd_status_.at(1) = impl_->udp_->udpState.SendCount  >0 ? impl_->udp_->udpState.SendCount : 0;
    upd_status_.at(2) = impl_->udp_->udpState.RecvCount  >0 ? impl_->udp_->udpState.RecvCount : 0;
    upd_status_.at(3) = impl_->udp_->udpState.SendError  >0 ? impl_->udp_->udpState.SendError : 0;
    upd_status_.at(4) = impl_->udp_->udpState.FlagError  >0 ? impl_->udp_->udpState.FlagError : 0;
    upd_status_.at(5) = impl_->udp_->udpState.RecvCRCError >0 ? impl_->udp_->udpState.RecvCRCError : 0;
    upd_status_.at(6) = impl_->udp_->udpState.RecvLoseError >0 ? impl_->udp_->udpState.RecvLoseError : 0;
}


/**
 * @brief DriverUnitreeH1::_set_driver_mode sets the driver mode.
 * @param mode The operation mode. Select the control strategy to command the robot.
 * @param level The control level to be used. HIGH or LOW.
 *              The HIGH level is used to send task space commands
 *              to the robot. The robot constraints are handled by the own robot  using a Unitree internal controller.
 *              The LOW mode is used to send joint position, velocity or torque commands. In this case, you must take into account
 *              all constraints in your controler (joint limits, control input limits, robot balance, self-collision avoidance, etc).
 */
void DriverUnitreeH1::_set_driver_mode(const MODE &mode, const LEVEL &level)
{
    // TODO: Update this function, and the driver architecture more generally, to amend the distinction between high level
    // and low level control. In the H1, the upper body does not appear to have a high level control interface, only
    // the lower body does. Maybe we can change it so that the LEVEL is clear that it only applies to the lower body, like
    // LOWER_BODY_LEVEL or similar.
    switch (mode)
    {
        case MODE::None:
            std::cerr<<"RobotDriverUnitreeH1::_set_driver_mode. Driver is set to Mode::None. "<<std::endl;
            break;
        case MODE::PositionControl:
            if (level == LEVEL::LOW)
            {
                throw std::runtime_error(std::string("RobotDriverUnitreeH1::_set_driver_mode. PositionControl in low-level mode is not available. "));
            }else { //HIGH LEVEL
                throw std::runtime_error(std::string("RobotDriverUnitreeH1::_set_driver_mode. PositionControl in high-level mode is not available. "));
            }

            break;
        case MODE::VelocityControl:
            if (level == LEVEL::LOW)
            {
                throw std::runtime_error(std::string("RobotDriverUnitreeH1::_set_driver_mode. VelocityControl in low-level mode is not available. "));
            }else { //HIGH LEVEL
                std::cerr<<"RobotDriverUnitreeH1::_set_driver_mode. VelocityControl in high-level mode is experimental. "<<std::endl;
                _initialize_high_cmd_variable();
            }
            break;
        case MODE::ForceControl:
            if (level == LEVEL::LOW)
            {
                throw std::runtime_error(std::string("RobotDriverUnitreeH1::_set_driver_mode. ForceControl in low-level mode is not available. "));
            }else { //HIGH LEVEL
                throw std::runtime_error(std::string("RobotDriverUnitreeH1::_set_driver_mode. ForceControl in high-level mode is not available. "));
            }

            break;
    }
    mode_ = mode;
    level_ = level;
}


/**
 * @brief DriverUnitreeH1::_initialize_high_cmd_variable sets the high_cmd_ (UNITREE_LEGGED_SDK::HighCmd struct) attribute with zeros.
 */
void DriverUnitreeH1::_initialize_high_cmd_variable()
{
    impl_->high_cmd_.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
    impl_->high_cmd_.gaitType = 0;
    impl_->high_cmd_.speedLevel = 0;
    impl_->high_cmd_.footRaiseHeight = 0;
    impl_->high_cmd_.bodyHeight = 0;
    impl_->high_cmd_.euler[0] = 0;
    impl_->high_cmd_.euler[1] = 0;
    impl_->high_cmd_.euler[2] = 0;
    impl_->high_cmd_.velocity[0] = 0.0f;
    impl_->high_cmd_.velocity[1] = 0.0f;
    impl_->high_cmd_.yawSpeed = 0.0f;
    impl_->high_cmd_.reserve = 0;
}

/**
 * @brief DriverUnitreeH1::are_approximately_equal returns true if two doubles are approximately equal
 * @param a A double
 * @param b A double
 * @param epsilon the tolerance
 * @return returns true if two doubles are approximately equal. False otherwise
 */
bool DriverUnitreeH1::are_approximately_equal(const double &a, const double &b, const double &epsilon)
{
    return std::abs(a - b) < epsilon;
}

/**
 * @brief DriverUnitreeH1::connect This method starts the threads related to the UPD communication between the PC running the
 *                          controller and the H1 hardware (motors, sensors, battery, etc). Furthermore, a thread to update the robot state
 *                          is started automatically. At this stage, there are no control loops running, and consequently, the robot
 *                          will not perform any movement.
 */
void DriverUnitreeH1::connect()
{

    //TODO: Determine if I need to use udp in this way. From the examples, it seems like it isn't necessary to explicitly
    // manage the communications in this way, but its worth making sure. If you give the arm example the wrong ethernet device,
    // then it doesn't warn you. It just runs the program and nothing happens. That's something we should fix before implementing
    // this function.
    if (current_status_ == STATUS::IDLE)
    {
        impl_->loop_udpSend_->start();
        impl_->loop_udpRecv_->start();
        impl_->loop_echo_state_->start();

        status_msg_ = "connecting...";
        for (int i=0;i<timeout_in_milliseconds_;i++)
        {
            if (get_udp_status().at(2) > 0)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        _show_status();
        if (get_udp_status().at(2) > 0)
        {
            current_status_ = STATUS::CONNECTED;
            status_msg_ = "connected!";
            communication_established_ = true;
            _show_status();

        }else
        {
            current_status_ = STATUS::IDLE;
            status_msg_ = "Connection failed!";
            impl_->loop_udpSend_->shutdown();
            impl_->loop_udpRecv_->shutdown();
            impl_->loop_echo_state_->shutdown();
            _show_status();
            //throw std::runtime_error("Unestablished connection with the H1 robot!");
        }


    }
}

/**
 * @brief DriverUnitreeH1::initialize starts the appropriate threads based on the selected mode of operation.
 *                  This method requires established communication with the robot, i.e. the user must call connect()
 *                  before calling initialize().
 *                  If the operation mode is different from None, the robot may move!
 *                  WARNING: Be prepared to stop the robot with an emergency stop protocol!
 */
void DriverUnitreeH1::initialize()
{
    if (current_status_ == STATUS::CONNECTED)
    {
        switch (mode_)
        {
            case MODE::None:
                break;
            case MODE::PositionControl:
                if (level_ == LEVEL::LOW)
                {
                    std::cerr<<"RobotDriverUnitreeH1::initialize. PositionControl  in low-level mode is not available. "<<std::endl;
                    deinitialize();
                }else{ //HIGH LEVEL
                    std::cerr<<"RobotDriverUnitreeH1::initialize. PositionControl  in high-level mode is not available. "<<std::endl;
                    deinitialize();
                    }
                break;
            case MODE::VelocityControl:

                if (level_ == LEVEL::LOW)
                {
                    std::cerr<<"RobotDriverUnitreeH1::initialize. VelocityControl in low-level mode is not available. "<<std::endl;
                    deinitialize();
                }else { //HIGH LEVEL
                        status_msg_ = "finishing echo state loop.";

                        // TODO: Understand the purpose of the echo state loop
                        _show_status();
                        impl_->loop_echo_state_->shutdown();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        impl_->loop_control_->start();
                        status_msg_ = "starting control loop.";
                        _show_status();
                    }
                break;
            case MODE::ForceControl:
                if (level_ == LEVEL::LOW)
                {
                    std::cerr<<"RobotDriverUnitreeH1::initialize. ForceControl in low-level mode is not available. "<<std::endl;
                    deinitialize();
                }else{
                    std::cerr<<"RobotDriverUnitreeH1::initialize. ForceControl in high-level mode is not available. "<<std::endl;
                    deinitialize();
                }
                break;
        }
        current_status_ = STATUS::INITIALIZED;
        status_msg_ = "Initialized!";
    }else{
        std::cerr<<"RobotDriverUnitreeH1::initialize. The driver must be connected before to be initialized. "<<std::endl;
    }
}


/**
 * @brief DriverUnitreeH1::deinitialize stops all communication threads.
 *                  This method requires initialized communication with the robot, i.e. the user must call both connect(),
 *                  and initialize() before calling deinitialize().
 */
void DriverUnitreeH1::deinitialize()
{
    //wait to finish;
    finish_motion_to_deinitialize_ = true;
    std::cerr<<"Waiting to deinitialize..."<<std::endl;

    if (current_status_ != STATUS::INITIALIZED)
        // If the robot was not initialized, set this flag to break the
        // while loop that waits for the robot to be ready.
        the_robot_is_ready_to_deinitialize_ = true;

    while(!the_robot_is_ready_to_deinitialize_){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

    std::cerr<<"We are ready to deinitialize!"<<std::endl;
    impl_->loop_udpSend_->shutdown();
    impl_->loop_udpRecv_->shutdown();
    impl_->loop_echo_state_->shutdown();
    impl_->loop_control_->shutdown();

    status_msg_ = "All loops are shutdown!";
    _show_status();

    current_status_ = STATUS::DEINITIALIZED;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    status_msg_ = "Deinitialized!";
    _show_status();
}

/**
 * @brief DriverUnitreeH1::disconnect
 */
void DriverUnitreeH1::disconnect()
{
    current_status_ = STATUS::DISCONNECTED;
    status_msg_ = "Disconnected!";
    _show_status();
}

/**
 * @brief DriverUnitreeH1::get_leg_joint_positions returns the joint positions of the robot legs.
 * @return A tuple containing the robot configuration legs in the following order: Front Right, Front Left, Rear Right, Rear Left.
 */
std::tuple<VectorXd, VectorXd, VectorXd, VectorXd> DriverUnitreeH1::get_leg_joint_positions() const
{
    //uLA, uRA, uLL, uRL
    return {qLA_, qRA_, qLL_, qRL_};
}

/**
 * @brief DriverUnitreeH1::get_joint_positions returns the joint positions of the specified leg.
 * @param branch The desired branch (arm of leg). You can use LA (left arm), RA (right arm), LL (left leg), or RL (right leg).
 * @return the current joint positions (unit: radian)
 */
VectorXd DriverUnitreeH1::get_joint_positions(const BRANCH &branch) const
{

    // TODO: The high level object contains the call for the sas function get_joint_positions. That function 
    // calls this function four times, once for each branch, and stitches the results together into one vector. 
    // I think it would be best in my implementation to do away with the branch structure entirely, and have 
    // this function just get all the joint states in a single vector.

    switch (branch){

    case BRANCH::LA:
        return qLA_;
    case BRANCH::RA:
        return qRA_;
    case BRANCH::LL:
        return qLL_;
    case BRANCH::RL:
        return qRL_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeH1::get_joint_positions");
        break;
    }
}

/**
 * @brief DriverUnitreeH1::get_joint_velocities returns the joint velocities of the specified leg.
 * @param branch The desired branch (arm of leg). You can use LA (left arm), RA (right arm), LL (left leg), or RL (right leg).
 * @return the current joint velocities (unit: radian/second)
 */
VectorXd DriverUnitreeH1::get_joint_velocities(const BRANCH &branch) const
{

    // TODO: see get_joint_positions
    switch (branch){

    case BRANCH::LA:
        return qLA_dot_;
    case BRANCH::RA:
        return qRA_dot_;
    case BRANCH::LL:
        return q_LL_dot_;
    case BRANCH::RL:
        return qRL_dot_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeH1::get_joint_velocities");
        break;
    }
}


/**
 * @brief DriverUnitreeH1::get_joint_accelerations returns the joint accelerations of the specified leg.
 * @param branch The desired branch (arm of leg). You can use LA (left arm), RA (right arm), LL (left leg), or RL (right leg).
 * @return the current joint accelerations (unit: radian/second^2)
 */
VectorXd DriverUnitreeH1::get_joint_accelerations(const BRANCH &branch) const
{
    // TODO: see get_joint_positions
    switch (branch){

    case BRANCH::LA:
        return qLA_ddot_;
    case BRANCH::RA:
        return qRA_ddot_;
    case BRANCH::LL:
        return q_LL_ddot_;
    case BRANCH::RL:
        return qRL_ddot_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeH1::get_joint_accelerations");
        break;
    }

}

/**
 * @brief DriverUnitreeH1::get_joint_estimated_torques returns the estimated joint torques of the specified leg.
 * @param branch The desired branch (arm of leg). You can use LA (left arm), RA (right arm), LL (left leg), or RL (right leg).
 * @return the estimated joint torques (unit: N.m)
 */
VectorXd DriverUnitreeH1::get_joint_estimated_torques(const BRANCH &branch) const
{
    // TODO: see get_joint_positions
    switch (branch){

    case BRANCH::LA:
        return tauLA_;
    case BRANCH::RA:
        return tauRA_;
    case BRANCH::LL:
        return tauLL_;
    case BRANCH::RL:
        return tauRL_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeH1::get_joint_estimated_torques");
        break;
    }
}


/**
 * @brief DriverUnitreeH1::get_joint_temperatures returns the motor temperatures of the specified leg.
 * @param branch The desired branch (arm of leg). You can use LA (left arm), RA (right arm), LL (left leg), or RL (right leg).
 * @return The motor temperatures.
 */
VectorXd DriverUnitreeH1::get_joint_temperatures(const BRANCH &branch) const
{
    switch (branch){

    case BRANCH::LA:
        return temperatureLA_;
    case BRANCH::RA:
        return temperatureRA_;
    case BRANCH::LL:
        return temperatureLL_;
    case BRANCH::RL:
        return temperatureRL_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeH1::get_joint_estimated_temperatures");
        break;
    }
}

/**
 * @brief DriverUnitreeH1::get_IMU_orientation returns the IMU-based estimated orientation.
 * @return
 */
DQ DriverUnitreeH1::get_IMU_orientation() const
{
    return IMU_orientation_;
}

DQ DriverUnitreeH1::get_last_IMU_orientation_when_robot_stopped() const
{
    return last_IMU_orientation_when_robot_stopped_;
}

Vector3d DriverUnitreeH1::get_IMU_rpy_angles() const
{
    return IMU_rpy_;
}

/**
 * @brief DriverUnitreeH1::get_IMU_gyroscope returns the IMU-based estimated velocities.
 * @return
 */
DQ DriverUnitreeH1::get_IMU_gyroscope() const
{
    return IMU_gyroscope_;
}

/**
 * @brief DriverUnitreeH1::get_IMU_accelerometer returns the IMU-based estimated accelerations.
 * @return
 */
DQ DriverUnitreeH1::get_IMU_accelerometer() const
{
    return IMU_accelerometer_;
}

/**
 * @brief DriverUnitreeH1::get_odometry_position returns the IMU-based estimated robot position.
 * @return
 */
DQ DriverUnitreeH1::get_odometry_position() const
{
    return odometry_position_;
}

/**
 * @brief DriverUnitreeH1::get_body_height returns the estimated body height.
 * @return
 */
double DriverUnitreeH1::get_body_height() const
{
    return body_height_;
}

/**
 * @brief DriverUnitreeH1::get_IMU_pose returns the estimated robot pose at the IMU body frame.
 *                         This value is computed using the odometry data (subjected to drift), the body height,
 *                         and the IMU orientation.
 * @return
 */
DQ DriverUnitreeH1::get_IMU_pose() const
{
    if (is_unit(IMU_orientation_))
    {
        const DQ& r = IMU_orientation_;

        // This value is used to match the the height in the CoppeliaSim model
        const double hoffset = 0.025;
        const VectorXd vec_auxp = odometry_position_.vec4();
        const double x = vec_auxp(1);
        const double y = vec_auxp(2);
        const double z = body_height_ + hoffset;
        const DQ p = x*i_ + y*j_ + z*k_;

        return (r + E_*0.5*p*r).normalize();
    }else
    {
        std::cerr<<"DriverUnitreeH1::get_IMU_pose(): The IMU orientation data is not a unit quaternion!"<<std::endl;
        return DQ(1);
    }
}


/**
 * @brief DriverUnitreeH1::get_mobile_platform_configuration_from_IMU_pose returns the configuration of the holonomic mobile platform.
 * @return A vector containing the x-position, y-position, and the rotation (yaw) angle.
 */
VectorXd DriverUnitreeH1::get_mobile_platform_configuration_from_IMU_pose() const
{
    auto x = get_IMU_pose();
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}


/**
 * @brief DriverUnitreeH1::get_high_level_angular_velocity returns the angular velocities when in High level mode
 * @return The angular velocity (yaw_speed*k_)
 */

DQ DriverUnitreeH1::get_high_level_angular_velocity() const
{
    return high_level_angular_velocity_;
}

/**
 * @brief DriverUnitreeH1::get_high_level_linear_velocity returns the linear velocities when in High level mode
 * @return The planar joint velocties (x_dot*i_ + y_dot*j_)
 */
DQ DriverUnitreeH1::get_high_level_linear_velocity() const
{
    return high_level_linear_velocity_;
}


/**
 * @brief DriverUnitreeH1::set_high_level_forward_speed sets the target forward speed of the holonomic mobile platform.
 * @param forward_speed The desired forward speed. This method is used when the driver is set in high-level.
 */
void DriverUnitreeH1::set_high_level_forward_speed(const double &forward_speed)
{
    target_high_level_forward_speed_ = forward_speed;
}

/**
 * @brief DriverUnitreeH1::set_high_level_yaw_speed sets the yaw speed of the holonomic mobile platform.
 * @param yaw_speed The desired yaw speed. This method is used when the driver is set in high-level.
 */
void DriverUnitreeH1::set_high_level_yaw_speed(const double &yaw_speed)
{
    target_high_level_yaw_speed_ = yaw_speed;
}

/**
 * @brief DriverUnitreeH1::set_high_level_forward_and_yaw_speed sets the target forward and yaw speeds of the holonomic mobile platform.
 *                         This method is used when the driver is set in high-level.
 * @param forward_speed The desired forward speed.
 * @param yaw_speed The desired yaw speed.
 */
void DriverUnitreeH1::set_high_level_forward_and_yaw_speed(const double &forward_speed, const double &yaw_speed)
{
    // TODO: Some of these methods, including this one, don't appear to be called anywhere. Do we need them?
    set_high_level_forward_speed(forward_speed);
    set_high_level_yaw_speed(yaw_speed);
}

/**
 * @brief DriverUnitreeH1::set_high_level_speed sets the target forward, side and yaw speeds of the holonomic mobile platform.
 *                         This method is used when the driver is set in high-level.
 * @param forward_speed The desired forward speed.
 * @param side_speed  The desired side speed.
 * @param yaw_speed The desired yaw speed.
 */
void DriverUnitreeH1::set_high_level_speed(const double &forward_speed,
                                           const double &side_speed,
                                           const double &yaw_speed)
{
    target_high_level_forward_speed_ = forward_speed;
    target_high_level_side_speed_ = side_speed;
    target_high_level_yaw_speed_ = yaw_speed;
}

void DriverUnitreeH1::set_forced_stand_commands(const double &roll_angle,
                                                const double &pitch_angle,
                                                const double &yaw_angle,
                                                const double &bodyheight)
{
    target_high_level_roll_angle_ = roll_angle;
    target_high_level_pitch_angle_ = pitch_angle;
    target_high_level_yaw_angle_ = yaw_angle;
    target_high_level_bodyheight_ = bodyheight;
}

/**
 * @brief DriverUnitreeH1::get_high_level_forward_speed_reference returns the mobile platform velocities.
 * @return
 */
double DriverUnitreeH1::get_high_level_forward_speed_reference() const
{
    return target_high_level_forward_speed_;
}

/**
 * @brief DriverUnitreeH1::get_high_level_yaw_speed_reference returns the yaw speed of the mobile platform.
 * @return
 */
double DriverUnitreeH1::get_high_level_yaw_speed_reference() const
{
    return target_high_level_yaw_speed_;
}

/**
 * @brief DriverUnitreeH1::show_high_mode displays the current high-level mode
 */
void DriverUnitreeH1::show_high_mode() const
{
    switch(current_high_level_mode_){

    case HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND:
        std::cerr<<"IDLE_DEFAULT_STAND"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::FORCED_STAND:
        std::cerr<<"FORCE_STAND"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING:
        std::cerr<<"TARGET_VELOCITY_WALKING"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::PATH_MODE_WALKING:
        std::cerr<<"PATH_MODE_WALKING"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::POSITION_STAND_DOWN:
        std::cerr<<"POSITION_STAND_DOWN"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::POSITION_STAND_UP:
        std::cerr<<"POSITION_STAND_UP"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::DAMPING_MODE:
        std::cerr<<"DAMPING_MODE"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::RECOVERY_STAND:
        std::cerr<<"RECOVERY_STAND"<<std::endl;
        break;
    }
}

/**
 * @brief DriverUnitreeH1::get_motion_time returns the elapsed time of the thread control loop.
 * @return
 */
unsigned long long DriverUnitreeH1::get_motion_time() const
{
    return motiontime_;
}


/**
 * @brief Requests a change to a new high-level robot control mode.
 *
 * Validates the requested mode and initiates a mode transition if different from current.
 * Supported modes: IDLE_DEFAULT_STAND, FORCED_STAND, TARGET_VELOCITY_WALKING.
 *
 * When a mode change is requested:
 * - Sets mode_change_in_progress_ = true to trigger stopping sequence
 * - Updates target_high_level_mode_ to the new mode
 * - The actual mode change occurs after the robot stops (handled in _robot_control())
 *
 * @param mode The requested high-level control mode
 * @throws std::runtime_error if mode is not supported
 *
 * @note Unsupported modes (PATH_MODE_WALKING, POSITION_STAND_DOWN, etc.) will throw an exception
 * @see mode_change_in_progress_
 * @see target_high_level_mode_
 */
void DriverUnitreeH1::request_change_in_high_level_control(const HIGH_LEVEL_MODE &mode)
{
    if (target_high_level_mode_ != mode)
    {
        // Validate mode is supported
        switch (mode) {
        case HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND:
        case HIGH_LEVEL_MODE::FORCED_STAND:
        case HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING:
            break;  // Supported
        default:
            throw std::runtime_error("DriverUnitreeH1::request_change_in_high_level_control: Unsupported mode!");
        }
        mode_change_in_progress_ = true;
        target_high_level_mode_ = mode;
    }
}


/**
 * @brief Converts HIGH_LEVEL_MODE enum to human-readable string.
 * @param mode The high-level mode to convert
 * @return String representation of the mode ("IDLE_DEFAULT_STAND", "FORCED_STAND",
 *         "TARGET_VELOCITY_WALKING", "PATH_MODE_WALKING", "POSITION_STAND_DOWN",
 *         "POSITION_STAND_UP", "DAMPING_MODE", "RECOVERY_STAND", or "UNKNOWN")
 */
string DriverUnitreeH1::high_level_mode_to_string(const HIGH_LEVEL_MODE &mode) const
{
    switch (mode) {
    case HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND: return "IDLE_DEFAULT_STAND";
    case HIGH_LEVEL_MODE::FORCED_STAND: return "FORCED_STAND";
    case HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING: return "TARGET_VELOCITY_WALKING";
    case HIGH_LEVEL_MODE::PATH_MODE_WALKING: return "PATH_MODE_WALKING";
    case HIGH_LEVEL_MODE::POSITION_STAND_DOWN: return "POSITION_STAND_DOWN";
    case HIGH_LEVEL_MODE::POSITION_STAND_UP: return "POSITION_STAND_UP";
    case HIGH_LEVEL_MODE::DAMPING_MODE: return "DAMPING_MODE";
    case HIGH_LEVEL_MODE::RECOVERY_STAND: return "RECOVERY_STAND";
    default: return "UNKNOWN";
    }
}


/**
 * @brief Converts GAIT_TYPE enum to human-readable string.
 * @param gait_type The gait type to convert
 * @return String representation ("IDLE", "TROT", "TROT_RUNNING", "CLIMB_STAIR", "TROT_OBSTACLE", or "UNKNOWN")
 */
std::string DriverUnitreeH1::gait_type_to_string(const GAIT_TYPE& gait_type) const
{
    switch (gait_type) {
    case GAIT_TYPE::IDLE: return "IDLE";
    case GAIT_TYPE::TROT: return "TROT";
    case GAIT_TYPE::TROT_RUNNING: return "TROT_RUNNING";
    case GAIT_TYPE::CLIMB_STAIR: return "CLIMB_STAIR";
    case GAIT_TYPE::TROT_OBSTACLE: return "TROT_OBSTACLE";
    default: return "UNKNOWN";
    }
}


/// Returns the target high-level control mode (what the driver is trying to achieve)
DriverUnitreeH1::HIGH_LEVEL_MODE DriverUnitreeH1::get_target_high_mode() const
{
    return target_high_level_mode_;
}

/// Returns the current high-level control mode reported by the robot
DriverUnitreeH1::HIGH_LEVEL_MODE DriverUnitreeH1::get_current_high_mode() const
{
    return current_high_level_mode_;
}

/// Returns the current gait type (e.g., TROT, IDLE) reported by the robot
DriverUnitreeH1::GAIT_TYPE DriverUnitreeH1::get_current_gait_type() const
{
    return current_gait_type_;
}




/**
 * @brief DriverUnitreeH1::_robot_control callback method used by the thread control loop.
 */

void DriverUnitreeH1::_robot_control()
{
    motiontime_ += 2;//motiontime_++;
    _update_data_from_robot_state();
    switch (mode_)
    {
    case MODE::None:
        break;
    case MODE::PositionControl:
        break;
    case MODE::VelocityControl:
        if (level_ == LEVEL::LOW)
        {
            throw std::runtime_error(std::string("RobotDriverUnitreeH1::_set_driver_mode. VelocityControl in low-level mode is not available. "));
        }
        else
        { //HIGH LEVEL
            if (!finish_motion_to_deinitialize_)
            {
                if (robot_is_prepared_for_high_level_motion_)
                {
                    if (!mode_change_in_progress_)
                    {
                        _command_robot_in_high_level_motion();
                    }else{
                        if (!frozen_time_in_request_check_was_set_)
                        {
                            frozen_time_in_request_check_ = motiontime_;
                            frozen_time_in_request_check_was_set_ = true;
                        }
                        const int deltatime = 2000; //This time is based on the Unitree Examples
                        if (motiontime_>= frozen_time_in_request_check_ && motiontime_ < frozen_time_in_request_check_+deltatime)
                        {
                            _stop_robot_in_high_level_motion();
                            //_command_in_high_level_mode(HIGH_LEVEL_MODE::FORCED_STAND, 0.0, 0.0, 0.0);
                        }else
                        {
                           mode_change_in_progress_ = false;
                           frozen_time_in_request_check_was_set_ = false;
                           _command_robot_in_high_level_motion();
                        }
                    }
                }else{
                    _prepare_the_robot_for_high_level_motion();
                }
            }else{
                _finish_high_level_motion();
            }
        }
        break;
    case MODE::ForceControl:
        break;
    }
}

/**
 * @brief Stops the robot's motion in high-level control mode.
 *
 * Uses either zero-velocity walking commands or a two-stage braking strategy
 * (walking stop -> force stand) depending on the FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO flag.
 * The two-stage approach is required for some firmware versions where FORCED_STAND
 * cannot stop a moving robot above speed_threshold_to_force_stand_mode_.
 */
void DriverUnitreeH1::_stop_robot_in_high_level_motion()
{
    bool force_stand_mode = flag_in_custom_flags(CUSTOM_FLAGS::FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO, custom_flags_);
    if (force_stand_mode)
    {
        const int STOPPING_DURATION_MS = 1500;

        // Start timer if not already started
        if (!frozen_time_high_level_stop_motion_was_set_)
        {
            frozen_time_high_level_stop_motion_ = motiontime_;
            frozen_time_high_level_stop_motion_was_set_ = true;
        }
        // Check if timer has expired
        bool timer_expired = (motiontime_ - frozen_time_high_level_stop_motion_) >= STOPPING_DURATION_MS;
        if (timer_expired)
        {
            // After STOPPING_DURATION_MS, use forced stand
            _command_in_high_level_mode(HIGH_LEVEL_MODE::FORCED_STAND, 0.0, 0.0, 0.0);
        }else
        {
            _command_in_high_level_mode(HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, 0.0, 0.0, 0.0);
        }
    }
    else
        _command_in_high_level_mode(HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, 0.0,0.0,0.0);

    last_IMU_orientation_when_robot_stopped_ = IMU_orientation_;
}



/**
 * @brief Prepares robot for high-level motion by transitioning to FORCED_STAND mode.
 *
 * The robot cannot directly switch from POSITION_STAND_UP to walking mode. It must first
 * transition to FORCED_STAND and stabilize for PREPARATION_DURATION_MS.
 *
 * Ready states (no preparation needed):
 * - FORCED_STAND: Robot is already in stable stand mode
 * - TARGET_VELOCITY_WALKING: Robot is already walking
 *
 * Transition sequence:
 * - POSITION_STAND_UP → FORCED_STAND → stabilize → ready
 *
 * @throws std::runtime_error If robot is in an unsupported mode (e.g., DAMPING_MODE,
 *         RECOVERY_STAND, IDLE_DEFAULT_STAND)
 *
 * @note DAMPING_MODE (lying down) requires transition: DAMPING_MODE → POSITION_STAND_UP →
 *       FORCED_STAND → stabilize → ready (not yet implemented)
 */
void DriverUnitreeH1::_prepare_the_robot_for_high_level_motion()
{
    const int PREPARATION_DURATION_MS = 1500;

    // If already in a valid motion mode, mark ready immediately
    if (current_high_level_mode_ == HIGH_LEVEL_MODE::FORCED_STAND ||
        current_high_level_mode_ == HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING)
    {
        robot_is_prepared_for_high_level_motion_ = true;
        frozen_time_high_level_motion_preparation_was_set_ = false;
        return;
    }

    if (!frozen_time_high_level_motion_preparation_was_set_)
    {
        frozen_time_high_level_motion_preparation_ = motiontime_;
        frozen_time_high_level_motion_preparation_was_set_ = true;
    }

    bool timer_expired = (motiontime_ - frozen_time_high_level_motion_preparation_) >= PREPARATION_DURATION_MS;

    // Only mark ready if timer expired AND we're in FORCED_STAND
    if (timer_expired && current_high_level_mode_ == HIGH_LEVEL_MODE::FORCED_STAND)
    {
        robot_is_prepared_for_high_level_motion_ = true;
        frozen_time_high_level_motion_preparation_was_set_ = false;
        return;
    }

    // If not ready yet, send appropriate commands
    if (current_high_level_mode_ == HIGH_LEVEL_MODE::POSITION_STAND_UP)
    {
        _command_in_high_level_mode(HIGH_LEVEL_MODE::FORCED_STAND, 0.0, 0.0, 0.0);
    }
    else if (current_high_level_mode_ != HIGH_LEVEL_MODE::FORCED_STAND)
    {
        // TODO: Handle other states like DAMPING_MODE (lying down) -> POSITION_STAND_UP -> FORCED_STAND
        throw std::runtime_error(
            "DriverUnitreeH1::_prepare_the_robot_for_high_level_motion: Cannot prepare robot from current mode: " +
            high_level_mode_to_string(current_high_level_mode_) +
            ". Only POSITION_STAND_UP, FORCED_STAND, and TARGET_VELOCITY_WALKING are supported."
            );
    }
    // If already in FORCED_STAND but timer not expired, just wait (do nothing)
}

/**
 * @brief Executes the appropriate high-level motion command based on the current target mode.
 *
 * Dispatches commands to the robot according to target_high_level_mode_:
 *
 * - TARGET_VELOCITY_WALKING: Commands walking with specified forward/side/yaw velocities.
 *   If all velocities are zero, calls _stop_robot_in_high_level_motion() instead.
 *
 * - FORCED_STAND: Commands force stand mode with specified roll, pitch, yaw, and body height.
 *
 * - Default (IDLE_DEFAULT_STAND): Commands idle stand mode (zero velocities).
 *
 * @note This function assumes mode_change_in_progress_ is false (no pending mode transition).
 * @see _stop_robot_in_high_level_motion()
 * @see _command_in_high_level_mode()
 */
void DriverUnitreeH1::_command_robot_in_high_level_motion()
{
    if (target_high_level_mode_ == HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING)
    {
        bool all_speeds_are_zero = are_approximately_equal(target_high_level_forward_speed_, 0.0) &&
                                   are_approximately_equal(target_high_level_side_speed_, 0.0) &&
                                   are_approximately_equal(target_high_level_yaw_speed_, 0.0);

        if (all_speeds_are_zero)
        {
            _stop_robot_in_high_level_motion();
        }
        else
        {
            // Reset the stop motion timer when we receive non-zero velocity commands
            // This allows the robot to start moving again immediately
            frozen_time_high_level_stop_motion_was_set_ = false;

            _command_in_high_level_mode(HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, target_high_level_forward_speed_,
                                        target_high_level_side_speed_,
                                        target_high_level_yaw_speed_,
                                        target_high_level_roll_angle_,
                                        target_high_level_pitch_angle_,
                                        target_high_level_yaw_angle_,
                                        target_high_level_bodyheight_);
        }
    }else if (target_high_level_mode_ == HIGH_LEVEL_MODE::FORCED_STAND)
    {
        // Reset timer when entering forced stand mode directly
        frozen_time_high_level_stop_motion_was_set_ = false;
        _command_in_high_level_mode(HIGH_LEVEL_MODE::FORCED_STAND,
                                    target_high_level_forward_speed_,
                                    target_high_level_side_speed_,
                                    target_high_level_yaw_speed_,
                                    target_high_level_roll_angle_,
                                    target_high_level_pitch_angle_,
                                    target_high_level_yaw_angle_,
                                    target_high_level_bodyheight_);

        bool all_target_angles_are_zero = are_approximately_equal(target_high_level_roll_angle_, 0.0) &&
                                          are_approximately_equal(target_high_level_pitch_angle_, 0.0) &&
                                          are_approximately_equal(target_high_level_yaw_angle_, 0.0);
        if (all_target_angles_are_zero)
            last_IMU_orientation_when_robot_stopped_ = IMU_orientation_;
    }
    else
    {
        _command_in_high_level_mode(HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND, 0,0,0);
    }
}


/**
 * @brief Executes controlled robot shutdown during driver deinitialization.
 *
 * Implements a 4-stage shutdown sequence (3 seconds per stage):
 * 1. Stop robot motion
 * 2. Stand up (if not in damping mode)
 * 3. Stand down (optional, based on LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_)
 * 4. Enter damping mode (optional)
 * 5. Final idle state with deinitialization flag set
 *
 * @note Uses static frozen_time - ensure single call per deinitialization
 */
void DriverUnitreeH1::_finish_high_level_motion()
{
    // This part of the code is executed when the driver is deinitialized.
    static unsigned long long frozen_time = motiontime_;
    const int deltatime = 3000;
    if (motiontime_>= frozen_time && motiontime_ < frozen_time+deltatime)
    {
        //show_high_mode();
        std::cout<<"Stopping...  "<< frozen_time+deltatime-motiontime_<<std::endl;
        _command_in_high_level_mode(HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, 0, 0, 0);//Stop the robot
    }
    else if (motiontime_>= frozen_time+deltatime && motiontime_ < frozen_time+2*deltatime)
    {
        //show_high_mode();
        if (current_high_level_mode_ == HIGH_LEVEL_MODE::DAMPING_MODE)
        {
            std::cout<<"ROBOT IS DAMPING MODE. I WILL IGNORE THE POSITION_STAND_UP... "<<frozen_time + 2*deltatime -motiontime_<<std::endl;
        }else
        {
            std::cout<<"POSITION_STAND_UP... "<<frozen_time + 2*deltatime -motiontime_<<std::endl;
            _command_in_high_level_mode(HIGH_LEVEL_MODE::POSITION_STAND_UP, 0, 0, 0); // Stand up pose
        }
    }
    else if (motiontime_>= frozen_time+2*deltatime && motiontime_ < frozen_time+ 3*deltatime && LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_)
    {
        //show_high_mode();
        std::cout<<"POSITION_STAND_DOWN... "<<frozen_time + 3*deltatime -motiontime_<<std::endl;
        _command_in_high_level_mode(HIGH_LEVEL_MODE::POSITION_STAND_DOWN, 0, 0, 0);//Stand down pose
    }
    else if (motiontime_>= frozen_time+ 3*deltatime && motiontime_ < frozen_time+ 4*deltatime && LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_)
    {
        //show_high_mode();
        std::cout<<"DAMPING_MODE... "<<frozen_time + 4*deltatime -motiontime_<<std::endl;
        _command_in_high_level_mode(HIGH_LEVEL_MODE::DAMPING_MODE, 0, 0, 0);//Damping mode
    }
    else{
        show_high_mode();
        std::cout<<"IDLE"<<std::endl;
        _command_in_high_level_mode(HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND, 0, 0, 0); //IDLE
        the_robot_is_ready_to_deinitialize_ = true;
    }
}




/**
 * @brief DriverUnitreeH1::_command_in_high_level_mode sets the high_cmd_ struct with the desired target values and sends to the robot.
 *
 * This method configures the Unitree H1 robot's high-level command structure based on the specified mode,
 * performs input validation, and transmits the command via UDP to the robot.
 *
 * @param high_level_mode The high level mode. This can be:
 *       - IDLE_DEFAULT_STAND (0): Idle/default stand mode (currently unsupported)
 *       - FORCED_STAND (1): Force stand mode controlled by body height and Euler angles
 *       - TARGET_VELOCITY_WALKING (2): Target velocity walking mode controlled by linear and angular velocities
 *       - PATH_MODE_WALKING (4): Path mode walking (reserved, currently unsupported)
 *       - POSITION_STAND_DOWN (5): Position stand down (currently unsupported)
 *       - POSITION_STAND_UP (6): Position stand up (currently unsupported)
 *       - DAMPING_MODE (7): Damping mode (currently unsupported)
 *       - RECOVERY_STAND (9): Recovery stand (currently unsupported)
 *
 * @param forward_vel Target forward linear velocity in m/s. Valid range depends on robot state.
 *                    Used only in TARGET_VELOCITY_WALKING mode.
 *
 * @param side_vel Target lateral (sideways) linear velocity in m/s. Valid range depends on robot state.
 *                 Used only in TARGET_VELOCITY_WALKING mode.
 *
 * @param yaw_speed Target yaw (rotation) angular velocity in rad/s. Valid range depends on robot state.
 *                  Used only in TARGET_VELOCITY_WALKING mode.
 *
 * @param roll_angle Target roll angle in radians. Valid range: [-0.3, 0.3].
 *                   Used only in FORCED_STAND mode.
 *
 * @param pitch_angle Target pitch angle in radians. Valid range: [-0.3, 0.3].
 *                    Used only in FORCED_STAND mode.
 *
 * @param yaw_angle Target yaw angle in radians. Valid range: [-0.6, 0.6].
 *                  Used only in FORCED_STAND mode.
 *
 * @param body_height Target body height from ground in meters. Valid range depends on robot configuration.
 *                    Used only in FORCED_STAND mode.
 *
 * @throws std::out_of_range If any FORCED_STAND mode Euler angle exceeds its valid range.
 * @throws std::runtime_error If an unsupported high_level_mode is provided.
 *
 * @note Only FORCED_STAND and TARGET_VELOCITY_WALKING modes are currently implemented.
 *       All other modes will throw a std::runtime_error.
 * @note The high_cmd_ structure is always zero-initialized before populating to prevent stale data.
 * @note The command is sent immediately via UDP after population.
 */
void DriverUnitreeH1::_command_in_high_level_mode(const HIGH_LEVEL_MODE& high_level_mode,
                                                  const double& forward_vel,
                                                  const double &side_vel,
                                                  const double& yaw_speed,
                                                  const double &roll_angle,
                                                  const double &pitch_angle,
                                                  const double &yaw_angle,
                                                  const double &body_height)
{
    _initialize_high_cmd_variable();
    impl_->high_cmd_.mode = high_level_mode_map_.at(high_level_mode);

    switch(high_level_mode) {
    case HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING:
    {
        impl_->high_cmd_.velocity[0] = forward_vel;
        impl_->high_cmd_.velocity[1] = side_vel;
        impl_->high_cmd_.yawSpeed = yaw_speed;
        break;
    }

    case HIGH_LEVEL_MODE::FORCED_STAND:
    {
        // (unit: rad), roll pitch yaw in stand mode,
        // roll range:[-0.3, 0.3],
        // pitch range:[-0.3, 0.3],
        // yaw range:[-0.6, 0.6]
        //range:[-0.16, 0.16]
        if (std::abs(roll_angle) > 0.3)
            throw std::out_of_range("Roll angle out of valid range [-0.3, 0.3] for FORCE_STAND mode");

        if (std::abs(pitch_angle) > 0.3)
            throw std::out_of_range("Pitch angle out of valid range [-0.3, 0.3] for FORCE_STAND mode");

        if (std::abs(yaw_angle) > 0.6)
           throw std::out_of_range("Yaw angle out of valid range [-0.6, 0.6] for FORCE_STAND mode");

        if (std::abs(body_height) > 0.16)
           throw std::out_of_range("Body height out of valid range [-0.16, 0.16] for FORCE_STAND mode");


        impl_->high_cmd_.euler[0] = roll_angle;
        impl_->high_cmd_.euler[1] = pitch_angle;
        impl_->high_cmd_.euler[2] = yaw_angle;



        impl_->high_cmd_.bodyHeight = body_height;
        break;
    }
    default:
        break;
        //std::cerr<<"DriverUnitreeH1::_command_in_high_level_mode: Unsupported mode!"<<std::endl;
    }

    impl_->udp_->SetSend(impl_->high_cmd_);
}


/**
 * @brief DriverUnitreeH1::_show_status displays the driver status if the verbosity flag is enabled.
 */
void DriverUnitreeH1::_show_status()
{
    if (verbosity_)
        std::cerr<<status_msg_<<std::endl;
}

/**
 * @brief DriverUnitreeH1::_update_data_from_robot_state callback method used in the thread related to the robot state.
 */
void DriverUnitreeH1::_update_data_from_robot_state()
{
    if (level_ == LEVEL::LOW)
    {
        impl_->udp_->GetRecv(impl_->low_state_);
        _update_joint_data(impl_->low_state_);
        _update_battery_data(impl_->low_state_);
        _update_IMU_data(impl_->low_state_);

        // real-time from motion controller
        tick_ = impl_->low_state_.tick;
    }else{
        impl_->udp_->GetRecv(impl_->high_state_);
        _update_joint_data(impl_->high_state_);
        _update_battery_data(impl_->high_state_);
        _update_IMU_data(impl_->high_state_);
        odometry_position_ = impl_->high_state_.position.at(0)*i_+
                             impl_->high_state_.position.at(1)*j_+
                             impl_->high_state_.position.at(2)*k_;
        body_height_ = impl_->high_state_.bodyHeight;
        current_high_level_mode_ = high_level_mode_map_inv_.at(impl_->high_state_.mode);

        current_gait_type_ = gait_type_map_inv_.at(impl_->high_state_.gaitType);

        high_level_linear_velocity_  = impl_->high_state_.velocity.at(0)*i_+
                                       impl_->high_state_.velocity.at(1)*j_;

        // this value impl_->high_state_.velocity.at(2)*k_ is not working. Therefore, I extracted the angular
        // velocity from yawSpeed.
        high_level_angular_velocity_ = impl_->high_state_.yawSpeed*k_;
    }
}


/**
 * @brief DriverUnitreeH1::_robot_update This method updates the robot state. This includes the
 *        joint positions, velocities, accelerations, torques, and temperatures.
 */
void DriverUnitreeH1::_robot_update()
{
    motiontime_++;
    _update_data_from_robot_state();
    if (finish_motion_to_deinitialize_)
        the_robot_is_ready_to_deinitialize_ = true;
}




/**
 * @brief DriverUnitreeH1::_update_joint_data updates the robot state related to the legs. This includes the joint positions, velocities,
 *                      accelerations, torques, and motor temperatures.
 * @param state The high-level of low-level structure to store the robot data.
 */
template<typename T>
void DriverUnitreeH1::_update_joint_data(const T &state)
{
    for (int i = 0; i<3;i++)
    {
        // Update the joint positions
        qLA_(i) = state.motorState[impl_->LA_index_.at(i)].q;
        qRA_(i) = state.motorState[impl_->RA_index_.at(i)].q;
        qLL_(i) = state.motorState[impl_->LL_index_.at(i)].q;
        qRL_(i) = state.motorState[impl_->RL_index_.at(i)].q;

        // Update the joint velocities
        qLA_dot_(i) = state.motorState[impl_->LA_index_.at(i)].dq;
        qRA_dot_(i) = state.motorState[impl_->RA_index_.at(i)].dq;
        qLL_dot_(i) = state.motorState[impl_->LL_index_.at(i)].dq;
        qRL_dot_(i) = state.motorState[impl_->RL_index_.at(i)].dq;

        // Update the joint accelerations
        qLA_ddot_(i) = state.motorState[impl_->LA_index_.at(i)].ddq;
        qRA_ddot_(i) = state.motorState[impl_->RA_index_.at(i)].ddq;
        qLL_ddot_(i) = state.motorState[impl_->LL_index_.at(i)].ddq;
        qRL_ddot_(i) = state.motorState[impl_->RL_index_.at(i)].ddq;

        // Update the estimated joint torques output
        tauLA_(i) = state.motorState[impl_->LA_index_.at(i)].tauEst;
        tauRA_(i) = state.motorState[impl_->RA_index_.at(i)].tauEst;
        tauLL_(i) = state.motorState[impl_->LL_index_.at(i)].tauEst;
        tauRL_(i) = state.motorState[impl_->RL_index_.at(i)].tauEst;

        temperatureLA_(i) = state.motorState[impl_->LA_index_.at(i)].temperature;
        temperatureRA_(i) = state.motorState[impl_->RA_index_.at(i)].temperature;
        temperatureLL_(i) = state.motorState[impl_->LL_index_.at(i)].temperature;
        temperatureRL_(i) = state.motorState[impl_->RL_index_.at(i)].temperature;
    }
}

/**
 * @brief DriverUnitreeH1::_update_IMU_data updates the IMU-based data
 * @param state The high-level of low-level structure to store the robot data.
 */
template<typename T>
void DriverUnitreeH1::_update_IMU_data(const T &state)
{
    IMU_orientation_ =   DQ(state.imu.quaternion.at(0),
                          state.imu.quaternion.at(1),
                          state.imu.quaternion.at(2),
                          state.imu.quaternion.at(3)).normalize();

    IMU_rpy_ << state.imu.rpy.at(0), state.imu.rpy.at(1), state.imu.rpy.at(2);

    IMU_gyroscope_ =  state.imu.gyroscope.at(0)*i_+
                      state.imu.gyroscope.at(1)*j_+
                      state.imu.gyroscope.at(2)*k_;

    IMU_accelerometer_  = state.imu.accelerometer.at(0)*i_+
                          state.imu.accelerometer.at(1)*j_+
                          state.imu.accelerometer.at(2)*k_;
}

/**
 * @brief DriverUnitreeH1::_update_battery_data updates the battery data.
 * @param state The high-level of low-level structure to store the robot data.
 */
template<typename T>
void DriverUnitreeH1::_update_battery_data(const T &state)
{
    // Update the battery status
    state_of_charge_ = state.bms.SOC;
}



