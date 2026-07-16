
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
//#include <sas_robot_driver_unitree_z1/sas_robot_driver_unitree_z1.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}

int main(int argc, char** argv)
{

    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_h1");

    try
    {
        sas::RobotDriverUnitreeH1Configuration robot_driver_unitree_h1_configuration;
        sas::get_ros_parameter(node,"mode", robot_driver_unitree_h1_configuration.mode);
        sas::get_ros_parameter(node,"LIE_DOWN_ROBOT_WHEN_DEINITIALIZE", robot_driver_unitree_h1_configuration.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE);
        sas::get_ros_parameter(node,"robot_name", robot_driver_unitree_h1_configuration.robot_name);
        sas::get_ros_parameter(node,"ROBOT_IP", robot_driver_unitree_h1_configuration.ROBOT_IP);
        sas::get_ros_parameter(node,"ROBOT_PORT", robot_driver_unitree_h1_configuration.ROBOT_PORT);
        sas::get_ros_optional_parameter(node, "FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO",
                                        robot_driver_unitree_h1_configuration.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO, false);


        auto robot_driver_unitree_h1 = std::make_shared<sas::RobotDriverUnitreeH1>(node,
                                                                        robot_driver_unitree_h1_configuration,
                                                                        shutdown_signaler);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");



        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_unitree_h1,
                                             robot_driver_ros_configuration,
                                             shutdown_signaler);
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    return 0;


}

