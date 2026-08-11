#include <chrono>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using Eigen::VectorXd;

int main(int argc, char * argv[])
{
    using namespace std::chrono_literals;

    // -------------------------------------------------------------------------
    // Set up ROS
    // -------------------------------------------------------------------------

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("interface_testing_node");

    const std::string topic_prefix = "/sas_h1/P_Body";

    auto joint_target_publisher =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(
            topic_prefix + "/set/target_joint_positions",
            rclcpp::QoS(1));

    auto torso_target_publisher =
    node->create_publisher<geometry_msgs::msg::TwistStamped>(
        topic_prefix + "/set/target_twist",
        rclcpp::QoS(1));

    // The latest valid joint positions received from the driver.
    VectorXd current_joint_positions = VectorXd::Zero(9);
    bool valid_joint_state_received = false;

    auto joint_position_subscriber =
        node->create_subscription<sensor_msgs::msg::JointState>(
            topic_prefix + "/get/joint_states",
            rclcpp::QoS(1),
            [&node,
             &current_joint_positions,
             &valid_joint_state_received](
                const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                // Check that the received vector has the expected size.
                if (msg->position.size() != 9) {
                    RCLCPP_ERROR_THROTTLE(
                        node->get_logger(),
                        *node->get_clock(),
                        1000,
                        "Expected 9 joint positions, received %zu.",
                        msg->position.size());

                    return;
                }

                // Copy the std::vector<double> into an Eigen vector.
                current_joint_positions =
                    Eigen::Map<const VectorXd>(
                        msg->position.data(),
                        static_cast<Eigen::Index>(
                            msg->position.size()));

                valid_joint_state_received = true;
            });

    // Prevent an unused-variable warning. Keeping this shared pointer alive is
    // necessary because destroying it would destroy the subscription.
    (void)joint_position_subscriber;

    // -------------------------------------------------------------------------
    // Wait for the driver interfaces
    // -------------------------------------------------------------------------

    while (
        rclcpp::ok() &&
        (
            joint_target_publisher->get_subscription_count() == 0 ||
            torso_target_publisher->get_subscription_count() == 0 ||
            !valid_joint_state_received
        ))
    {
        // Process incoming messages, including joint-state messages.
        rclcpp::spin_some(node);

        RCLCPP_INFO_THROTTLE(
            node->get_logger(),
            *node->get_clock(),
            1000,
            "Waiting for driver subscribers and a valid joint state...");

        rclcpp::sleep_for(20ms);
    }

    if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return 1;
    }

    std::cout
        << "Driver subscribers and valid joint state detected."
        << std::endl;

    // -------------------------------------------------------------------------
    // Wait for operator input
    // -------------------------------------------------------------------------

    std::cout << "Press ENTER to perform motion task ...";
    std::cout.flush();
    std::cin.get();

    // Continue processing joint states during the countdown. This means the
    // starting position is sampled immediately before the motion begins,
    // rather than before the operator presses ENTER.
    for (int count = 5; count >= 1 && rclcpp::ok(); --count) {
        std::cout << count << std::endl;

        const auto countdown_end =
            std::chrono::steady_clock::now() + 1s;

        while (
            rclcpp::ok() &&
            std::chrono::steady_clock::now() < countdown_end)
        {
            rclcpp::spin_some(node);
            rclcpp::sleep_for(20ms);
        }
    }

    if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return 1;
    }

    // -------------------------------------------------------------------------
    // Define the motion
    // -------------------------------------------------------------------------

    std::cout << "    Defining variables..." << std::endl;

    constexpr double command_frequency_hz = 50.0;
    constexpr double motion_duration_sec = 4.0;

    constexpr std::size_t num_cycles =
        static_cast<std::size_t>(
            command_frequency_hz * motion_duration_sec);

    VectorXd target_pos(9);
    target_pos <<
         1.571, 0.0, 1.571, 0.0,
        -1.571, 0.0, -1.571, 0.0,
         0.0;

    // Capture the latest valid state immediately before beginning the motion.
    const VectorXd starting_pos = current_joint_positions;

    VectorXd desired_torso_velocity(3);
    desired_torso_velocity << 0.2, 0.0, 0.0;

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Starting joint positions: "
            << starting_pos.transpose());

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Target joint positions: "
            << target_pos.transpose());

    // WallRate compensates for time spent constructing and publishing messages
    // better than sleeping for a fixed duration at the end of each iteration.
    rclcpp::WallRate command_rate(command_frequency_hz);

    // -------------------------------------------------------------------------
    // Perform the motion
    // -------------------------------------------------------------------------

    std::cout << "    Begin motion..." << std::endl;

    for (std::size_t i = 0;i < num_cycles && rclcpp::ok();++i){
        const double alpha =
            static_cast<double>(i + 1) /
            static_cast<double>(num_cycles);

        const VectorXd current_target =
            starting_pos +
            alpha * (target_pos - starting_pos);

        // ---------------------------------------------------------------------
        // Send torso command
        // ---------------------------------------------------------------------

        geometry_msgs::msg::TwistStamped torso_msg;

        torso_msg.header.stamp = node->now();
        torso_msg.twist.linear.x = desired_torso_velocity(0);
        torso_msg.twist.linear.y = desired_torso_velocity(1);
        torso_msg.twist.linear.z = 0.0;

        torso_msg.twist.angular.x = 0.0;
        torso_msg.twist.angular.y = 0.0;
        torso_msg.twist.angular.z = desired_torso_velocity(2);

        torso_target_publisher->publish(torso_msg);

        // ---------------------------------------------------------------------
        // Send joint command
        // ---------------------------------------------------------------------

        std_msgs::msg::Float64MultiArray joint_msg;

        joint_msg.data.assign(
            current_target.data(),
            current_target.data() +
                current_target.size());

        joint_target_publisher->publish(joint_msg);

        // Process ROS events without blocking. This keeps graph and
        // subscription processing active while the trajectory is running.
        rclcpp::spin_some(node);

        // Maintain approximately 50 Hz using a monotonic wall clock.
        if (!command_rate.sleep()) {
            RCLCPP_WARN(
                node->get_logger(),
                "The command loop failed to maintain %.1f Hz.",
                command_frequency_hz);
        }
    }

    // -------------------------------------------------------------------------
    // Stop the torso
    // -------------------------------------------------------------------------

    if (rclcpp::ok()) {
        geometry_msgs::msg::TwistStamped torso_stop_msg;

        torso_stop_msg.header.stamp = node->now();

        torso_stop_msg.twist.linear.x = 0.0;
        torso_stop_msg.twist.linear.y = 0.0;
        torso_stop_msg.twist.linear.z = 0.0;

        torso_stop_msg.twist.angular.x = 0.0;
        torso_stop_msg.twist.angular.y = 0.0;
        torso_stop_msg.twist.angular.z = 0.0;

        torso_target_publisher->publish(torso_stop_msg);

        /*
         * Give the middleware an opportunity to send the final command before
         * destroying the publisher. The driver's own command timeout and safe
         * deinitialisation remain the authoritative safety mechanisms.
         */
        const auto stop_publish_end =
            std::chrono::steady_clock::now() + 100ms;

        while (
            rclcpp::ok() &&
            std::chrono::steady_clock::now() < stop_publish_end)
        {
            rclcpp::spin_some(node);
            rclcpp::sleep_for(10ms);
        }
    }

    std::cout << "    Done." << std::endl;

    // -------------------------------------------------------------------------
    // Wrap up
    // -------------------------------------------------------------------------

    rclcpp::shutdown();
    return 0;
}