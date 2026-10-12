/**
 * @file ROS2_camera_viewer.cpp
 * @brief Display JPEG camera images received from the H1 ROS 2 publisher.
 *
 * Processing pipeline:
 *   ROS 2 compressed image -> retain latest message -> decode -> display
 *
 * Default topic:
 *   /sas_h1/camera/image_raw/compressed
 *
 * Message type:
 *   sensor_msgs::msg::CompressedImage
 *
 * This viewer needs a working graphical session to display the images
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>


class H1CameraViewer : public rclcpp::Node
{
public:
    H1CameraViewer()
        : Node("h1_camera_viewer")
    {
        // ----------------------------------------------------------
        // 1. Use same topic as publisher
        // ----------------------------------------------------------
        const auto topic = declare_parameter<std::string>("topic", "/sas_h1/camera/image_raw/compressed");


        // ----------------------------------------------------------
        // 2. Subscribe
        // ----------------------------------------------------------
        // Best effort allows frame loss rather than requiring retries.
        // Depth 1 prevents a large backlog of queued images.
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);

        subscription_ =
            create_subscription<sensor_msgs::msg::CompressedImage>(
                topic,
                qos,
                [this](
                    sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
                {
                    // Keep ownership of the incoming message through
                    // its shared pointer, without copying JPEG bytes.
                    //
                    // Replacing the pointer discards any previous
                    // message that has not yet been displayed.
                    latest_message_ = std::move(msg);
                });

        RCLCPP_INFO(
            get_logger(),
            "Subscribed to %s. Press Q or Escape to exit.",
            topic.c_str());
    }

    void display_latest_frame()
    {
        // ----------------------------------------------------------
        // 3. Consume the latest available message
        // ----------------------------------------------------------
        if (!latest_message_)
        {
            return;
        }

        // Move the shared pointer into a local variable. This clears
        // latest_message_, so this frame is not decoded again on the
        // next loop iteration.
        auto message = std::move(latest_message_);

        if (message->data.empty())
        {
            return;
        }

        try
        {
            // ------------------------------------------------------
            // 4. Decode the JPEG into a displayable BGR image
            // ------------------------------------------------------
            // This viewer is intended for the matching JPEG publisher.
            // It does not implement arbitrary image-transport formats.
            cv::Mat frame =
                cv::imdecode(message->data, cv::IMREAD_COLOR);

            if (frame.empty())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Could not decode compressed image");
                return;
            }


            // ------------------------------------------------------
            // 5. Display the already-upright image
            // ------------------------------------------------------
            cv::imshow(window_name(), frame);

            // Report the first successful image received
            if (!received_first_frame_)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Receiving %dx%d images; first JPEG %.1f KiB",
                    frame.cols, frame.rows,
                    static_cast<double>(message->data.size()) / 1024.0);

                received_first_frame_ = true;
            }
        }
        catch (const cv::Exception& error)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Image display failed: %s", error.what());
        }
    }

    static const char* window_name()
    {
        return "Unitree H1 compressed ROS 2 camera";
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;

    // Holds at most one pending message between callback processing
    // and display. Access remains on the single main thread.
    sensor_msgs::msg::CompressedImage::ConstSharedPtr latest_message_;

    bool received_first_frame_{false};
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    int exit_code = 0;

    try
    {
        auto node = std::make_shared<H1CameraViewer>();

        // ----------------------------------------------------------
        // 6. Create a resizable display window
        // ----------------------------------------------------------
        cv::namedWindow(H1CameraViewer::window_name(), cv::WINDOW_NORMAL);


        // ----------------------------------------------------------
        // 7. Interleave ROS callbacks and GUI event processing
        // ----------------------------------------------------------
        // A normal blocking rclcpp::spin() would not give this main loop
        // an opportunity to process window events independently.
        //
        // spin_some() processes available ROS work, then returns so
        // the latest image can be displayed and GUI events handled.
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (rclcpp::ok())
        {
            executor.spin_some();
            node->display_latest_frame();

            // Process window events even if no new image has arrived.
            // The delay is a GUI polling interval, not a frame-rate guarantee.
            const int key = cv::waitKey(1);

            if (key == 27 || key == 'q' || key == 'Q')
            {
                break;
            }

            // Also, exit if the user closes the window.
            if (cv::getWindowProperty(
                    H1CameraViewer::window_name(),
                    cv::WND_PROP_VISIBLE) < 1)
            {
                break;
            }
        }

        executor.remove_node(node);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Camera viewer failed: "<< error.what() << '\n';
        exit_code = 1;
    }


    // --------------------------------------------------------------
    // 8. Release GUI resources and shut down
    // --------------------------------------------------------------
    cv::destroyAllWindows();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return exit_code;
}
``