/**
 * @file ROS2_camera_publisher.cpp
 * @brief Publish upright, resized JPEG camera images from the Unitree H1.
 *
 * Processing pipeline:
 *   SDK image request -> decode -> resize -> rotate -> JPEG encode -> ROS 2
 *
 * Default topic:
 *   /sas_h1/camera/image_raw/compressed
 *
 * Message type:
 *   sensor_msgs::msg::CompressedImage
 *
 * This node is headless: it does not create windows or require X11.
 *
 * Best-effort delivery can drop frames.
 * 
 * SDK acquisition blocks the timer callback. Keep this standalone
 *   node separate from time-critical robot-control callbacks.
 */

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>


class H1CameraPublisher : public rclcpp::Node
{
public:
    H1CameraPublisher()
        : Node("h1_camera_publisher")
    {
        // ----------------------------------------------------------
        // 1. Read startup parameters
        // ----------------------------------------------------------

        // Network interface used by the Unitree SDK.
        const auto interface = declare_parameter<std::string>("interface", "");

        // Topic name
        const auto topic = declare_parameter<std::string>("topic", "/sas_h1/camera/image_raw/compressed");

        // Optical frame of the upright image.
        frame_id_ = declare_parameter<std::string>("frame_id", "h1_camera_upright_optical_frame");

        // Correct the upside-down camera image before publishing.
        rotate_180_ = declare_parameter<bool>("rotate_180", true);

        // Scale applies to BOTH width and height.
        // For example, 0.5 produces half-width, half-height images.
        scale_ = declare_parameter<double>("scale", 0.5);

        // controls the output encoding quality.
        jpeg_quality_ = declare_parameter<int>("jpeg_quality", 70);

        // Requested acquisition frequency in Hz, not guaranteed frame rate.
        const double rate = declare_parameter<double>("publish_rate", 20.0);

        // ----------------------------------------------------------
        // 2. Validate parameters before initialising communication
        // ----------------------------------------------------------
        if (!std::isfinite(rate) || rate <= 0.0)
        {
            throw std::invalid_argument(
                "publish_rate must be finite and positive");
        }

        if (!std::isfinite(scale_) || scale_ <= 0.0 || scale_ > 1.0)
        {
            throw std::invalid_argument(
                "scale must be greater than 0 and no greater than 1");
        }

        if (jpeg_quality_ < 0 || jpeg_quality_ > 100)
        {
            throw std::invalid_argument(
                "jpeg_quality must be between 0 and 100");
        }

        // Convert frequency into the timer's integer duration.
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));

        if (period.count() <= 0)
        {
            throw std::invalid_argument("publish_rate is too large");
        }


        // ----------------------------------------------------------
        // 3. Initialise the SDK
        // ----------------------------------------------------------
        unitree::robot::ChannelFactory::Instance()->Init(0, interface);

        // The unique_ptr starts empty, allowing explicit construction
        // after communication has been initialised.
        // Note: not sure if this is necessary
        video_client_ = std::make_unique<unitree::robot::go2::VideoClient>();

        // Limit how long an SDK request can wait before timing out.
        video_client_->SetTimeout(1.0f);
        video_client_->Init();


        // ----------------------------------------------------------
        // 4. Create the publisher
        // ----------------------------------------------------------
        // SensorDataQoS uses best-effort delivery and volatile durability.
        // Depth 1 retains only the latest queued sample.
        //
        // This configuration favours responsive live viewing rather
        // than guaranteed delivery of every frame. Subscribers must use
        // compatible QoS.
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);

        publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(topic, qos);


        // ----------------------------------------------------------
        // 5. Schedule acquisition and publication
        // ----------------------------------------------------------
        // The single-threaded executor calls publish_frame() on each
        // timer event.
        timer_ = create_wall_timer(period, std::bind(&H1CameraPublisher::publish_frame, this));

        RCLCPP_INFO(
            get_logger(),
            "Topic=%s; requested rate=%.1f Hz; scale=%.2f; "
            "JPEG quality=%d; rotate_180=%s",
            topic.c_str(), rate, scale_, jpeg_quality_,
            rotate_180_ ? "true" : "false");
    }

private:
    // Use a monotonic clock for duration measurements.
    using SteadyClock = std::chrono::steady_clock;

    static double milliseconds(SteadyClock::time_point start, SteadyClock::time_point end)
    {
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    void publish_frame()
    {
        try
        {
            const auto start = SteadyClock::now();


            // ------------------------------------------------------
            // 6. Request an encoded image from the robot
            // ------------------------------------------------------
            // Clear the previous contents so failed requests cannot
            // accidentally reuse an earlier image.
            image_sample_.clear();

            const int result = video_client_->GetImageSample(image_sample_);

            const auto acquired = SteadyClock::now();

            if (result != 0 || image_sample_.empty())
            {
                // Throttle repeated failures to avoid flooding logs.
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Camera request failed: code=%d, bytes=%zu",
                    result, image_sample_.size());
                return;
            }

            // The SDK call used here does not supply an exposure time.
            // Record when the response was received as an approximation.
            const auto receipt_time = now();


            // ------------------------------------------------------
            // 7. Decode the SDK image into an OpenCV BGR matrix
            // ------------------------------------------------------
            cv::Mat frame =
                cv::imdecode(image_sample_, cv::IMREAD_COLOR);

            if (frame.empty())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Could not decode SDK image");
                return;
            }


            // ------------------------------------------------------
            // 8. Reduce resolution before encoding
            // ------------------------------------------------------
            // INTER_AREA is used for downsampling.
            // Round dimensions to whole pixels and enforce a minimum
            // of one pixel in each direction.
            //
            // This does not change the camera's acquisition resolution.
            if (scale_ < 1.0)
            {
                const int width = std::max(
                    1, static_cast<int>(std::lround(frame.cols * scale_)));

                const int height = std::max(
                    1, static_cast<int>(std::lround(frame.rows * scale_)));

                cv::Mat smaller;
                cv::resize(
                    frame, smaller, cv::Size(width, height),
                    0.0, 0.0, cv::INTER_AREA);

                frame = smaller;
            }


            // ------------------------------------------------------
            // 9. Correct the camera orientation
            // ------------------------------------------------------
            // Rotate after resizing so the operation handles fewer pixels. 
            // This is necessary because Unitree mounted the H1 camera upside down
            if (rotate_180_)
            {
                cv::Mat upright;
                cv::rotate(frame, upright, cv::ROTATE_180);
                frame = upright;
            }


            // ------------------------------------------------------
            // 10. Encode directly into the ROS message's JPEG buffer
            // ------------------------------------------------------
            sensor_msgs::msg::CompressedImage message;

            message.header.stamp = receipt_time;
            message.header.frame_id = frame_id_;

            // Format convention:
            // original pixel format; codec compressed pixel format
            message.format = "bgr8; jpeg compressed bgr8";

            const std::vector<int> encoding_parameters{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

            if (!cv::imencode(".jpg", frame, message.data, encoding_parameters))
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "JPEG encoding failed");
                return;
            }

            const auto encoded = SteadyClock::now();
            const std::size_t jpeg_bytes = message.data.size();


            // ------------------------------------------------------
            // 11. Publish and report diagnostic timings
            // ------------------------------------------------------
            publisher_->publish(message);

            const auto finished = SteadyClock::now();

            // SDK: time spent obtaining the original image.
            // Processing: decoding, resizing, rotation and JPEG encoding.
            // Callback: total elapsed time through publish().
            //
            // These are individual callback timings, not averages or
            // end-to-end latency measurements. publish() returning does
            // not mean a remote viewer has received the image.
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "%dx%d; JPEG %.1f KiB; SDK %.1f ms; "
                "processing %.1f ms; callback %.1f ms",
                frame.cols, frame.rows,
                static_cast<double>(jpeg_bytes) / 1024.0,
                milliseconds(start, acquired),
                milliseconds(acquired, encoded),
                milliseconds(start, finished));
        }
        catch (const std::exception& error)
        {
            // Recoverable C++/OpenCV exceptions are logged. This handler
            // cannot catch a segmentation fault.
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Camera error: %s", error.what());
        }
    }

    // Deferred construction preserves the SDK initialisation order.
    std::unique_ptr<unitree::robot::go2::VideoClient> video_client_;

    // Reused buffer for the SDK's encoded image response.
    std::vector<uint8_t> image_sample_;

    // Startup configuration retained for each acquisition callback.
    std::string frame_id_;
    bool rotate_180_{true};
    double scale_{0.5};
    int jpeg_quality_{70};

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char* argv[])
{
    // Initialise ROS 2 and process command-line ROS arguments.
    rclcpp::init(argc, argv);
    int exit_code = 0;

    try
    {
        auto node = std::make_shared<H1CameraPublisher>();

        // Run the node until shutdown, normally requested by Ctrl+C.
        rclcpp::spin(node);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Camera publisher failed: "
                  << error.what() << '\n';
        exit_code = 1;
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return exit_code;
}