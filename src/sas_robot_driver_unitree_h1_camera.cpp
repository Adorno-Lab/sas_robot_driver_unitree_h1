/*
# Copyright (c) 2026-2026 Adorno-Lab software developments
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
#   Author: Daniel S. J. Derwent, email: daniel.derwent@manchester.ac.uk
#
# ################################################################
*/

#include <sas_robot_driver_unitree_h1/sas_robot_driver_unitree_h1_camera.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <utility>

double UnitreeH1CameraHandler::milliseconds_(std::chrono::steady_clock::time_point start,
                                          std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void UnitreeH1CameraHandler::publish_frame_()
{
    try
    {
        const auto start = std::chrono::steady_clock::now();

        image_sample_.clear();
        const int result = video_client_->GetImageSample(image_sample_);
        const auto acquired = std::chrono::steady_clock::now();

        if (result != 0 || image_sample_.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "Camera request failed: code=%d, bytes=%zu",
                result, image_sample_.size());
            return;
        }

        const auto receipt_time = node_->now();
        cv::Mat frame = cv::imdecode(image_sample_, cv::IMREAD_COLOR);

        if (frame.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000, "Could not decode SDK image");
            return;
        }

        if (scale_ < 1.0)
        {
            const int new_width = std::max(1, static_cast<int>(std::lround(frame.cols * scale_)));
            const int new_height = std::max(1, static_cast<int>(std::lround(frame.rows * scale_)));

            cv::Mat smaller;
            cv::resize(frame, smaller, cv::Size(new_width, new_height), 0.0, 0.0, cv::INTER_AREA);
            frame = smaller;
        }

        cv::Mat upright;
        cv::rotate(frame, upright, cv::ROTATE_180);
        frame = upright;

        sensor_msgs::msg::CompressedImage message;
        message.header.stamp = receipt_time;
        message.header.frame_id = frame_id_;
        message.format = "bgr8; jpeg compressed bgr8";

        const std::vector<int> encoding_parameters{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

        if (!cv::imencode(".jpg", frame, message.data, encoding_parameters))
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000, "JPEG encoding failed");
            return;
        }

        const auto encoded = std::chrono::steady_clock::now();
        const std::size_t jpeg_bytes = message.data.size();

        publisher_->publish(message);

        const auto finished = std::chrono::steady_clock::now();

        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "%dx%d; JPEG %.1f KiB; SDK %.1f ms; "
            "processing %.1f ms; callback %.1f ms",
            frame.cols, frame.rows,
            static_cast<double>(jpeg_bytes) / 1024.0,
            milliseconds_(start, acquired),
            milliseconds_(acquired, encoded),
            milliseconds_(start, finished));
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "[UnitreeH1CameraHandler::publish_frame_] Camera error: %s",
            error.what());
    }
}

UnitreeH1CameraHandler::UnitreeH1CameraHandler(
    std::shared_ptr<rclcpp::Node> node,
    const UnitreeH1CameraConfiguration& configuration)
    : topic_prefix_(configuration.robot_name),
      configuration_(configuration),
      node_(std::move(node)),
      timer_period_(0.002),
      print_count_(0),
      network_interface_(configuration.network_interface),
      publish_rate_(configuration.publish_rate),
      scale_(configuration.scale),
      jpeg_quality_(configuration.jpeg_quality)
{

    topic_name_ = topic_prefix_ + "/get/camera_feed";
    frame_id_ = topic_prefix_ + "_camera_frame";

    // Validate arguments
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0)
    {
        throw std::invalid_argument("publish_rate must be finite and positive");
    }

    if (!std::isfinite(scale_) || scale_ <= 0.0 || scale_ > 1.0)
    {
        throw std::invalid_argument("scale must be greater than 0 and no greater than 1");
    }

    if (jpeg_quality_ < 0 || jpeg_quality_ > 100)
    {
        throw std::invalid_argument("jpeg_quality must be between 0 and 100");
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));
    if (period.count() <= 0)
    {
        throw std::invalid_argument("publish_rate is too large");
    }

    // Initialise the channel factory 
    RCLCPP_INFO(
        node_->get_logger(), "Initialising camera SDK; network_interface=%s",
        network_interface_.c_str());
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

    // Create the video client
    video_client_ = std::make_unique<unitree::robot::go2::VideoClient>();
    video_client_->SetTimeout(1.0f);
    video_client_->Init();

    // Create the publisher
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    publisher_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name_, qos);

    // Setup timer callback to publish most recent frame at a regular rate
    timer_ = node_->create_wall_timer(
        period, std::bind(&UnitreeH1CameraHandler::publish_frame_, this));

    RCLCPP_INFO(node_->get_logger(),
                "Camera topic=%s; requested rate=%.1f Hz; scale=%.2f; JPEG quality=%d",
                topic_name_.c_str(), publish_rate_, scale_, jpeg_quality_);
}
