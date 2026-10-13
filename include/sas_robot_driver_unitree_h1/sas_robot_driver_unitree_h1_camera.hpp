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

#pragma once
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct UnitreeH1CameraConfiguration
{
    std::string network_interface;
    std::string robot_name;
    double publish_rate;
    double scale;
    int jpeg_quality;
};

class UnitreeH1CameraHandler
{
private:
    std::string topic_prefix_;
    UnitreeH1CameraConfiguration configuration_;
    std::shared_ptr<rclcpp::Node> node_;
    double timer_period_;
    int print_count_;
    std::string network_interface_;
    double publish_rate_;

    std::unique_ptr<unitree::robot::go2::VideoClient> video_client_;
    std::vector<uint8_t> image_sample_;

    std::string frame_id_;
    std::string topic_name_;
    double scale_;
    int jpeg_quality_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    static double milliseconds_(std::chrono::steady_clock::time_point start,std::chrono::steady_clock::time_point end);

    void publish_frame_();

public:
    UnitreeH1CameraHandler(std::shared_ptr<rclcpp::Node> node, const UnitreeH1CameraConfiguration& configuration);

};

