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

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

// This script opens a live feed from the H1's onboard camera.

namespace
{
volatile std::sig_atomic_t running = 1;

void stop(int)
{
    running = 0;
}
}

int main(int argc, char* argv[])
{
    if (argc > 2)
    {
        std::cerr << "Usage: " << argv[0]
                  << " [network_interface]\n";
        return 1;
    }

    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    const std::string interface = argc == 2 ? argv[1] : "";
    const std::string window_name = "Unitree H1 camera";

    try
    {
        // An empty interface preserves the original example's behaviour.
        unitree::robot::ChannelFactory::Instance()->Init(0, interface);

        unitree::robot::go2::VideoClient video_client;
        video_client.SetTimeout(1.0f);
        video_client.Init();

        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        std::cout << "Requesting camera images.\n"
                  << "Press Q or Escape in the window, or Ctrl+C, to quit.\n";

        std::vector<uint8_t> image_sample;
        unsigned int consecutive_failures = 0;

        while (running)
        {
            image_sample.clear();
            const int ret = video_client.GetImageSample(image_sample);

            if (ret != 0 || image_sample.empty())
            {
                ++consecutive_failures;

                // Avoid printing an error for every failed request.
                if (consecutive_failures == 1 ||
                    consecutive_failures % 10 == 0)
                {
                    std::cerr << "Image request failed: code=" << ret
                              << ", bytes=" << image_sample.size()
                              << ", consecutive failures="
                              << consecutive_failures << '\n';
                }
            }
            else
            {
                consecutive_failures = 0;

                const cv::Mat frame =
                    cv::imdecode(image_sample, cv::IMREAD_COLOR);

                if (frame.empty())
                {
                    std::cerr << "Could not decode camera image.\n";
                }
                else
                {
                    cv::imshow(window_name, frame);
                }
            }

            // Process window events, including on failed requests.
            const int key = cv::waitKey(ret == 0 ? 1 : 100);
            if (key == 27 || key == 'q' || key == 'Q')
            {
                break;
            }

            if (cv::getWindowProperty(
                    window_name, cv::WND_PROP_VISIBLE) < 1)
            {
                break;
            }
        }

        cv::destroyAllWindows();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Camera viewer error: " << e.what() << '\n';
        cv::destroyAllWindows();
        return 1;
    }

    return 0;
}