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
# ################################################################*/

#pragma once
#include <QMainWindow>
#include <qspinbox.h>
#include "DriverUnitreeH1.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    struct RobotConfiguration
    {
        std::string mode;              //const std::string mode= "PositionControl";
        bool LIE_DOWN_ROBOT_WHEN_DEINITIALIZE;  //std::string LIE_DOWN_ROBOT_WHEN_DEINITIALIZE
        std::string ROBOT_IP; //"192.168.123.220",  // Target IP   //192.168.123.10 for low-level mode
        int ROBOT_PORT; //    8082,              // Target port  //8007 for low-level mode
        std::string robot_name;
        bool FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO; // to handle this https://github.com/Adorno-Lab/sas_robot_driver_unitree_h1/issues/4
    };
    RobotConfiguration configuration_;
    std::shared_ptr<DriverUnitreeH1> unitree_h1_driver_;
    std::atomic_bool* st_break_loops_;

public:
    MainWindow(std::atomic_bool *break_loops, QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void update_horizontal_slider_roll();
    void update_horizontal_slider_pitch();
    void update_horizontal_slider_yaw();
    void update_vertical_slider_height();


    void update_horizontal_slider_forward_speed();
    void update_horizontal_slider_side_speed();
    void update_horizontal_slider_yaw_speed();

    void update_dial_select_robot();
    void update_dial_change_operation_high_level_mode();



private:
    Ui::MainWindow *ui;
    void timerEvent(QTimerEvent *event);
    int timerId_;
    int time_step_in_milliseconds_;
    double elapsed_time_;

    double target_roll_{0};
    double target_pitch_{0};
    double target_yaw_{0};
    double target_height_{0};

    double target_forward_speed_{0};
    double target_side_speed_{0};
    double target_yaw_speed_{0};

    double max_read_vx_{0};
    double max_read_vy_{0};
    double max_read_wz_{0};

    double max_filtered_read_vx_{0};
    double max_filtered_read_vy_{0};
    double max_filtered_read_wz_{0};

    const double slider_factor_ = 100.0;

    QString pause_yellow_color_ = "background-color: #fcca03;";

    void _set_zero_commands();

    void _connect();
    void _initialize();
    void _deinitialize();
    void _disconnect();

    void _config_spin_boxes_as_read_only(const std::vector<QDoubleSpinBox *> &spinboxes);


    Eigen::Vector3d _compute_euler_angles_from_unit_quaternion(const DQ& r) const;




};
