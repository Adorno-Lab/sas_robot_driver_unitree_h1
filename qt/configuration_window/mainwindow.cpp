#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <iomanip>

MainWindow::MainWindow(std::atomic_bool *break_loops, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow),
    st_break_loops_{break_loops},
    time_step_in_milliseconds_{100},
    elapsed_time_{0}
{
    ui->setupUi(this);
    setWindowTitle("Unitree H1 driver test");
    ui->statusbar->showMessage("Welcome!", 5000);
    timerId_ = startTimer(time_step_in_milliseconds_);

    ui->horizontalSlider_roll_->setRange(-30,30);
    ui->horizontalSlider_roll_->setTickInterval(1);
    ui->doubleSpinBox_roll_->setRange(-0.3,0.3);

    ui->horizontalSlider_pitch_->setRange(-30,30);
    ui->horizontalSlider_pitch_->setTickInterval(1);
    ui->doubleSpinBox_pitch_->setRange(-0.3,0.3);

    ui->horizontalSlider_yaw_->setRange(-60,60);
    ui->horizontalSlider_yaw_->setTickInterval(1);
    ui->doubleSpinBox_yaw_->setRange(-0.6,0.6);

    ui->verticalSlider_height_delta_->setRange(-16,16);
    ui->verticalSlider_height_delta_->setTickInterval(1);
    ui->doubleSpinBox_height_delta_->setRange(-0.16,0.16);



    ui->horizontalSlider_forward_speed_->setRange(-10,10);
    ui->horizontalSlider_forward_speed_->setTickInterval(1);
    ui->doubleSpinBox_forward_speed_->setRange(-0.1,0.1);

    ui->horizontalSlider_side_speed_->setRange(-10,10);
    ui->horizontalSlider_side_speed_->setTickInterval(1);
    ui->doubleSpinBox_side_speed_->setRange(-0.1,0.1);

    ui->horizontalSlider_yaw_->setRange(-30,30);
    ui->horizontalSlider_yaw_->setTickInterval(1);
    ui->doubleSpinBox_yaw_->setRange(-0.3,0.3);


    ui->dial_select_robot_->setRange(1,3);
    ui->dial_select_robot_->setValue(2);

    ui->dial_change_operation_high_level_mode_->setRange(0,1);
    ui->dial_change_operation_high_level_mode_->setValue(0);

    //ui->doubleSpinBox_read_vx_->setRange(-5,5);
    //ui->doubleSpinBox_read_vy_->setRange(-5,5);
    //ui->doubleSpinBox_read_wz_->setRange(-5,5);

    _config_spin_boxes_as_read_only({ui->doubleSpinBox_read_vx_,
                                     ui->doubleSpinBox_read_vy_,
                                     ui->doubleSpinBox_read_wz_,
                                     ui->doubleSpinBox_read_roll_,
                                     ui->doubleSpinBox_read_pitch_,
                                     ui->doubleSpinBox_read_yaw_,
                                     ui->doubleSpinBox_read_height_,
                                     ui->doubleSpinBox_computed_roll_,
                                     ui->doubleSpinBox_computed_pitch_,
                                     ui->doubleSpinBox_computed_yaw_,
                                     ui->doubleSpinBox_computed_rel_roll_,
                                     ui->doubleSpinBox_computed_rel_pitch_,
                                     ui->doubleSpinBox_computed_rel_yaw_
                                     });


    ui->pushButton_connect_->setEnabled(false);
    ui->pushButton_initialize_->setEnabled(false);
    ui->pushButton_deinitialize_->setEnabled(false);
    ui->pushButton_disconnect_->setEnabled(false);

    ui->pushButton_zero_commands_->setStyleSheet(pause_yellow_color_);

    connect(ui->pushButton_zero_commands_, &QPushButton::clicked,
            this,&MainWindow::_set_zero_commands);

    connect(ui->horizontalSlider_roll_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_roll);
    connect(ui->horizontalSlider_pitch_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_pitch);
    connect(ui->horizontalSlider_yaw_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_yaw);
    connect(ui->verticalSlider_height_delta_, &QSlider::sliderMoved,
            this, &MainWindow::update_vertical_slider_height);


    connect(ui->horizontalSlider_forward_speed_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_forward_speed);
    connect(ui->horizontalSlider_side_speed_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_side_speed);
    connect(ui->horizontalSlider_yaw_speed_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_yaw_speed);


    connect(ui->pushButton_connect_, &QPushButton::clicked,
            this, &MainWindow::_connect);
    connect(ui->pushButton_initialize_, &QPushButton::clicked,
            this, &MainWindow::_initialize);
    connect(ui->pushButton_deinitialize_, &QPushButton::clicked,
            this, &MainWindow::_deinitialize);
    connect(ui->pushButton_disconnect_, &QPushButton::clicked,
            this, &MainWindow::_disconnect);

    connect(ui->dial_select_robot_, &QDial::valueChanged,
            this, &MainWindow::update_dial_select_robot);
    connect(ui->dial_change_operation_high_level_mode_, &QDial::valueChanged,
            this, &MainWindow::update_dial_change_operation_high_level_mode);

}

MainWindow::~MainWindow()
{
    delete ui;
    killTimer(timerId_);
    _deinitialize();
    _disconnect();
    std::cerr<<"destructor called"<<std::endl;
}

void MainWindow::update_horizontal_slider_roll()
{
    target_roll_  = ui->horizontalSlider_roll_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_roll_->setValue(target_roll_);

}

void MainWindow::update_horizontal_slider_pitch()
{
    target_pitch_ = ui->horizontalSlider_pitch_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_pitch_->setValue(target_pitch_);
}

void MainWindow::update_horizontal_slider_yaw()
{
    target_yaw_ = ui->horizontalSlider_yaw_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_yaw_->setValue(target_yaw_);
}

void MainWindow::update_vertical_slider_height()
{
    target_height_ = ui->verticalSlider_height_delta_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_height_delta_->setValue(target_height_);
}

void MainWindow::update_horizontal_slider_forward_speed()
{
    target_forward_speed_ = ui->horizontalSlider_forward_speed_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_forward_speed_->setValue(target_forward_speed_);
}

void MainWindow::update_horizontal_slider_side_speed()
{
    target_side_speed_ = ui->horizontalSlider_side_speed_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_side_speed_->setValue(target_side_speed_);
}

void MainWindow::update_horizontal_slider_yaw_speed()
{
    target_yaw_speed_ = ui->horizontalSlider_yaw_speed_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_yaw_speed_->setValue(target_yaw_speed_);
}

void MainWindow::update_dial_select_robot()
{
    int sr = ui->dial_select_robot_->sliderPosition();

    std::string ip = "192.168.8.170";

    if (sr == 1) //White robot
    {
        configuration_.ROBOT_IP = "192.168.8.170";
        configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE = false;
        configuration_.ROBOT_PORT = 8082;
        configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO = false;
        ui->label_ip_->setText(QString(("IP: "+configuration_.ROBOT_IP).c_str()));
        ui->pushButton_connect_->setEnabled(true);
    }else if (sr == 2)
    {
        ui->pushButton_connect_->setEnabled(false);
    }else if (sr == 3) //Black robot
    {
        configuration_.ROBOT_IP = "192.168.8.226";
        configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE = false;
        configuration_.ROBOT_PORT = 8082;
        configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO = true; //true
        ui->label_ip_->setText(QString(("IP: "+configuration_.ROBOT_IP).c_str()));
        ui->pushButton_connect_->setEnabled(true);
    }



}

void MainWindow::update_dial_change_operation_high_level_mode()
{
    if (unitree_h1_driver_)
    {
        int dial = ui->dial_change_operation_high_level_mode_->sliderPosition();
        if (dial == 0) //walking
            unitree_h1_driver_->request_change_in_high_level_control(DriverUnitreeH1::HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING);
        if (dial == 1)
            unitree_h1_driver_->request_change_in_high_level_control(DriverUnitreeH1::HIGH_LEVEL_MODE::FORCED_STAND);
    }
}

void MainWindow::timerEvent([[maybe_unused]] QTimerEvent *event)
{
    elapsed_time_ += time_step_in_milliseconds_;

    // Convert to seconds
    int total_seconds = static_cast<int>(elapsed_time_ / 1000.0);
    // Calculate hours, minutes, seconds
    int hours = total_seconds / 3600;
    int minutes = (total_seconds % 3600) / 60;
    int seconds = total_seconds % 60;

    // Format as hh:mm:ss
    QString time_string = QString("%1:%2:%3")
                              .arg(hours, 2, 10, QChar('0'))
                              .arg(minutes, 2, 10, QChar('0'))
                              .arg(seconds, 2, 10, QChar('0'));

  //  qDebug() << "target_roll: " << target_roll_ << " target_pitch: "<<target_pitch_<<" target_yaw: "<<target_yaw_;
  //  qDebug() << "vx: " << target_forward_speed_ << " vy: "<<target_side_speed_<<" vw: "<<target_yaw_speed_;
    ui->elapsed_time_label_->setText(time_string);

    //update_horizontalSlider_roll();
    if (unitree_h1_driver_)
    {
        std::string status_msg = unitree_h1_driver_->get_status_message();
        ui->label_status_->setText(QString(status_msg.c_str()));

        unitree_h1_driver_->set_high_level_speed(target_forward_speed_, target_side_speed_, target_yaw_speed_);
        unitree_h1_driver_->set_forced_stand_commands(target_roll_, target_pitch_, target_yaw_, target_height_);

        VectorXd w = unitree_h1_driver_->get_high_level_angular_velocity().vec3();
        VectorXd v   = unitree_h1_driver_->get_high_level_linear_velocity().vec3();



        if (w(2) > max_read_vx_)
            max_read_wz_ = w(2);
        if (v(0) > max_read_vx_)
            max_read_vx_ = v(0);
        if (v(1) > max_read_vy_)
            max_read_vy_ = v(1);


            max_filtered_read_vy_ = v(1);



        // compute Euler angles
        auto r_0b = unitree_h1_driver_->get_IMU_orientation();
       // Eigen::Quaterniond q1(r(0), r(1), r(2), r(3));
       // Eigen::Matrix3d R1 = q1.toRotationMatrix();


        // Method B: Manual extraction
        //double roll_manual  = std::atan2(R1(2, 1), R1(2, 2));
        //double pitch_manual = std::atan2(-R1(2, 0),
         //                                std::sqrt(R1(2, 1)*R1(2, 1) + R1(2, 2)*R1(2, 2)));
       // double yaw_manual   = std::atan2(R1(1, 0), R1(0, 0));

        Eigen::Vector3d  rpy_from_orientation = _compute_euler_angles_from_unit_quaternion(r_0b);

        // Your existing RPY angles from driver
        Eigen::Vector3d rpy_from_driver = unitree_h1_driver_->get_IMU_rpy_angles();

        DQ r_0f = unitree_h1_driver_->get_last_IMU_orientation_when_robot_stopped();
        //std::cout <<" r_0f: "<<r_0f<<std::endl;
        DQ r_fb = r_0f.conj()*r_0b;

        Eigen::Vector3d rpy_rel = _compute_euler_angles_from_unit_quaternion(r_fb);


        ui->doubleSpinBox_computed_rel_roll_->setValue(rpy_rel.x());
        ui->doubleSpinBox_computed_rel_pitch_->setValue(rpy_rel.y());
        ui->doubleSpinBox_computed_rel_yaw_->setValue(rpy_rel.z());


        ui->doubleSpinBox_computed_roll_->setValue(rpy_from_orientation.x());
        ui->doubleSpinBox_computed_pitch_->setValue(rpy_from_orientation.y());
        ui->doubleSpinBox_computed_yaw_->setValue(rpy_from_orientation.z());

        // Print comparison
        std::cout << "========== RPY ANGLE COMPARISON ==========" << std::endl;
        std::cout << std::fixed << std::setprecision(6);
        std::cout << std::setw(15) << "Method"
                  << std::setw(15) << "Roll (X)"
                  << std::setw(15) << "Pitch (Y)"
                  << std::setw(15) << "Yaw (Z)" << std::endl;
        std::cout << "----------------------------------------------" << std::endl;


        std::cout << std::setw(15) << "Manual Extraction"
                  << std::setw(15) << rpy_from_orientation.x()
                  << std::setw(15) << rpy_from_orientation.y()
                  << std::setw(15) << rpy_from_orientation.z()  << " rad" << std::endl;

        std::cout << std::setw(15) << "Driver Direct"
                  << std::setw(15) << rpy_from_driver.x()
                  << std::setw(15) << rpy_from_driver.y()
                  << std::setw(15) << rpy_from_driver.z() << " rad" << std::endl;

        std::cout << std::setw(15) << "Relative"
                  << std::setw(15) << rpy_rel.x()
                  << std::setw(15) << rpy_rel.y()
                  << std::setw(15) << rpy_rel.z()  << " rad" << std::endl;

        std::cout << "==============================================" << std::endl;


        ui->doubleSpinBox_read_vx_->setValue(v(0));
        ui->doubleSpinBox_read_vy_->setValue(v(1));
        ui->doubleSpinBox_read_wz_->setValue(w(2));




        ui->progressBar_battery->setValue(unitree_h1_driver_->get_state_of_charge());

        std::string current_mode = unitree_h1_driver_->high_level_mode_to_string(
                                   unitree_h1_driver_->get_current_high_mode());
        std::string target_mode  = unitree_h1_driver_->high_level_mode_to_string(
                                   unitree_h1_driver_->get_target_high_mode());
        std::string gait_type  =   unitree_h1_driver_->gait_type_to_string(
                                   unitree_h1_driver_->get_current_gait_type());

        ui->pushButton_target_mode_->setText(QString(target_mode.c_str()));
        ui->pushButton_current_mode_->setText(QString(current_mode.c_str()));
        ui->pushButton_gait_type_->setText(QString(gait_type.c_str()));

        Vector3d rpy = unitree_h1_driver_->get_IMU_rpy_angles();
        ui->doubleSpinBox_read_roll_->setValue(rpy.x());
        ui->doubleSpinBox_read_pitch_->setValue(rpy.y());
        ui->doubleSpinBox_read_yaw_->setValue(rpy.z());

        ui->doubleSpinBox_read_height_->setValue(unitree_h1_driver_->get_body_height());








    }
}

void MainWindow::_set_zero_commands()
{
    target_roll_ = 0;
    target_pitch_ = 0;
    target_yaw_ = 0;
    target_height_ = 0;

    target_forward_speed_ = 0;
    target_side_speed_ = 0;
    target_yaw_speed_ = 0;

    ui->horizontalSlider_roll_->setSliderPosition(0);
    ui->doubleSpinBox_roll_->setValue(0);

    ui->horizontalSlider_pitch_->setSliderPosition(0);
    ui->doubleSpinBox_pitch_->setValue(0);

    ui->horizontalSlider_yaw_->setSliderPosition(0);
    ui->doubleSpinBox_yaw_->setValue(0);

    ui->verticalSlider_height_delta_->setSliderPosition(0);
    ui->doubleSpinBox_height_delta_->setValue(0);


    ui->horizontalSlider_forward_speed_->setSliderPosition(0);
    ui->doubleSpinBox_forward_speed_->setValue(0);

    ui->horizontalSlider_side_speed_->setSliderPosition(0);
    ui->doubleSpinBox_side_speed_->setValue(0);

    ui->horizontalSlider_yaw_speed_->setSliderPosition(0);
    ui->doubleSpinBox_yaw_speed_->setValue(0);

}

void MainWindow::_connect()
{
    ui->pushButton_connect_->setEnabled(false);
    ui->pushButton_initialize_->setEnabled(true);
    ui->dial_select_robot_->setEnabled(false);

    if (!unitree_h1_driver_)
    {
        std::vector<DriverUnitreeH1::CUSTOM_FLAGS> custom_flags;
        if (configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO)
            custom_flags.push_back(DriverUnitreeH1::CUSTOM_FLAGS::FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO);
        unitree_h1_driver_ = std::make_shared<DriverUnitreeH1>(st_break_loops_,
                                          DriverUnitreeH1::MODE::VelocityControl, // Driver mode
                                          DriverUnitreeH1::LEVEL::HIGH,       // Level mode
                                          true,   //verbosity
                                          2000,   // TIMEOUT in ms
                                          configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE, // LIE DOWN ROBOT WHEN DEINITIALIZE
                                          configuration_.ROBOT_IP,  // Target IP   //192.168.123.10 for low-level mode
                                          configuration_.ROBOT_PORT,// Target port  //8007 for low-level mode
                                          8090,
                                          custom_flags);

    }
    unitree_h1_driver_->connect();
}

void MainWindow::_initialize()
{
    ui->pushButton_initialize_->setEnabled(false);
    ui->pushButton_deinitialize_->setEnabled(true);
    if (unitree_h1_driver_)
    {
        unitree_h1_driver_->initialize();
    }
}

void MainWindow::_deinitialize()
{
    ui->pushButton_deinitialize_->setEnabled(false);
    ui->pushButton_disconnect_->setEnabled(true);
    if (unitree_h1_driver_)
        unitree_h1_driver_->deinitialize();
}

void MainWindow::_disconnect()
{

    ui->pushButton_disconnect_->setEnabled(false);
    if (unitree_h1_driver_)
        unitree_h1_driver_->disconnect();
}

void MainWindow::_config_spin_boxes_as_read_only(const std::vector<QDoubleSpinBox *> &spinboxes)
{
    for (size_t i = 0; i < spinboxes.size(); ++i) {
        spinboxes.at(i)->setReadOnly(true);
        spinboxes.at(i)->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spinboxes.at(i)->setAlignment(Qt::AlignCenter); // Optional: center align
        spinboxes.at(i)->setDecimals(2);  // Set precision to 3 decimals
        spinboxes.at(i)->setRange(-500,500);
    }
}

Vector3d MainWindow::_compute_euler_angles_from_unit_quaternion(const DQ &r) const
{
    VectorXd rv = r.vec4();
    Eigen::Quaterniond q(rv(0), rv(1), rv(2), rv(3));
    Eigen::Matrix3d R1 = q.toRotationMatrix();


    // Method B: Manual extraction
    double roll_manual  = std::atan2(R1(2, 1), R1(2, 2));
    double pitch_manual = std::atan2(-R1(2, 0),
                                     std::sqrt(R1(2, 1)*R1(2, 1) + R1(2, 2)*R1(2, 2)));
    double yaw_manual   = std::atan2(R1(1, 0), R1(0, 0));


    // Return as Vector3d (roll, pitch, yaw) - radians
    Vector3d euler_angles;
    euler_angles << roll_manual, pitch_manual, yaw_manual;

    return euler_angles;
}



