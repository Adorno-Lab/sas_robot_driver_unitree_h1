#!/usr/bin/env python3

import csv
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

# ============================================================
# USER SETTINGS
# ============================================================

ROBOT_NAME = "P_Body"
# ROBOT_NAME = "Dummy"

# Logging frequency [Hz]
LOG_FREQUENCY = 10.0

# ============================================================


class ExperimentLogger(Node):

    def __init__(self):

        super().__init__("experiment_logger")

        imu_topic = f"/sas_h1/{ROBOT_NAME}/get/IMU_state"
        joint_topic = f"/sas_h1/{ROBOT_NAME}/get/joint_states"
        temps_topic = f"/sas_h1/{ROBOT_NAME}/get/temperatures"

        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(JointState, joint_topic, self.joint_callback, 10)
        self.create_subscription(
            Float64MultiArray, temps_topic, self.temps_callback, 10
        )

        self.latest_imu = None
        self.latest_joint = None
        self.latest_temps = None

        self.logging_enabled = False

        self.csv_file = None
        self.csv_writer = None

        self.log_start_time = None

        self.header_written = False
        self.joint_names = [
            "L_Sho_R",  # Left Shoulder Roll
            "L_Sho_P",  # Left Shoulder Pitch
            "L_Sho_Y",  # Left Shoulder Yaw
            "L_Elb",  # Left Elbow
            "R_Sho_R",  # Right Shoulder Roll
            "R_Sho_P",  # Right Shoulder Pitch
            "R_Sho_Y",  # Right Shoulder Yaw
            "R_Elb",  # Right Elbow
            "Waist",  # Waist
        ]
        self.temps_joint_names = [
            "IMU",
            "L_Sho_R",
            "L_Sho_P",
            "L_Sho_Y",
            "L_Elb",
            "R_Sho_R",
            "R_Sho_P",
            "R_Sho_Y",
            "R_Elb",
            "Waist",
            "L_Hip_R",
            "L_Hip_P",
            "L_Hip_Y",
            "L_Knee",
            "L_Ankle",
            "R_HipR",
            "R_HipP",
            "R_HipY",
            "R_Knee",
            "R_Ankle",
        ]

        self.lock = threading.Lock()

        self.create_timer(1.0 / LOG_FREQUENCY, self.log_sample)

        self.get_logger().info(f"Subscribed to {imu_topic}")
        self.get_logger().info(f"Subscribed to {joint_topic}")
        self.get_logger().info(f"Subscribed to {temps_topic}")
        self.get_logger().info(f"Logging frequency: {LOG_FREQUENCY} Hz")

    # =====================================================
    # Topic Callbacks
    # =====================================================

    def imu_callback(self, msg):
        self.latest_imu = msg

    def joint_callback(self, msg):
        self.latest_joint = msg

    def temps_callback(self, msg):
        self.latest_temps = msg

    # =====================================================
    # Logging Control
    # =====================================================

    def start_logging(self, filename):

        if self.latest_joint is None:
            print(
                "\nNo log data is being published yet. Start the drivers and try again.\n"
            )
            return False

        filename = Path(filename).with_suffix(".csv")

        self.csv_file = open(filename, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.write_header()

        self.log_start_time = time.perf_counter()

        self.logging_enabled = True

        self.get_logger().info(f"Logging started -> {filename}")

        return True

    def stop_logging(self):

        self.logging_enabled = False

        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        self.header_written = False

        self.get_logger().info("Logging stopped")

    # =====================================================
    # CSV Header
    # =====================================================

    def write_header(self):

        header = [
            "time",
            "imu_qw",
            "imu_qx",
            "imu_qy",
            "imu_qz",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "accel_x",
            "accel_y",
            "accel_z",
        ]

        for joint in self.joint_names:
            header.append(f"{joint}_pos")

        for joint in self.joint_names:
            header.append(f"{joint}_vel")

        for joint in self.joint_names:
            header.append(f"{joint}_eff")

        for joint in self.temps_joint_names:
            header.append(f"{joint}_temp")

        self.csv_writer.writerow(header)

        self.header_written = True

    # =====================================================
    # Logging Timer
    # =====================================================

    def log_sample(self):

        if not self.logging_enabled:
            return

        if self.latest_imu is None:
            return

        if self.latest_joint is None:
            return

        imu = self.latest_imu
        joint = self.latest_joint
        temps = self.latest_temps

        t = time.perf_counter() - self.log_start_time

        row = [
            t,
            imu.orientation.w,
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ]

        row.extend(joint.position)
        row.extend(joint.velocity)
        row.extend(joint.effort)

        row.extend(temps.data)

        self.csv_writer.writerow(row)

    # =====================================================


def spin_ros(node):
    rclpy.spin(node)


def main():

    rclpy.init()

    node = ExperimentLogger()

    spin_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)

    spin_thread.start()

    try:

        while True:

            filename = input(
                "\nEnter filename " "(without extension, q to quit): "
            ).strip()

            if filename.lower() == "q":
                break

            if not filename:
                continue

            input("\nPress ENTER to begin logging...")

            success = node.start_logging(filename)

            if not success:
                continue

            input("Logging...\n" "Press ENTER to stop.")

            node.stop_logging()

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_logging()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
