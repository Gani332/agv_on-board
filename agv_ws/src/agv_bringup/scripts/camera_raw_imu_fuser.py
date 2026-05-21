#!/usr/bin/env python3
"""Fuse RealSense raw gyro/accel samples into a sensor_msgs/Imu stream."""

import threading

import rospy
from sensor_msgs.msg import Imu


class CameraRawImuFuser:
    def __init__(self):
        self.accel_topic = rospy.get_param("~accel_topic", "/camera/accel/sample")
        self.gyro_topic = rospy.get_param("~gyro_topic", "/camera/gyro/sample")
        self.output_topic = rospy.get_param("~output_topic", "/camera/imu")
        self.frame_id = rospy.get_param("~frame_id", "camera_imu_optical_frame")
        self.start_delay = float(rospy.get_param("~start_delay", 0.0))
        self.linear_accel_cov = float(rospy.get_param("~linear_accel_cov", 0.01))
        self.angular_velocity_cov = float(rospy.get_param("~angular_velocity_cov", 0.01))

        self._lock = threading.Lock()
        self._latest_accel = None
        self._latest_accel_cov = None

        self.pub = rospy.Publisher(self.output_topic, Imu, queue_size=200)
        if self.start_delay > 0.0:
            rospy.loginfo("camera_raw_imu_fuser delaying subscriptions by %.1fs", self.start_delay)
            rospy.sleep(self.start_delay)
        rospy.Subscriber(self.accel_topic, Imu, self._accel_cb, queue_size=200)
        rospy.Subscriber(self.gyro_topic, Imu, self._gyro_cb, queue_size=400)

    def _accel_cb(self, msg):
        with self._lock:
            self._latest_accel = msg.linear_acceleration
            self._latest_accel_cov = list(msg.linear_acceleration_covariance)

    def _gyro_cb(self, msg):
        with self._lock:
            if self._latest_accel is None:
                return
            accel = self._latest_accel
            accel_cov = self._latest_accel_cov

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.orientation_covariance[0] = -1.0
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = accel

        out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        if all(value == 0.0 for value in out.angular_velocity_covariance):
            out.angular_velocity_covariance = [
                self.angular_velocity_cov, 0.0, 0.0,
                0.0, self.angular_velocity_cov, 0.0,
                0.0, 0.0, self.angular_velocity_cov,
            ]

        out.linear_acceleration_covariance = accel_cov
        if all(value == 0.0 for value in out.linear_acceleration_covariance):
            out.linear_acceleration_covariance = [
                self.linear_accel_cov, 0.0, 0.0,
                0.0, self.linear_accel_cov, 0.0,
                0.0, 0.0, self.linear_accel_cov,
            ]

        self.pub.publish(out)


def main():
    rospy.init_node("camera_raw_imu_fuser")
    CameraRawImuFuser()
    rospy.loginfo("camera_raw_imu_fuser publishing fused camera IMU")
    rospy.spin()


if __name__ == "__main__":
    main()
