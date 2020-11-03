#!/usr/bin/env python3

import rospy
import os
import cv2
import yaml
from cv_bridge import CvBridge
import numpy as np

import gym_duckietown
from gym_duckietown.simulator import Simulator

from duckietown.dtros import DTROS, NodeType

from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped


class DuckiegymRosWrapperNode(DTROS):
    """
    Run a gym-duckietown simulator, redirect the simulator's image to a ros topic, redirect the wheel commands
    to simulator actions.
    """
    def __init__(self, node_name):
        super(DuckiegymRosWrapperNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.action = [0, 0]  # wheel commands for simulator
        self.camera_info = self.load_camera_info("/data/config/calibrations/camera_intrinsic/default.yaml")

        self.bridge = CvBridge()

        vehicle_name = os.environ.get("VEHICLE_NAME")  # ToDo put in launch file / launcher arg?
        image_topic = "/" + vehicle_name + "/camera_node/image/compressed"
        self.pub_img = rospy.Publisher(image_topic, CompressedImage)
        camera_info_topic = "/" + vehicle_name + "/camera_node/camera_info"
        self.pub_camera_info = rospy.Publisher(camera_info_topic, CameraInfo)
        wheels_cmd_topic = "/" + vehicle_name + "/wheels_driver_node/wheels_cmd"
        self.sub_wheels_cmd = rospy.Subscriber(wheels_cmd_topic, WheelsCmdStamped, self.callback_wheels_cmd, queue_size=1)

        self.sim = Simulator(
            seed=123,  # random seed
            map_name="loop_empty",
            max_steps=500001,  # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4,  # start close to straight
            full_transparency=True,
            distortion=False,
        )
        self.log("SKKKKKKKKKKKKKKKKKKKKKRRRRRRRRRRRRRRRRRRRRRA")

    def run_simulator(self):
        while not rospy.is_shutdown():
            observation, reward, done, misc = self.sim.step(self.action)
            self.publish_camera_info()
            self.publish_camera_image(observation)
            self.sim.render()
            if done:
                self.sim.reset()

    def callback_wheels_cmd(self, msg):
        self.action = [msg.vel_left, msg.vel_right]

    def publish_camera_image(self, observation):
        img = self.bridge.cv2_to_compressed_imgmsg(cv2.cvtColor(observation, cv2.COLOR_RGB2BGR), dst_format="jpg")
        img.header.stamp = rospy.Time.now()
        self.pub_img.publish(img)

    def publish_camera_info(self):
        self.camera_info.header.stamp = rospy.Time.now()
        self.pub_camera_info.publish(self.camera_info)

    def load_camera_info(self, filename):
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


if __name__ == '__main__':
    duckiegym_ros_wrapper_node = DuckiegymRosWrapperNode("duckiegym_ros_wrapper_node")
    duckiegym_ros_wrapper_node.run_simulator()
    rospy.spin()