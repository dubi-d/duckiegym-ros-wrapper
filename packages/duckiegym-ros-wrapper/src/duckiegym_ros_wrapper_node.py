#!/usr/bin/env python3

import rospy
import os
import cv2
from cv_bridge import CvBridge
import numpy as np

import gym_duckietown
from gym_duckietown.simulator import Simulator

from duckietown.dtros import DTROS, NodeType

from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


class DuckiegymRosWrapperNode(DTROS):
    def __init__(self, node_name):
        super(DuckiegymRosWrapperNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.action = [0.1, 0.1]  # wheel commands for simulator

        self.bridge = CvBridge()

        vehicle_name = os.environ.get("VEHICLE_NAME")
        image_topic = "/" + vehicle_name + "/camera_node/image/compressed"
        wheels_cmd_topic = "/" + vehicle_name + "/wheel_driver_node/wheels_cmd_executed"
        self.pub_img = rospy.Publisher(image_topic, CompressedImage)
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
            self.publish_camera_image(observation)
            self.sim.render()
            if done:
                self.sim.reset()

    def callback_wheels_cmd(self, msg):
        self.log("I heard " + str(msg))

    def publish_camera_image(self, observation):
        img = self.bridge.cv2_to_compressed_imgmsg(cv2.cvtColor(observation, cv2.COLOR_RGB2BGR), dst_format="jpg")
        img.header.stamp = rospy.Time.now()
        self.pub_img.publish(img)


if __name__ == '__main__':
    duckiegym_ros_wrapper_node = DuckiegymRosWrapperNode("duckiegym_ros_wrapper_node")
    duckiegym_ros_wrapper_node.run_simulator()
    rospy.spin()