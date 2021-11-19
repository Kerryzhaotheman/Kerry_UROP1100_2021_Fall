#! /usr/bin/env python

import rospy
from geometry_msgs.msg import *
from vicon_bridge.msg import *
from socket import *
import math
import json
import time

class ViconDecode:
    def __init__(self):
        # declare initial location
        self.start_point = [0, 0, 0]
        self.start_angle = [0, 0, 0]
        self.flag1 = True
        # current position of turtle1
        self.tur1_pos = [0, 0, 0]
        self.tur1_ang = [0, 0, 0]
        # position of obstacles
        self.obs_pos = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), \
        (0, 0), (0, 0), (0, 0)]
        # get tag position
        self.vicon_pose1 = TransformStamped()
        self.vicon_obs_pose = Markers()
        # subscribers
        self.vicon_subsriber1 = rospy.Subscriber \
        ("/vicon/turtle1/turtle1", TransformStamped, self.viconCB1)
        self.vicon_subsriber2 = rospy.Subscriber \
        ("/vicon/markers", Markers, self.viconCB_obs)
        # loop frequency
        rate = rospy.Rate(20)
        self.start_time = time.time()
        #self.goal = [0.75,0.54]

        #### main loop ####
        while not rospy.is_shutdown():
            print("Pos:", round(self.tur1_pos[0], 4), ",", round\
            (self.tur1_pos[1], 4))
            print("Ang:", round(self.tur1_ang[0], 4))

            print("obs1:", round(self.obs_pos[0][0], 4), ",", round\
            (self.obs_pos[0][1], 4))
            print("obs2:", round(self.obs_pos[1][0], 4), ",", round\
            (self.obs_pos[1][1], 4))
            print("obs3:", round(self.obs_pos[2][0], 4), ",", round\
            (self.obs_pos[2][1], 4))
            print("obs4:", round(self.obs_pos[3][0], 4), ",", round\
            (self.obs_pos[3][1], 4))
            print("obs5:", round(self.obs_pos[4][0], 4), ",", round\
            (self.obs_pos[4][1], 4))
            print("obs6:", round(self.obs_pos[5][0], 4), ",", round\
            (self.obs_pos[5][1], 4))
            print("obs7:", round(self.obs_pos[6][0], 4), ",", round\
            (self.obs_pos[6][1], 4))
            print("obs8:", round(self.obs_pos[7][0], 4), ",", round\
            (self.obs_pos[7][1], 4))
            print("obs9:", round(self.obs_pos[8][0], 4), ",", round\
            (self.obs_pos[8][1], 4))

            print("\n")
            rate.sleep()

    def viconCB1(self, msg):
        # get tags position
        self.vicon_pose1 = msg
        x = self.vicon_pose1.transform.translation.x
        y = self.vicon_pose1.transform.translation.y
        z = self.vicon_pose1.transform.translation.z
        quat_x = self.vicon_pose1.transform.rotation.x
        quat_y = self.vicon_pose1.transform.rotation.y
        quat_z = self.vicon_pose1.transform.rotation.z
        quat_w = self.vicon_pose1.transform.rotation.w
        angle = self.quat_to_euler_angle(quat_x, quat_y, quat_z, quat_w)
        self.tur1_pos = [x,y,z]
        self.tur1_ang = angle

        if self.flag1:
            self.start_point = [x, y, z]
            self.start_angle = angle
        self.flag1 = False

    def viconCB_obs(self, msg):
        self.vicon_obs_pose = msg
        for i in self.vicon_obs_pose.markers:
            if i.marker_name == "obs1":
                self.obs_pos[0] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs2":
                self.obs_pos[1] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs3":
                self.obs_pos[2] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs4":
                self.obs_pos[3] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs5":
                self.obs_pos[4] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs6":
                self.obs_pos[5] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs7":
                self.obs_pos[6] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs8":
                self.obs_pos[7] = (i.translation.x / 1000, i.translation.y / 1000)
            if i.marker_name == "obs9":
                self.obs_pos[8] = (i.translation.x / 1000, i.translation.y / 1000)


    def quat_to_euler_angle(self, x, y, z, w):
        t0 = -2.0 * (x * y - w * z)
        t1 = w * w + x * x - y * y - z * z
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (x * z + w * y)
        pitch_y = math.asin(t2)

        t3 = -2.0 * (y * z - w * x)
        t4 = w * w - x * x - y * y + z * z
        yaw_z = math.atan2(t3, t4)

        x = (roll_x/3.14159)*180 #- 90
        # if x <= -180:
        #     x = x + 360
        y = (pitch_y/3.14159)*180
        z = (yaw_z/3.14159)*180

        #if x < 0:
        #    x = x + 360

        theta = [x,y,z]

        return theta # in degree


if __name__=='__main__':
    rospy.init_node("vicon_decode")
    ViconDecode()
    rospy.spin()
