#!/usr/bin/env python

import rospy
from crazyflie_driver.srv import *
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

from CrazyflieServices import CrazyflieServices


class Crazyflie:
    """
    Represents a Crazyflie drone. It contains parameters such as id and position, and allows
    controlling the drone.
    """

    def __init__(self, drone_id, high_level=True):
        prefix = "/cf" + str(drone_id)
        self.__prefix = prefix
        self.__id = drone_id
        self.__services = CrazyflieServices(drone_id, high_level)

        self.__position_subs = rospy.Subscriber(prefix + '/local_position',
                                                crazyflie_driver.msg.GenericLogData,
                                                self.__pose_callback)
        self.__pose = Pose()

    def pause(self):
        pass

    def takeoff(self, target_height, duration, group_mask=0):
        self.__services.takeoff(target_height, duration, group_mask)

    def land(self, target_height, duration, group_mask=0):
        self.__services.land(target_height, duration, group_mask)

    def stop(self, group_mask=0):
        self.__services.stop(group_mask)

    def goto(self, goal, yaw, duration, relative=False, group_mask=0):
        self.__services.goto(goal, yaw, duration, relative, group_mask)

    def upload_trajectory(self, trajectory_id, piece_offset, trajectory):
        self.__services.upload_trajectory(trajectory_id, piece_offset, trajectory)

    def start_trajectory(self, trajectory_id, timescale=1.0, reverse=False, relative=True,
                         group_mask=0):
        self.__services.start_trajectory(trajectory_id, timescale, reverse, relative, group_mask)

    @property
    def pose(self):
        return self.__pose

    @property
    def id(self):
        return self.__id

    def __pose_callback(self, data):
        self.__pose.position.x = data.values[0]
        self.__pose.position.y = data.values[1]
        self.__pose.position.z = data.values[2]
        self.__pose.orientation = quaternion_from_euler(
            data.values[3], data.values[4], data.values[5])
