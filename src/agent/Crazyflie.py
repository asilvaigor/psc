#!/usr/bin/env python

import rospy
import numpy as np
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece
from tf import TransformListener
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])


class Crazyflie:
    def __init__(self, id, use_tf=False):
        """
        Creates a Crazyflie high-level wrapper

        Args:
            prefix (str): ROS namespace of the drone. Ex = "/cf1"
            cf_id (int): drone id. Ex : 1
        """
        prefix = "cf" + str(id)
        self.prefix = prefix
        if use_tf:
            self.tf = TransformListener()
        self.cf_id = id

        self.__position_subs = rospy.Subscriber(prefix + '/local_position', crazyflie_driver.msg.GenericLogData,
                                                self.__pose_callback)
        self.__pose = Pose

        rospy.wait_for_service(prefix + "/set_group_mask")
        self.setGroupMaskService = rospy.ServiceProxy(prefix + "/set_group_mask", SetGroupMask)
        rospy.wait_for_service(prefix + "/takeoff")
        self.takeoffService = rospy.ServiceProxy(prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(prefix + "/land")
        self.landService = rospy.ServiceProxy(prefix + "/land", Land)
        rospy.wait_for_service(prefix + "/stop")
        self.stopService = rospy.ServiceProxy(prefix + "/stop", Stop)
        rospy.wait_for_service(prefix + "/go_to")
        self.goToService = rospy.ServiceProxy(prefix + "/go_to", GoTo)
        rospy.wait_for_service(prefix + "/upload_trajectory")
        self.uploadTrajectoryService = rospy.ServiceProxy(prefix + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service(prefix + "/start_trajectory")
        self.startTrajectoryService = rospy.ServiceProxy(prefix + "/start_trajectory", StartTrajectory)
        rospy.wait_for_service(prefix + "/update_params")
        self.updateParamsService = rospy.ServiceProxy(prefix + "/update_params", UpdateParams)

    def setGroup(self, groupMask):
        self.setGroupMaskService(groupMask)

    def takeoff(self, targetHeight, duration, groupMask=0):
        self.takeoffService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, groupMask=0):
        self.landService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def stop(self, groupMask=0):
        self.stopService(groupMask)

    def goTo(self, goal, yaw, duration, relative=False, groupMask=0):
        gp = arrayToGeometryPoint(goal)
        self.goToService(groupMask, relative, gp, yaw, rospy.Duration.from_sec(duration))

    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        pieces = []
        for poly in trajectory.polynomials:
            piece = TrajectoryPolynomialPiece()
            piece.duration = rospy.Duration.from_sec(poly.duration)
            piece.poly_x = poly.px.p
            piece.poly_y = poly.py.p
            piece.poly_z = poly.pz.p
            piece.poly_yaw = poly.pyaw.p
            pieces.append(piece)
        self.uploadTrajectoryService(trajectoryId, pieceOffset, pieces)

    def startTrajectory(self, trajectoryId, timescale=1.0, reverse=False, relative=True, groupMask=0):
        self.startTrajectoryService(groupMask, trajectoryId, timescale, reverse, relative)

    def position(self):
        if not self.tf:
            raise RuntimeError("CF instance was created without using tf. set use_tf=True to get position")
        self.tf.waitForTransform("/world", "/cf" + str(self.cf_id), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/cf" + str(self.cf_id), rospy.Time(0))
        return np.array(position)

    def pose(self):
        return self.__pose

    def __pose_callback(self, data):
        self.__pose.position.x = data.values[0]
        self.__pose.position.y = data.values[1]
        self.__pose.position.z = data.values[2]
        self.__pose.orientation = quaternion_from_euler(
            data.values[3], data.values[4], data.values[5])

    def getParam(self, name):
        return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService([name])

    def setParams(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService(params.keys())

    def enableHighLevel(self):
        self.setParam("commander/enHighLevel", 1)
