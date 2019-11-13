import rospy
import numpy as np
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece
from tf import TransformListener


class CrazyflieServices:
    def __init__(self, drone_id, high_level, use_tf=False):
        prefix = "/cf" + str(drone_id)
        self.__prefix = prefix
        self.__id = drone_id

        if use_tf:
            self.__tf = TransformListener()

        rospy.wait_for_service(prefix + "/set_group_mask")
        self.__setGroupMaskService = rospy.ServiceProxy(prefix + "/set_group_mask", SetGroupMask)
        rospy.wait_for_service(prefix + "/takeoff")
        self.__takeoffService = rospy.ServiceProxy(prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(prefix + "/land")
        self.__landService = rospy.ServiceProxy(prefix + "/land", Land)
        rospy.wait_for_service(prefix + "/stop")
        self.__stopService = rospy.ServiceProxy(prefix + "/stop", Stop)
        rospy.wait_for_service(prefix + "/go_to")
        self.__goToService = rospy.ServiceProxy(prefix + "/go_to", GoTo)
        rospy.wait_for_service(prefix + "/upload_trajectory")
        self.__uploadTrajectoryService = rospy.ServiceProxy(prefix + "/upload_trajectory",
                                                            UploadTrajectory)
        rospy.wait_for_service(prefix + "/start_trajectory")
        self.__startTrajectoryService = rospy.ServiceProxy(prefix + "/start_trajectory",
                                                           StartTrajectory)
        rospy.wait_for_service(prefix + "/update_params")
        self.__updateParamsService = rospy.ServiceProxy(prefix + "/update_params", UpdateParams)

        if high_level:
            self.set_param("commander/enHighLevel", 1)

    def set_group(self, group_mask):
        self.__setGroupMaskService(group_mask)

    def takeoff(self, target_height, duration, group_mask=0):
        self.__takeoffService(group_mask, target_height, rospy.Duration.from_sec(duration))

    def land(self, target_height, duration, group_mask=0):
        self.__landService(group_mask, target_height, rospy.Duration.from_sec(duration))

    def stop(self, group_mask=0):
        self.__stopService(group_mask)

    def goto(self, goal, yaw, duration, relative=False, group_mask=0):
        self.__goToService(group_mask, relative, goal, yaw, rospy.Duration.from_sec(duration))

    def upload_trajectory(self, trajectory_id, piece_offset, trajectory):
        pieces = []
        for poly in trajectory.polynomials:
            piece = TrajectoryPolynomialPiece()
            piece.duration = rospy.Duration.from_sec(poly.duration)
            piece.poly_x = poly.px.p
            piece.poly_y = poly.py.p
            piece.poly_z = poly.pz.p
            piece.poly_yaw = poly.pyaw.p
            pieces.append(piece)
        self.__uploadTrajectoryService(trajectory_id, piece_offset, pieces)

    def start_trajectory(self, trajectory_id, timescale=1.0, reverse=False, relative=True,
                         group_mask=0):
        self.__startTrajectoryService(group_mask, trajectory_id, timescale, reverse, relative)

    def position(self):
        if not self.__tf:
            raise RuntimeError("CF instance was created without using tf. set use_tf=True to get "
                               "position")
        self.__tf.waitForTransform("/world", self.__prefix, rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.__tf.lookupTransform("/world", self.__prefix, rospy.Time(0))
        return np.array(position)

    def get_param(self, name):
        return rospy.get_param(self.__prefix + "/" + name)

    def set_param(self, name, value):
        rospy.set_param(self.__prefix + "/" + name, value)
        self.__updateParamsService([name])

    def set_params(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.__prefix + "/" + name, value)
        self.__updateParamsService(params.keys())
