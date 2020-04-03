import numpy as np
from math import pi

from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class StablePose:
    """
    3D Pose for a flying drone. It has x, y and z properties for the point and the yaw property for
    the orientation. The drones always have 0 roll and pitch.
    """

    def __init__(self, x=0, y=0, z=0, yaw=0):
        """
        Stores values. x, y, z in meters and yaw in radians [-pi, pi].
        """
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def position(self):
        """
        Returns Pose base point..
        :return: geometry_msgs.Point
        """
        return Point(self.x, self.y, self.z)

    def to_ros(self):
        """
        Converts to the ROS default Pose class.
        :return: rosmsg Pose type.
        """
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        aux = quaternion_from_euler(0, 0, self.yaw)
        pose.orientation.x = aux[0]
        pose.orientation.y = aux[1]
        pose.orientation.z = aux[2]
        pose.orientation.w = aux[3]
        return pose

    @staticmethod
    def from_ros(pose):
        """
        Converts from a ROS pose type to DronePose.
        :param pose: rosmsg pose type.
        :return: Converted DronePose object.
        """
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                      pose.orientation.w)
        yaw = euler_from_quaternion(quaternion)[2]
        return StablePose(pose.position.x, pose.position.y, pose.position.z, yaw)

    def __neg__(self):
        """
        Operator -StablePose.
        :return: Negative of the current pose.
        """
        return StablePose(-self.x, -self.y, -self.z, -self.yaw)

    def __add__(self, p):
        """
        Operator StablePose + StablePose.
        :param p: Pose to be added.
        :return: Sum of both poses.
        """
        return StablePose(self.x + p.x, self.y + p.y, self.z + p.z,
                          self.__normalize_ang(self.yaw + p.yaw))

    def __sub__(self, p):
        """
        Operator StablePose - StablePose.
        :param p: Pose to be subtracted.
        :return: Subtraction of poses.
        """
        return self + p.__neg__()

    def __mul__(self, k):
        """
        Operator StablePose * k, works as a vector.
        :param k: float value.
        :return: Multiplication of pose by scalar.
        """
        return StablePose(self.x * k, self.y * k, self.z * k,
                          self.__normalize_ang(self.yaw * k))

    def __div__(self, k):
        """
        Operator StablePose / k, works as a vector.
        :param k: float value.
        :return: Multiplication of pose by scalar.
        """
        return self.__mul__(1.0 / k)

    def __repr__(self):
        """
        Used for printing.
        :return: String representing StablePose.
        """
        return "[" + str(self.x) + ", " + str(self.y) + ", " + \
               str(self.z) + ", " + str(self.yaw) + "]"

    @staticmethod
    def __normalize_ang(ang):
        """
        Normalizes an angle to [-pi, pi].
        :param ang: float, Angle in radians.
        :return: float, Normalized angle.
        """
        while ang > pi:
            ang -= 2 * pi
        while ang < -pi:
            ang += 2 * pi
        return ang
