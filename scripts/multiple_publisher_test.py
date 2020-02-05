#!/usr/bin/env python

import rospy
import time
import math

from representations.Constants import RATE
from agent.Crazyflie import Crazyflie
from tools.visualization.VisualizationPublisher import VisualizationPublisher


# Testing the class VisualizationPublisher.py

def up_and_down(drones):
    rospy.init_node("multiple_publisher_test")
    r = rospy.Rate(RATE)
    publisher = VisualizationPublisher(drones)
    for i in range(4):
        drones[i].goto(0, 0, 1.5)
    while not (drones[0].is_inactive() and drones[1].is_inactive() and drones[2].is_inactive() and drones[3].is_inactive()):
        publisher.visualize()
        r.sleep()
    for i in range(4):
        drones[i].land()
    while not (drones[0].is_inactive() and drones[1].is_inactive() and drones[2].is_inactive() and drones[3].is_inactive()):
        publisher.visualize()
        r.sleep()


if __name__ == '__main__':
    cf1 = Crazyflie(1)
    cf2 = Crazyflie(2)
    cf3 = Crazyflie(3)
    cf4 = Crazyflie(4)
    cfs = {1: cf1, 2: cf2, 3: cf3, 4: cf4}
    up_and_down(cfs)
    for i in range(4):
        cfs[i].stop()
