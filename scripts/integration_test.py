#!/usr/bin/env python

import rospy
import math

from agent.Crazyflie import Crazyflie


def up_and_down(cf):
    rospy.init_node("integration_test")
    cf.goto(0, 0, 1.5)
    cf.sleep_until_inactive()
    cf.land()
    cf.sleep_until_inactive()


def circle_trajectory(n_pts, r):
    x = list()
    y = list()
    for i in range(n_pts):
        x.append(r * math.cos(2 * math.pi * i / n_pts))
        y.append(r * math.sin(2 * math.pi * i / n_pts))

    rospy.init_node("integration_test")
    cf.goto(r, 0, 1.5)
    cf.sleep_until_inactive()
    for i in range(len(x)):
        cf.goto(x[i], y[i], 1.5)
        cf.sleep_until_inactive()
    cf.goto(0, 0, 1.5)
    cf.sleep_until_inactive()
    cf.land()
    cf.sleep_until_inactive()


if __name__ == '__main__':
    cf = Crazyflie(1)
    up_and_down(cf)
    # circle_trajectory(50, 0.8)
