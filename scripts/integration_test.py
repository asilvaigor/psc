#!/usr/bin/env python

import rospy
import time
import math

from agent.Crazyflie import Crazyflie


def up_and_down(cf):
    rospy.init_node("integration_test")
    cf.goto(0, 0, 1.5)
    cf.sleep_until_inactive()
    cf.land()
    cf.sleep_until_inactive()


def circle_trajectory(freq, duration, r):
    x = list()
    y = list()
    t = list()
    for i in range(freq):
        t0 = 2 * math.pi * (1.0 / (freq - 1)) * duration * i
        x.append(r * math.cos(t0 / duration))
        y.append(r * math.sin(t0 / duration))
        t.append((1.0 / (freq - 1)) * duration * i)

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
    # up_and_down(cf)
    circle_trajectory(50, 10, 0.8)
    cf.stop()
