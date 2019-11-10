#!/usr/bin/env python

import rospy
from agent.Crazyflie import Crazyflie
import time
import math


def up_and_down(cf):
    cf.takeoff(target_height=1.5, duration=2.0)
    time.sleep(5.0)
    cf.land(target_height=0.0, duration=2.0)
    time.sleep(3.0)
    cf.stop()


def circle_trajectory(freq, duration, r):
    x = list()
    y = list()
    t = list()
    for i in range(freq):
        t0 = 2 * math.pi * (1.0 / (freq - 1)) * duration * i
        x.append(r * math.cos(t0 / duration))
        y.append(r * math.sin(t0 / duration))
        t.append((1.0 / (freq - 1)) * duration * i)

    cf.goto(goal=[r, 0, 1.5], yaw=0.0, duration=4, relative=False)
    time.sleep(4)
    for i in range(len(x)):
        cf.goto(goal=[x[i], y[i], 1.5], yaw=0.0, duration=1.0*duration/freq, relative=False)
        time.sleep(1.0*duration/freq)
    cf.goto(goal=[0, 0, 1.5], yaw=0.0, duration=4, relative=False)
    time.sleep(4)
    cf.land(target_height=0.0, duration=2.0)
    time.sleep(3.0)


if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = Crazyflie(1)
    # up_and_down(cf)
    circle_trajectory(50, 10, 0.8)
    cf.stop()
