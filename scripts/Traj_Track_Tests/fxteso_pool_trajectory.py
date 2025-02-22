#!/usr/bin/env python

import os
import time
import rospy
import rosbag
import math
import scipy.io as sio
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, UInt8

trajectory = rospy.get_param("fxteso_pool_trajectory/trajectory", 'fxteso_infinity')
#Trajectory for second order controller

class Test:
    def __init__(self):
        self.testing = True

        self.flag = 0
        self.arduino = 0

        self.traj = Pose2D()
        self.trajder = Pose2D()
        self.trajder2 = Pose2D()

        rospy.Subscriber("/arduino_br/ardumotors/flag", UInt8, self.flag_callback)
        rospy.Subscriber("arduino", UInt8, self.arduino_callback)

        self.d_traj_pub = rospy.Publisher("/mission/trajectory", Pose2D, queue_size=10)
        self.d_trajder_pub = rospy.Publisher("/mission/trajectory_derivative", Pose2D, queue_size=10)
        self.d_trajder2_pub = rospy.Publisher("/mission/trajectory_second_derivative", Pose2D, queue_size=10)

    def flag_callback(self, _flag):
        self.flag = _flag

    def arduino_callback(self, _arduino):
        self.arduino = _arduino

    def desired(self, _xd, _yd, _xddot, _yddot, _xdddot, _ydddot, _start):
        self.traj.x = _xd
        self.traj.y = _yd
        self.traj.theta = _start
        self.trajder.x = _xddot
        self.trajder.y = _yddot
        self.trajder2.x = _xdddot
        self.trajder2.y = _ydddot
        self.d_traj_pub.publish(self.traj)
        self.d_trajder_pub.publish(self.trajder)
        self.d_trajder2_pub.publish(self.trajder2)

def main():
    rospy.init_node('fxteso_pool_trajectory', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    dir_name = os.path.dirname(__file__)
    xd = sio.loadmat(dir_name + '/mat/' + trajectory + '/fxteso_pool_xd.mat')
    xd = xd['data']
    yd = sio.loadmat(dir_name + '/mat/' + trajectory + '/fxteso_pool_yd.mat')
    yd = yd['data']
    xddot = sio.loadmat(dir_name + '/mat/' + trajectory + '/fxteso_pool_xddot.mat')
    xddot = xddot['data']
    yddot = sio.loadmat(dir_name + '/mat/' + trajectory + '/fxteso_pool_yddot.mat')
    yddot = yddot['data']
    xdddot = sio.loadmat(dir_name + '/mat/' + trajectory + '/fxteso_pool_xdddot.mat')
    xdddot = xdddot['data']
    ydddot = sio.loadmat(dir_name + '/mat/' + trajectory + '/fxteso_pool_ydddot.mat')
    ydddot = ydddot['data']
    time.sleep(10)
    rospy.logwarn("Start")
    if t.testing:
        start_time = rospy.Time.now().secs
        i = 0
        while (not rospy.is_shutdown()) and (i < xd.shape[1]):
            x = xd[0,i]
            y = yd[0,i]
            x_dot = xddot[0,i]
            y_dot = yddot[0,i]
            x_ddot = xdddot[0,i]
            y_ddot = ydddot[0,i]
            if i < 2:
                s = 1
            else:
                s = 2
            t.desired(x,y,x_dot,y_dot,x_ddot,y_ddot,s)
            i = i + 1
            rate.sleep()
        t.desired(0,0,0,0,0,0,0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
