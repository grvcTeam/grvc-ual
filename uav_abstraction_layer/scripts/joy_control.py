#!/usr/bin/env python
import argparse
import subprocess
import sys
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from uav_abstraction_layer.srv import TakeOff, GoToWaypoint, Land
import time

class StateMachine:
    def __init__(self):
        rospy.init_node('joy_control')
        self.state = "landed"
        self.subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.lastJoy = Joy()
        self.publisher = rospy.Publisher('/ual/set_velocity', TwistStamped, queue_size=10)
        self.takeoff = rospy.ServiceProxy("/ual/take_off", TakeOff)
        self.land = rospy.ServiceProxy("/ual/land", Land)
        self.hasReceived = False
    
    def joy_callback(self, data):
        self.lastJoy = data
        self.hasReceived = True

    def isInit(self):
        return self.hasReceived

    def loop(self):
        rate = rospy.Rate(30) # 10hz
        while not rospy.is_shutdown():
            if self.state == "landed":
                self.landedFn()
            if self.state == "flying":
                self.flyingFn()
            else:
                pass
            rate.sleep()
    
    def landedFn(self):
        if self.lastJoy.buttons[0] == 1:
            self.takeoff(1, True)
            self.state = "flying"

    def flyingFn(self):
        if self.lastJoy.buttons[2] == 1:
            self.land(True)
            self.state = "landed"
        else:
            cmd = TwistStamped()
            cmd.twist.linear.x = -self.lastJoy.axes[3]*5
            cmd.twist.linear.y = self.lastJoy.axes[4]*5
            cmd.twist.linear.z = self.lastJoy.axes[1]*5
            cmd.twist.angular.z = self.lastJoy.axes[0]*5
            self.publisher.publish(cmd)


if __name__ == "__main__":
    machine = StateMachine()
    while(not machine.isInit()):
        time.sleep(0.1)
    machine.loop()
    