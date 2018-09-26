#!/usr/bin/env python
import yaml
import rospy
import rospkg
import math
from sensor_msgs.msg import Joy
from enum import Enum
from uav_abstraction_layer.srv import TakeOff, Land, SetVelocity 
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import PyKDL
from tf_conversions import posemath

ButtonState = Enum('ButtonState', 'UNKNOWN JUST_PRESSED PRESSED JUST_RELEASED RELEASED')
gains = [0.5, 0.8, 1.0, 1.3, 1.8, 2.1, 2.5]

def update_button_state(prev_state, value):
    if prev_state is ButtonState.UNKNOWN:
        if value:
            return ButtonState.PRESSED
        else:
            return ButtonState.RELEASED
    elif prev_state is ButtonState.JUST_PRESSED:
        if value:
            return ButtonState.PRESSED
        else:
            return ButtonState.JUST_RELEASED
    elif prev_state is ButtonState.PRESSED:
        if value:
            return ButtonState.PRESSED
        else:
            return ButtonState.JUST_RELEASED
    elif prev_state is ButtonState.JUST_RELEASED:
        if value:
            return ButtonState.JUST_PRESSED
        else:
            return ButtonState.RELEASED
    elif prev_state is ButtonState.RELEASED:
        if value:
            return ButtonState.JUST_PRESSED
        else:
            return ButtonState.RELEASED
    else:
        raise IOError("Unexpected button state")

class JoyHandle:

    def __init__(self, config_file):
        self.ros_data = Joy()
        self.buttons_state = []
        with open(config_file, 'r') as config:
            self.layout = yaml.load(config)['joy_layout']

    def update(self, data):
        self.ros_data = data
        # Update buttons
        if len(self.ros_data.buttons) is not len(self.buttons_state):
            self.buttons_state = [ButtonState.UNKNOWN] * len(self.ros_data.buttons)
        for i, state in enumerate(self.buttons_state):
            self.buttons_state[i] = update_button_state(state, self.ros_data.buttons[i])

    # TODO(franreal): Study other interface functions like is_pressed(id)
    def get_axis(self, id):
        # TODO(franreal): Check id is an axis first?
        if self.layout[id]['reversed']:
            return -self.ros_data.axes[self.layout[id]['index']]
        else:
            return self.ros_data.axes[self.layout[id]['index']]

    def get_button(self, id):
        # TODO(franreal): Check id is a button first?
        return self.ros_data.buttons[self.layout[id]['index']]

    def get_button_state(self, id):
        # TODO(franreal): Check id is a button first?
        return self.buttons_state[self.layout[id]['index']]

    def __str__(self):
        output = '---\n'
        for button in ['a', 'b', 'x', 'y', 
                       'left_shoulder', 'right_shoulder',
                       'left_trigger', 'right_trigger',
                       'select', 'start',
                       'left_thumb', 'right_thumb']:
            output = output + button + ': ' + str(self.get_button_state(button)) + '\n'
        for axis in ['left_analog_x', 'left_analog_y',
                     'right_analog_x', 'right_analog_y',
                     'dpad_x', 'dpad_y']:
            output = output + axis + ': ' + str(self.get_axis(axis)) + '\n'
        return output

class JoyTeleop:

    def __init__(self, joy_file, robot_id):
        self.joy_handle = JoyHandle(joy_file)
        take_off_url = robot_id + '/ual/take_off'
        land_url =     robot_id + '/ual/land'
        velocity_url = robot_id + '/ual/set_velocity'
        rospy.wait_for_service(take_off_url)
        rospy.wait_for_service(land_url)
        rospy.wait_for_service(velocity_url)
        self.take_off = rospy.ServiceProxy(take_off_url, TakeOff)
        self.land     = rospy.ServiceProxy(land_url,     Land)
        self.velocity = rospy.ServiceProxy(velocity_url, SetVelocity)
        self.ual_state = String()
        self.headless = True
        self.ual_yaw = 0.0
        self.gain_value = 2

    def state_callback(self, data):
        self.ual_state = data

    def motion_callback(self, data):
        rot = PyKDL.Rotation.Quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        self.ual_yaw = rot.GetRPY()[2]
        #print "Yaw = " + str(self.ual_yaw) # DEBUG

    def joy_callback(self, data):
        self.joy_handle.update(data)
        # print self.joy_handle  # DEBUG
        if self.joy_handle.get_button('left_shoulder'):
            if self.joy_handle.get_button_state('x') is ButtonState.JUST_PRESSED and self.ual_state.data == 'LANDED':
                print "TAKING OFF"
                self.take_off(2.0, False)  # TODO(franreal): takeoff height?
            if self.joy_handle.get_button_state('b') is ButtonState.JUST_PRESSED and self.ual_state.data == 'FLYING':
                print "LANDING"
                self.land(False)
        
        if self.headless == True and (self.joy_handle.get_button_state('right_shoulder') is ButtonState.JUST_PRESSED):
            print "EXITING \"HEADLESS\" MODE"
            self.headless = False
        elif self.headless == False and (self.joy_handle.get_button_state('right_shoulder') is ButtonState.JUST_PRESSED):
            print "ENTERING \"HEADLESS\" MODE"
            self.headless = True

        if self.joy_handle.get_button_state('left_trigger') is ButtonState.JUST_PRESSED:
            self.gain_value = self.gain_value - 1 if self.gain_value > 0 else 0
            print "SPEED LEVEL: " + str(self.gain_value)
        if self.joy_handle.get_button_state('right_trigger') is ButtonState.JUST_PRESSED:
            self.gain_value = self.gain_value + 1 if self.gain_value < 6 else 6
            print "SPEED LEVEL: " + str(self.gain_value)
            
        if self.ual_state.data == 'FLYING':
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = rospy.Time.now()
            vel_cmd.header.frame_id = 'map'
            if self.headless == True:
                vel_cmd.twist.linear.x = 0.5 * gains[self.gain_value] * self.joy_handle.get_axis('right_analog_x') #NOTE: For ds4 controller X axis must be inverted
                vel_cmd.twist.linear.y = 0.5 * gains[self.gain_value] * self.joy_handle.get_axis('right_analog_y')
                vel_cmd.twist.linear.z = 0.2 * gains[self.gain_value] * self.joy_handle.get_axis('left_analog_y')
                vel_cmd.twist.angular.z =     -self.joy_handle.get_axis('left_analog_x')
            else:
                x = 0.5 * gains[self.gain_value] * self.joy_handle.get_axis('right_analog_x') #NOTE: For ds4 controller X axis must be inverted
                y = 0.5 * gains[self.gain_value] * self.joy_handle.get_axis('right_analog_y')
                vel_cmd.twist.linear.x = (x*math.cos(self.ual_yaw) - y*math.sin(self.ual_yaw))
                vel_cmd.twist.linear.y = (x*math.sin(self.ual_yaw) + y*math.cos(self.ual_yaw))
                vel_cmd.twist.linear.z = 0.2 * gains[self.gain_value] * self.joy_handle.get_axis('left_analog_y')
                vel_cmd.twist.angular.z =     -self.joy_handle.get_axis('left_analog_x')
            self.velocity(vel_cmd)

def main():
    rospy.init_node('joy_teleop', anonymous=True)
    joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/saitek_p3200.yaml'  # TODO(franreal): argument!
    robot_id = 'uav_1'  # TODO(franreal): argument!
    teleop = JoyTeleop(joy_file, robot_id)
    rospy.Subscriber(robot_id + '/ual/state', String, teleop.state_callback)
    rospy.Subscriber(robot_id + '/ual/pose', PoseStamped, teleop.motion_callback) #TODO: Use ground truth
    rospy.Subscriber('joy', Joy, teleop.joy_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
