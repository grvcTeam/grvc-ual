#!/usr/bin/env python
import yaml
import rospy
from sensor_msgs.msg import Joy
from enum import Enum
from uav_abstraction_layer.srv import *
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

config = """
joy_layout:
  a:
    type: button
    index: 0
  b:
    type: button
    index: 1
  x:
    type: button
    index: 3
  y:
    type: button
    index: 4

  left_shoulder:
    type: button
    index: 6

  right_shoulder:
    type: button
    index: 7

  left_trigger:
    type: button
    index: 8

  right_trigger:
    type: button
    index: 9

  select:
    type: button
    index: 10

  start:
    type: button
    index: 11

  left_thumb:
    type: button
    index: 13

  right_thumb:
    type: button
    index: 14

  left_analog_x:
    type: axis
    index: 0
    reversed: true

  left_analog_y:
    type: axis
    index: 1
    reversed: false

  right_analog_x:
    type: axis
    index: 2
    reversed: true

  right_analog_y:
    type: axis
    index: 3
    reversed: false

  dpad_x:
    type: axis
    index: 6
    reversed: true

  dpad_y:
    type: axis
    index: 7
    reversed: false

"""

ButtonState = Enum('ButtonState', 'UNKNOWN JUST_PRESSED PRESSED JUST_RELEASED RELEASED')

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

    def __init__(self):
        self.ros_data = Joy()
        self.buttons_state = []
        self.layout = yaml.load(config)['joy_layout']

    def update(self, data):
        self.ros_data = data
        # Update buttons
        if len(self.ros_data.buttons) is not len(self.buttons_state):
            self.buttons_state = [ButtonState.UNKNOWN] * len(self.ros_data.buttons)
        for i, state in enumerate(self.buttons_state):
            self.buttons_state[i] = update_button_state(state, self.ros_data.buttons[i])

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

class JoyTeleop:

    def __init__(self):
        self.joy_handle = JoyHandle()
        robot_id = 'uav_1'  # TODO(franreal): other id?
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

    def state_callback(self, data):
        self.ual_state = data

    def joy_callback(self, data):
        self.joy_handle.update(data)
        if self.joy_handle.get_button('left_shoulder'):
            if self.joy_handle.get_button_state('x') is ButtonState.JUST_PRESSED:
                self.take_off(2.0, False)  # TODO(franreal): takeoff height?
            if self.joy_handle.get_button_state('b') is ButtonState.JUST_PRESSED:
                self.land(False)
        if self.ual_state.data == 'FLYING':
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = rospy.Time.now()
            vel_cmd.header.frame_id = 'map'
            vel_cmd.twist.linear.x = 0.5 * self.joy_handle.get_axis('right_analog_x')  # TODO(franreal): gain
            vel_cmd.twist.linear.y = 0.5 * self.joy_handle.get_axis('right_analog_y')  # TODO(franreal): gain
            vel_cmd.twist.linear.z = 0.2 * self.joy_handle.get_axis('left_analog_y')   # TODO(franreal): gain
            vel_cmd.twist.angular.z =     -self.joy_handle.get_axis('left_analog_x')   # TODO(franreal): gain
            self.velocity(vel_cmd)

def main():
    rospy.init_node("joy_teleop", anonymous=True)
    teleop = JoyTeleop()
    rospy.Subscriber("uav_1/ual/state", String, teleop.state_callback)
    rospy.Subscriber("joy", Joy, teleop.joy_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
