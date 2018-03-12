#!/usr/bin/env python
import yaml
import rospy
import rospkg
from sensor_msgs.msg import Joy
from enum import Enum
from uav_abstraction_layer.srv import TakeOff, Land, SetVelocity 
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

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

    def state_callback(self, data):
        self.ual_state = data

    def joy_callback(self, data):
        self.joy_handle.update(data)
        # print self.joy_handle  # DEBUG
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
    rospy.init_node('joy_teleop', anonymous=True)
    joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/saitek_p3200.yaml'  # TODO(franreal): argument!
    robot_id = 'uav_1'  # TODO(franreal): argument!
    teleop = JoyTeleop(joy_file, robot_id)
    rospy.Subscriber(robot_id + '/ual/state', String, teleop.state_callback)
    rospy.Subscriber('joy', Joy, teleop.joy_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
