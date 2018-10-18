#!/usr/bin/env python
import yaml
import rospy
import rospkg
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from enum import Enum

# TODO: Duplicated from here...
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
# TODO: ... to here in ual_teleop/scripts/joy_teleop.py!

class RCSimulation:
    def __init__(self, robot_id='uav_1'):
        rc_topic = robot_id + '/mavros/rc/override'
        self.pub = rospy.Publisher(rc_topic, OverrideRCIn, queue_size=1)

        centered_pwm = 1500
        self.rc_in = OverrideRCIn()
        self.rc_in.channels[0] = 900           # throttle (down!)
        self.rc_in.channels[1] = centered_pwm  # yaw
        self.rc_in.channels[2] = centered_pwm  # pitch
        self.rc_in.channels[3] = centered_pwm  # roll
        self.rc_in.channels[4] = 900           # fltmode  (down!)
        self.rc_in.channels[5] = centered_pwm  # NOT MAPPED
        self.rc_in.channels[6] = centered_pwm  # NOT MAPPED
        self.rc_in.channels[7] = centered_pwm  # NOT MAPPED
        rospy.Timer(rospy.Duration(0.1), self.update_callback)  # 10Hz

    def set_channel(self, channel_id, channel_pwm):
        if channel_id < 1 or channel_id > 8:  # channel id in [1, 8] 
            rospy.logerr("Invalid channel_id[%d]", channel_id)
            return
        if channel_pwm < 900 or channel_pwm > 2100:  # pwm in [900, 2100]
            rospy.logerr("PWM value [%d] out of bounds", channel_pwm)
            return
        self.rc_in.channels[channel_id - 1] = channel_pwm  # 0-indexing!

    def get_channel(self, channel_id):
        if channel_id < 1 or channel_id > 8:  # channel id in [1, 8] 
            rospy.logerr("Invalid channel_id[%d]", channel_id)
            return
        return self.rc_in.channels[channel_id - 1]

    def update_callback(self, event):
        self.pub.publish(self.rc_in)

class RCJoy:
    def __init__(self, joy_file, robot_id='uav_1'):
        self.joy_handle = JoyHandle(joy_file)
        self.rc_simulation = RCSimulation(robot_id)
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def joy_callback(self, data):
        self.joy_handle.update(data)
        # print self.joy_handle  # DEBUG
        self.rc_simulation.set_channel(1, 1500 + 600*self.joy_handle.get_axis('left_analog_y'))   # throttle
        self.rc_simulation.set_channel(2, 1500 + 600*self.joy_handle.get_axis('left_analog_x'))   # yaw
        self.rc_simulation.set_channel(3, 1500 + 600*self.joy_handle.get_axis('right_analog_y'))  # pitch
        self.rc_simulation.set_channel(4, 1500 + 600*self.joy_handle.get_axis('right_analog_x'))  # roll
        if self.joy_handle.get_button('left_shoulder'):
            fltmode_pwm = self.rc_simulation.get_channel(5)
            if (self.joy_handle.get_button('x')):
                fltmode_pwm = 2100
            if (self.joy_handle.get_button('y')):
                fltmode_pwm = 1500
            if (self.joy_handle.get_button('b')):  # This button has maximum priority (stabilized)
                fltmode_pwm = 900
            self.rc_simulation.set_channel(5, fltmode_pwm)                                        # fltmode

def main():
    rospy.init_node('rc_simulation', anonymous=True)
    joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/saitek_p3200.yaml'  # TODO(franreal): argument!
    robot_id = 'uav_1'  # TODO(franreal): argument!
    rc_joy = RCJoy(joy_file, robot_id)
    rospy.spin()

if __name__ == '__main__':
    main()
