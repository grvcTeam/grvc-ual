#!/usr/bin/env python
import yaml
import rospkg
import rospy
from enum import Enum
from sensor_msgs.msg import Joy

expected_axes =     ['left_analog_x', 'left_analog_y',
                     'right_analog_x', 'right_analog_y',
                     'dpad_x', 'dpad_y']

expected_buttons =  ['a', 'b', 'x', 'y', 
                     'left_shoulder', 'right_shoulder',
                     'left_trigger', 'right_trigger',
                     'select', 'start',
                     'left_thumb', 'right_thumb']

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

    def __init__(self, joy_name, act_joy_map = {}):
        joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/joysticks/' + joy_name + '.yaml'
        with open(joy_file, 'r') as joy_config:
            joy_msg_map = yaml.load(joy_config)['joy_layout']

        found_axis_indices = []
        found_button_indices = []
        for key, info in joy_msg_map.items():
            index = info['index']
            if key in expected_axes:
                if 'reversed' not in info:
                    info['reversed'] = False
                if index in found_axis_indices:
                    raise ValueError('Axis index {} already used'.format(index))
                else:
                    found_axis_indices.append(index)
            elif key in expected_buttons:
                if index in found_button_indices:
                    raise ValueError('Button index {} already used'.format(index))
                else:
                    found_button_indices.append(index)
            else:
                raise ValueError('Unexpected key: {}'.format(key))

        self.ros_data = Joy()
        self.buttons_state = []
        self.joy_msg_map = joy_msg_map
        self.act_msg_axis_map = {}
        self.act_msg_button_map = {}
        self.unreachable_ids = []
        self.unreachable_actions = []

        found_ids = []
        for action, info in act_joy_map.items():
            id = info['id']
            if id not in joy_msg_map:
                self.unreachable_ids.append(id)
                self.unreachable_actions.append(action)
                rospy.logwarn("Id [%s] undefined for this joystick, unable to use [%s] action", id, action)
                continue
            if id in found_ids:
                rospy.logwarn("Id [%s] already in use, you may have unexpected behaviours", id)
            else:
                found_ids.append(id)
            if id in expected_axes:
                if 'reversed' not in info:
                    info['reversed'] = False
                self.act_msg_axis_map[action] = {'index': joy_msg_map[id]['index'], 'reversed': (bool(joy_msg_map[id]['reversed']) != bool(info['reversed']))}
            elif id in expected_buttons:
                self.act_msg_button_map[action] = {'index': joy_msg_map[id]['index']}
            else:
                raise ValueError('Unexpected joy axis/button id: {}'.format(id))
        # print 'joy_msg_map: {}'.format(self.joy_msg_map)
        # print 'act_joy_map: {}'.format(act_joy_map)
        # print 'act_msg_axis_map: {}'.format(self.act_msg_axis_map)
        # print 'act_msg_button_map: {}'.format(self.act_msg_button_map)
        return

    # TODO: Self-callback?
    def update(self, data):
        self.ros_data = data  # TODO: deepcopy?
        # Update buttons
        if len(self.ros_data.buttons) is not len(self.buttons_state):
            self.buttons_state = [ButtonState.UNKNOWN] * len(self.ros_data.buttons)
        for i, state in enumerate(self.buttons_state):
            self.buttons_state[i] = update_button_state(state, self.ros_data.buttons[i])

    def get_axis(self, id):
        if id in self.unreachable_ids:
            return 0.0
        if id not in expected_axes:
            raise ValueError("Unexpected axis id: {}".format(id))
        if self.joy_msg_map[id]['reversed']:
            return -self.ros_data.axes[self.joy_msg_map[id]['index']]
        else:
            return self.ros_data.axes[self.joy_msg_map[id]['index']]

    def get_button(self, id):
        if id in self.unreachable_ids:
            return 0
        if id not in expected_buttons:
            raise ValueError("Unexpected button id: {}".format(id))
        return self.ros_data.buttons[self.joy_msg_map[id]['index']]

    def get_button_state(self, id):
        if id in self.unreachable_ids:
            return ButtonState.RELEASED
        if id not in expected_buttons:
            raise ValueError("Unexpected button id: {}".format(id))
        return self.buttons_state[self.joy_msg_map[id]['index']]

    def get_action_axis(self, action):
        if action in self.unreachable_actions:
            return 0.0
        if action not in self.act_msg_axis_map:
            raise ValueError("Action {} of type axis is unknown".format(action))
        if self.act_msg_axis_map[action]['reversed']:
            return -self.ros_data.axes[self.act_msg_axis_map[action]['index']]
        else:
            return self.ros_data.axes[self.act_msg_axis_map[action]['index']]

    def get_action_button(self, action):
        if action in self.unreachable_actions:
            return 0
        if action not in self.act_msg_button_map:
            raise ValueError("Action {} of type button is unknown".format(action))
        return self.ros_data.buttons[self.act_msg_button_map[action]['index']]

    def get_action_button_state(self, action):
        if action in self.unreachable_actions:
            return ButtonState.RELEASED
        if action not in self.act_msg_button_map:
            raise ValueError("Action {} of type button is unknown".format(action))
        return self.buttons_state[self.act_msg_button_map[action]['index']]

    def __str__(self):
        output = '---\n'
        for key in self.joy_msg_map:
            if key in expected_axes:
                output = output + key + ': ' + str(self.get_axis(key)) + '\n'
            elif key in expected_buttons:
                output = output + key + ': ' + str(self.get_button(key)) + '\n'
        return output

if __name__ == "__main__":
    print "This is not a script!"
