#!/usr/bin/env python
import yaml
from enum import Enum
from sensor_msgs.msg import Joy

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

if __name__ == "__main__":
    print "This is not a script!"
