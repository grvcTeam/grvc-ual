#!/usr/bin/env python
import argparse
import rospy
import rospkg
import math
from sensor_msgs.msg import Joy
import yaml
import time
import copy
from joy_handle import expected_axes, expected_buttons

current_joy = Joy()
joy_is_connected = False
def joy_callback(data):
    global joy_is_connected, current_joy
    joy_is_connected = True
    current_joy = data

def wait_button(label):
    global current_joy
    previous_joy = copy.deepcopy(current_joy)
    rospy.loginfo("Press [%s] button", label.upper())
    while not rospy.is_shutdown():
        if current_joy.buttons != previous_joy.buttons:
            for i, button in enumerate(current_joy.buttons):
                if button and not previous_joy.buttons[i]:
                    return {'index': i}
            previous_joy = copy.deepcopy(current_joy)
        else:
            time.sleep(0.1)

def wait_axis(label):
    global current_joy
    previous_joy = copy.deepcopy(current_joy)
    if label.endswith("x"):
        rospy.loginfo("Move [%s] all the way to the RIGHT and release it fast", label[:-2].upper())
    elif label.endswith("y"):
        rospy.loginfo("Move [%s] all the way UP and release it fast", label[:-2].upper())
    else:
        rospy.logerr("Unexpected axis [%s] direction", label)
    while not rospy.is_shutdown():
        if current_joy.axes != previous_joy.axes:
            for i, axis in enumerate(current_joy.axes):
                if axis != previous_joy.axes[i] and math.fabs(axis) > 0.99:
                    return {'index': i, 'reversed': (axis < 0)}
            previous_joy = copy.deepcopy(current_joy)
        else:
            time.sleep(0.1)

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Configure joystick generating yaml file')
    parser.add_argument('-joy_name', type=str, default=None,
                        help='Joystick name used to name the configuration yaml file')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('configure_joy', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)

    rospy.loginfo("Connect joystick, rosrun joy joy_node and press any button to continue")
    global joy_is_connected, current_joy
    while not joy_is_connected and not rospy.is_shutdown():
        time.sleep(1)

    rospy.loginfo("Detected joystick with [%d] axes and [%d] buttons", len(current_joy.axes), len(current_joy.buttons))
    layout = {}

    used_indices = []
    for button in expected_buttons:
        info = wait_button(button)
        if info['index'] in used_indices:
            rospy.logwarn("Already in use!")
        else:
            # rospy.loginfo("[%s] index is [%d]", button, info['index'])
            used_indices.append(info['index'])
            layout[button] = info

    used_indices = []
    for axis in expected_axes:
        info = wait_axis(axis)
        if info['index'] in used_indices:
            rospy.logwarn("Already in use!")
        else:
            # rospy.loginfo("[%s] index is [%d]", axis, info['index'])
            used_indices.append(info['index'])
            layout[axis] = info

    if args.joy_name is None:
        args.joy_name = 'new_joy'
    joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/joysticks/' + args.joy_name + '.yaml'

    with open(joy_file, 'w') as config:
        yaml.dump({'joy_layout': layout}, config, default_flow_style=False)
    rospy.loginfo("Joy configuration map stored in [%s]", joy_file)

    return

if __name__ == '__main__':
    main()
