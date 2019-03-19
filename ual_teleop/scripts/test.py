#!/usr/bin/env python
import argparse
import rospy
import rospkg
import math
from joy_handle import JoyHandle, ButtonState
from sensor_msgs.msg import Joy
# from uav_abstraction_layer.srv import TakeOff, Land, SetVelocity 
# from geometry_msgs.msg import TwistStamped
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import String
import yaml
import pprint
import time
import copy

# TODO(franreal): Define expected axes/buttons in file?
expected_buttons =  ['a', 'b', 'x', 'y', 
                     'left_shoulder', 'right_shoulder',
                     'left_trigger', 'right_trigger',
                     'select', 'start',
                     'left_thumb', 'right_thumb']

expected_axes =     ['left_analog_x', 'left_analog_y',
                     'right_analog_x', 'right_analog_y',
                     'dpad_x', 'dpad_y']

current_joy = Joy()
joy_is_connected = False
def joy_callback(data):
    global joy_is_connected, current_joy
    joy_is_connected = True
    current_joy = data

def wait_button(label):
    global current_joy
    previous_joy = copy.deepcopy(current_joy)
    rospy.loginfo("Press [%s] button", label)
    while not rospy.is_shutdown():
        if current_joy.buttons != previous_joy.buttons:
            for i, button in enumerate(current_joy.buttons):
                if button and not previous_joy.buttons[i]:
                    return {'type': 'button', 'index': i}
            previous_joy = copy.deepcopy(current_joy)
        else:
            time.sleep(0.1)

def wait_axis(label):
    global current_joy
    previous_joy = copy.deepcopy(current_joy)
    if label.endswith("x"):
        rospy.loginfo("Move [%s] all the way left", label)
    elif label.endswith("y"):
        rospy.loginfo("Move [%s] all the way up", label)
    else:
        rospy.logerr("Unexpected axis [%s] direction", label)
    while not rospy.is_shutdown():
        if current_joy.axes != previous_joy.axes:
            for i, axis in enumerate(current_joy.axes):
                if math.fabs(axis) > 0.9:
                    return {'type': 'axis', 'index': i, 'reversed': (axis < 0)}
            previous_joy = copy.deepcopy(current_joy)
        else:
            time.sleep(0.1)


def main():

    # TODO: From file; reversed tag!!
    act_joy_dict = {'jump':{'id': 'a'}, 'run':{'id': 'b'}, 'move_x':{'id': 'dpad_x', 'reversed': True}, 'move_y':{'id': 'dpad_y', 'reversed': False}}
    jh = JoyHandle('saitek_p3200', act_joy_dict)
    print jh
    # act_msg_dict = {}
    # for key, value in act_joy_dict.items():
    #     act_msg_dict[key] = joy_msg_dict[value]
    # print act_msg_dict
    return



    # Parse arguments
    parser = argparse.ArgumentParser(description='Configure joystick generating yaml file')
    parser.add_argument('-joy_name', type=str, default=None,
                        help='Joystick name used to name the configuration yaml file')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('configure_joy', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)

    if args.joy_name is None:
        joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/new_joy.yaml'
    else:
        joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/' + joy_name + '.yaml'
    rospy.loginfo("Joy configuration map will be stored in [%s]", default_joy_file)

    # rospy.loginfo("Connect a joystick, rosrun joy joy_node and press any button to continue")
    # global joy_is_connected, current_joy
    # while not joy_is_connected and not rospy.is_shutdown():
    #     time.sleep(1)

    # rospy.loginfo("Detected a joystick with [%d] axes and [%d] buttons", len(current_joy.axes), len(current_joy.buttons))
    # layout = {}

    # used_indices = []
    # for button in expected_buttons:
    #     info = wait_button(button)
    #     if info['index'] in used_indices:
    #         rospy.logwarn("Already in use!")
    #     else:
    #         rospy.loginfo("[%s] index is [%d]", button, info['index'])
    #         used_indices.append(info['index'])
    #         layout[button] = info

    # used_indices = []
    # for axis in expected_axes:
    #     info = wait_axis(axis)
    #     if info['index'] in used_indices:
    #         rospy.logwarn("Already in use!")
    #     else:
    #         rospy.loginfo("[%s] index is [%d]", axis, info['index'])
    #         used_indices.append(info['index'])
    #         layout[axis] = info

    # print layout == file_layout
    print yaml.dump({'layout': file_layout}, default_flow_style=False)

    new_joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/new.yaml'
    with open(new_joy_file, 'w') as config:
        yaml.dump({'layout': file_layout}, config, default_flow_style=False)
        


    return

    previous_joy = copy.deepcopy(current_joy)

    rospy.loginfo("Press A button")
    waiting_button = True
    while waiting_button and not rospy.is_shutdown():
        if current_joy.buttons != previous_joy.buttons:
            for i, button in enumerate(current_joy.buttons):
                if button and not previous_joy.buttons[i]:
                    rospy.loginfo("A is index: %d", i)
                    waiting_button = False
                    break


    rospy.spin()
    return



    previous_joy = copy.deepcopy(current_joy)

    rospy.loginfo("Press A button")
    while previous_joy.buttons == current_joy.buttons:
        #  print current_joy
         pass

    button_found = False
    for i, button in enumerate(current_joy.buttons):
        if button:
            if button_found:
                rospy.logerr("Already assigned!")
            rospy.loginfo("A is index: %d", i)
            button_found = True


    # rospy.spin()
    return

    # rospy.init_node('test', anonymous=True)
    if args.joy_file is None:
        default_joy_file = rospkg.RosPack().get_path('ual_teleop') + '/config/saitek_p3200.yaml'
        rospy.loginfo("Using default joy map file [%s]", default_joy_file)
        args.joy_file = default_joy_file

    rospy.loginfo("Checking joy map file [%s]", args.joy_file)
    with open(args.joy_file, 'r') as config:
        layout = yaml.load(config)['joy_layout']

    tests_passed = True
    elements_to_delete = []
    axes_used_indices = []
    buttons_used_indices = []
    for element in layout:
        if 'type' not in layout[element]:
            rospy.logerr("Layout error: element [%s] does not have a type!", element)
            tests_passed = False
        if 'index' not in layout[element]:
            rospy.logerr("Layout error: element [%s] does not have an index!", element)
            tests_passed = False

        if element in expected_axes:
            # if 'type' not in layout[element] or layout[element]['type'] != 'axis'
            # if 'index' not in layout[element] or type(layout[element]['index']) is not int:
            if type(layout[element]['index']) is not int:
                rospy.logerr("Layout error: element %s should have an integer index, setting it to 0!", element)
                layout[element]['index'] = 0
                tests_passed = False
            if layout[element]['type'] != 'axis':
                rospy.logerr("Layout error: %s type should be axis, forcing it!", element)
                layout[element]['type'] = 'axis'
                tests_passed = False
            if 'reversed' not in layout[element]:
                rospy.logwarn("Layout warning: axis [%s] does not specify if reversed, setting it to false", element)
                layout[element]['reversed'] = False
            if layout[element]['index'] in axes_used_indices:
                rospy.logwarn("Layout warning: axis [%s] uses an already used index [%d]", element, layout[element]['index'])
                tests_passed = False
            else:
                axes_used_indices.append(layout[element]['index'])

            continue
        if element in expected_buttons:
            if layout[element]['type'] != 'button':
                rospy.logerr("Layout error: %s type should be button, forcing it!", element)
                layout[element]['type'] = 'button'
                tests_passed = False
            continue
        # Arrived here, element is not in expected axes nor buttons:
        elements_to_delete.append(element)
        tests_passed = False

    for element in elements_to_delete:
        rospy.logerr("Layout error: unexpected element [%s], deleting it!", element)
        del layout[element]

    pp = pprint.PrettyPrinter(indent=4)
    pp.pprint(layout)

    # rospy.Subscriber('ual/state', String, teleop.state_callback)
    # rospy.Subscriber('ual/pose', PoseStamped, teleop.pose_callback)  # TODO: Use ground truth
    # rospy.Subscriber('ual_teleop/joy', Joy, teleop.joy_callback)
    # rospy.spin()

if __name__ == '__main__':
    main()
