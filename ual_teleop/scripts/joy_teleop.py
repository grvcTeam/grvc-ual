#!/usr/bin/env python
import yaml
import argparse
import rospy
import rospkg
import math
from joy_handle import JoyHandle, ButtonState
from sensor_msgs.msg import Joy
from uav_abstraction_layer.srv import TakeOff, Land, SetVelocity
from uav_abstraction_layer.msg import State
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

class JoyTeleop:

    def __init__(self, joy_name):
        action_file = rospkg.RosPack().get_path('ual_teleop') + '/config/joy_teleop.yaml'
        with open(action_file, 'r') as action_config:
            action_map = yaml.load(action_config)['joy_actions']

        self.joy_handle = JoyHandle(joy_name, action_map)
        take_off_url = 'ual/take_off'
        land_url =     'ual/land'
        velocity_url = 'ual/set_velocity'
        rospy.wait_for_service(take_off_url)
        rospy.wait_for_service(land_url)
        self.take_off = rospy.ServiceProxy(take_off_url, TakeOff)
        self.land     = rospy.ServiceProxy(land_url,     Land)
        self.velocity_pub = rospy.Publisher(velocity_url, TwistStamped, queue_size=1)
        self.ual_state = State()
        self.headless = False
        self.uav_yaw = 0.0
        self.gains_table = [0.5, 0.8, 1.0, 1.3, 1.8, 2.1, 2.5]
        self.gain_index = 2

    def state_callback(self, data):
        self.ual_state = data

    def pose_callback(self, data):
        self.uav_yaw = 2.0 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)

    def joy_callback(self, data):
        self.joy_handle.update(data)
        # print self.joy_handle  # DEBUG
        if self.joy_handle.get_action_button('secure'):
            if self.joy_handle.get_action_button_state('take_off') is ButtonState.JUST_PRESSED and self.ual_state.state == State.LANDED_ARMED:
                rospy.loginfo("Taking off")
                self.take_off(2.0, False)  # TODO(franreal): takeoff height?
            if self.joy_handle.get_action_button_state('land') is ButtonState.JUST_PRESSED and self.ual_state.state == State.FLYING_AUTO:
                rospy.loginfo("Landing")
                self.land(False)

        if self.headless == True and (self.joy_handle.get_action_button_state('toggle_headless') is ButtonState.JUST_PRESSED):
            rospy.loginfo("Exiting headless mode")
            self.headless = False
        elif self.headless == False and (self.joy_handle.get_action_button_state('toggle_headless') is ButtonState.JUST_PRESSED):
            rospy.loginfo("Entering headless mode")
            self.headless = True

        if self.joy_handle.get_action_button_state('speed_down') is ButtonState.JUST_PRESSED:
            self.gain_index = self.gain_index - 1 if self.gain_index > 0 else 0
            rospy.loginfo("Speed level: %d", self.gain_index)
        if self.joy_handle.get_action_button_state('speed_up') is ButtonState.JUST_PRESSED:
            max_index = len(self.gains_table) - 1
            self.gain_index = self.gain_index + 1 if self.gain_index < max_index else max_index
            rospy.loginfo("Speed level: %d", self.gain_index)
            
        if self.ual_state.state == State.FLYING_AUTO:
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = rospy.Time.now()
            # TODO: Use frame_id = 'uav_1' in not-headless mode?
            vel_cmd.header.frame_id = 'map'
            if self.headless:
                vel_cmd.twist.linear.x = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_forward')
                vel_cmd.twist.linear.y = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_right')
                vel_cmd.twist.linear.z = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_up')
                vel_cmd.twist.angular.z = self.joy_handle.get_action_axis('move_yaw')
            else:
                x = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_forward')
                y = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_right')
                vel_cmd.twist.linear.x = (x*math.cos(self.uav_yaw) - y*math.sin(self.uav_yaw))
                vel_cmd.twist.linear.y = (x*math.sin(self.uav_yaw) + y*math.cos(self.uav_yaw))
                vel_cmd.twist.linear.z = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_up')
                vel_cmd.twist.angular.z = self.joy_handle.get_action_axis('move_yaw')
            self.velocity_pub.publish(vel_cmd)

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Teleoperate ual with a joystick')
    parser.add_argument('-joy_name', type=str, default=None,
                        help='Joystick name, must have a equally named .yaml file in ual_teleop/config/joysticks folder')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('joy_teleop', anonymous=True)

    if args.joy_name is None:
        default_joy = 'saitek_p3200'
        rospy.loginfo("Using default joy [%s]", default_joy)
        args.joy_name = default_joy
    teleop = JoyTeleop(args.joy_name)

    rospy.Subscriber('ual/state', State, teleop.state_callback)
    rospy.Subscriber('ual/pose', PoseStamped, teleop.pose_callback)  # TODO: Use ground truth
    rospy.Subscriber('ual_teleop/joy', Joy, teleop.joy_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
