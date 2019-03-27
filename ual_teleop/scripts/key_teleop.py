#! /usr/bin/env python

import curses
import math
import rospy
from curses import textpad
from enum import Enum
from uav_abstraction_layer.srv import TakeOff, Land
from uav_abstraction_layer.msg import State
from geometry_msgs.msg import TwistStamped, PoseStamped

TeleopState = Enum('TeleopState', 'IDLE VELOCITY_CONTROL POSE_CONTROL')

class ConsoleInterface(object):

    def __init__(self, stdscr):    
        self.screen = stdscr
        self.screen.nodelay(True)
        # curses.curs_set(0)
        self.win_y = 2
        self.win_x = 2
        self.head_y = self.win_y-2
        self.foot_y = self.win_y+2
        self.win = curses.newwin(1, 16, self.win_y, self.win_x)
        textpad.rectangle(self.screen, self.win_y-1, self.win_x-1, self.win_y+1, self.win_x+16)
        self.box = textpad.Textbox(self.win, 'insert mode')

    def get_key(self):
        c = self.screen.getch()
        curses.flushinp()
        return c

    def get_string(self):
        cmd = self.box.edit().strip()
        return cmd

    # def refresh(self):
    #     self.screen.refresh()

    def set_header(self, msg):
        self.screen.move(self.head_y, 0)
        self.screen.clrtoeol()
        self.screen.addstr(self.head_y, self.win_x, msg)
        self.screen.refresh()

    def set_footer(self, msg):
        self.screen.move(self.foot_y, 0)
        self.screen.clrtoeol()
        self.screen.addstr(self.foot_y, self.win_x, msg)
        self.screen.refresh()

class KeyAxis(object):

    def __init__(self, inc_key, dec_key):
        self.value = 0.0
        self.step = 0.09
        self.dump = 0.43
        self.inc_key = inc_key
        self.dec_key = dec_key

    def press(self, key):
        if key == self.inc_key:
            self.increase()
        elif key == self.dec_key:
            self.decrease()
        else:
            self.update()

    def increase(self):
        self.value += self.step
        if self.value > 1.0:
            self.value = 1.0

    def decrease(self):
        self.value -= self.step
        if self.value < -1.0:
            self.value = -1.0

    def update(self):
        self.value *= self.dump
        if abs(self.value) < 1e-3:
            self.value = 0.0

class KeyJoystick(object):

    def __init__(self):
        # self.expected_keys = [curses.KEY_UP, curses.KEY_DOWN, curses.KEY_LEFT, curses.KEY_RIGHT, ord('a'), ord('s'), ord('d'), ord('w')]
        self.axis = {}
        self.axis["move_forward"] = KeyAxis(curses.KEY_UP, curses.KEY_DOWN)
        self.axis["move_right"] = KeyAxis(curses.KEY_RIGHT, curses.KEY_LEFT)
        self.axis["move_up"] = KeyAxis(ord('w'), ord('s'))
        self.axis["move_yaw"] = KeyAxis(ord('d'), ord('a'))

    def press(self, key):
        for action, axis in self.axis.items():
            axis.press(key)

    def update(self):
        for action, axis in self.axis.items():
            axis.update()

    def __str__(self):
        output = "[ "
        for key, axis in self.axis.items():
            output += "{:.3f}".format(axis.value) + " "
            # axis.update()
        output += "]"
        return output

class KeyTeleop(object):

    def __init__(self, console):
        self.console = console

        take_off_url = 'ual/take_off'
        land_url     = 'ual/land'
        set_vel_url  = 'ual/set_velocity'
        set_pose_url = 'ual/set_pose'
        # rospy.wait_for_service(take_off_url)
        # rospy.wait_for_service(land_url)
        self.take_off = rospy.ServiceProxy(take_off_url, TakeOff)
        self.land     = rospy.ServiceProxy(land_url,     Land)
        self.velocity_pub = rospy.Publisher(set_vel_url,  TwistStamped, queue_size=1)
        self.pose_pub     = rospy.Publisher(set_pose_url, PoseStamped,  queue_size=1)
        self.ual_state = State()
        self.uav_pose = PoseStamped()
        self.uav_yaw = 0.0
        self.telop_state = TeleopState.VELOCITY_CONTROL
        # self.headless = False
        # rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # [s]

    def state_callback(self, data):
        self.ual_state = data

    def pose_callback(self, data):
        self.uav_pose = data
        self.uav_yaw = 2.0 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)

    def run(self):
        takeoff_cmd      = "takeoff"
        land_cmd         = "land"
        vel_control_cmd  = "velocity"
        pose_control_cmd = "pose"
        quit_cmd         = "quit"
        idle_msg = "Idle: [{}] [{}] [{}] [{}] [{}]".format(takeoff_cmd, land_cmd, vel_control_cmd, pose_control_cmd, quit_cmd)
        velocity_msg = "Velocity control: [q] to return to idle"
        pose_msg = "Pose control: [q] to return to idle"
        # move_msg = "[w][s]/[a][d] to move in z/yaw, [arrows] to move forward-backwards-sideways"
        joy = KeyJoystick()

        rate = rospy.Rate(10) # [Hz]
        while not rospy.is_shutdown():
            rate.sleep()
            # self.console.refresh()
            if self.telop_state == TeleopState.IDLE:
                self.console.set_header(idle_msg)
                command = self.console.get_string()
                # print command
                if command == takeoff_cmd:
                    self.console.set_footer("Taking off...")
                    # self.take_off(2.0, False)  # TODO(franreal): takeoff height?
                elif command == land_cmd:
                    self.console.set_footer("Landing...")
                    # self.land(False)
                elif command == vel_control_cmd:
                    self.telop_state = TeleopState.VELOCITY_CONTROL
                elif command == pose_control_cmd:
                    self.telop_state = TeleopState.POSE_CONTROL
                elif command == quit_cmd:
                    break
                else:
                    self.console.set_footer("Unknown command [{}]".format(command))
            else:
                # self.console.set_footer(move_msg)
                keycode = self.console.get_key()
                if keycode == ord('q'):
                    self.telop_state = TeleopState.IDLE
                    self.console.set_footer("")
                # if keycode == -1:
                #     pass
                # elif keycode in joy.expected_keys:
                else:
                    joy.press(keycode)
                self.console.set_footer(str(joy))

            if self.telop_state == TeleopState.VELOCITY_CONTROL:
                self.console.set_header(velocity_msg)
                vel_cmd = TwistStamped()
                vel_cmd.header.stamp = rospy.Time.now()
                vel_cmd.header.frame_id = 'map'
                x = joy.axis['move_forward'].value
                y = joy.axis['move_right'].value
                vel_cmd.twist.linear.x = (x*math.cos(self.uav_yaw) - y*math.sin(self.uav_yaw))
                vel_cmd.twist.linear.y = (x*math.sin(self.uav_yaw) + y*math.cos(self.uav_yaw))
                vel_cmd.twist.linear.z = joy.axis['move_up'].value
                vel_cmd.twist.angular.z = joy.axis['move_yaw'].value
                self.velocity_pub.publish(vel_cmd)

            if self.telop_state == TeleopState.POSE_CONTROL:
                self.console.set_header(pose_msg)

    # def timer_callback(self, event):
    #     # print self.buffer

        # if self.headless == True and (self.joy_handle.get_action_button_state('toggle_headless') is ButtonState.JUST_PRESSED):
        #     rospy.loginfo("Exiting headless mode")
        #     self.headless = False
        # elif self.headless == False and (self.joy_handle.get_action_button_state('toggle_headless') is ButtonState.JUST_PRESSED):
        #     rospy.loginfo("Entering headless mode")
        #     self.headless = True

        # if self.joy_handle.get_action_button_state('speed_down') is ButtonState.JUST_PRESSED:
        #     self.gain_index = self.gain_index - 1 if self.gain_index > 0 else 0
        #     rospy.loginfo("Speed level: %d", self.gain_index)
        # if self.joy_handle.get_action_button_state('speed_up') is ButtonState.JUST_PRESSED:
        #     max_index = len(self.gains_table) - 1
        #     self.gain_index = self.gain_index + 1 if self.gain_index < max_index else max_index
        #     rospy.loginfo("Speed level: %d", self.gain_index)
            
        # if self.ual_state.state == State.FLYING_AUTO:
        #     vel_cmd = TwistStamped()
        #     vel_cmd.header.stamp = rospy.Time.now()
        #     # TODO: Use frame_id = 'uav_1' in not-headless mode?
        #     vel_cmd.header.frame_id = 'map'
        #     if self.headless:
        #         vel_cmd.twist.linear.x = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_forward')
        #         vel_cmd.twist.linear.y = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_right')
        #         vel_cmd.twist.linear.z = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_up')
        #         vel_cmd.twist.angular.z = self.joy_handle.get_action_axis('move_yaw')
        #     else:
        #         x = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_forward')
        #         y = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_right')
        #         vel_cmd.twist.linear.x = (x*math.cos(self.uav_yaw) - y*math.sin(self.uav_yaw))
        #         vel_cmd.twist.linear.y = (x*math.sin(self.uav_yaw) + y*math.cos(self.uav_yaw))
        #         vel_cmd.twist.linear.z = self.gains_table[self.gain_index] * self.joy_handle.get_action_axis('move_up')
        #         vel_cmd.twist.angular.z = self.joy_handle.get_action_axis('move_yaw')
        #     self.velocity_pub.publish(vel_cmd)
        # del self.buffer[:]

def main(stdscr):
    # Parse arguments
    # parser = argparse.ArgumentParser(description='Teleoperate ual with a joystick')
    # parser.add_argument('-joy_name', type=str, default=None,
                        # help='Joystick name, must have a equally named .yaml file in ual_teleop/config/joysticks folder')
    # args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('key_teleop')

    console = ConsoleInterface(stdscr)
    teleop = KeyTeleop(console)

    rospy.Subscriber('ual/state', State, teleop.state_callback)
    rospy.Subscriber('ual/pose', PoseStamped, teleop.pose_callback)
    teleop.run()

if __name__ == '__main__':
    curses.wrapper(main)
