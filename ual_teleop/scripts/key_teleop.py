#! /usr/bin/env python

import curses
import math
import rospy
from curses import textpad
from enum import Enum
from uav_abstraction_layer.srv import TakeOff, Land
from uav_abstraction_layer.msg import State
from geometry_msgs.msg import TwistStamped, PoseStamped

TeleopState = Enum('TeleopState', 'MAIN VELOCITY_CONTROL POSE_CONTROL')

class ConsoleInterface(object):

    def __init__(self, stdscr):    
        self.screen = stdscr
        self.screen.nodelay(True)
        self.header_lines = 1
        self.win_x = 2
        self.win_y = self.header_lines+1
        self.head_y = 0
        self.foot_y = self.win_y+2
        self.win = curses.newwin(1, 16, self.win_y, self.win_x)
        textpad.rectangle(self.screen, self.win_y-1, self.win_x-1, self.win_y+1, self.win_x+16)
        self.box = textpad.Textbox(self.win, 'insert mode')

    def get_key(self):
        c = self.screen.getch()
        curses.flushinp()
        return c

    def get_box(self):
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

    def reset_box(self):
        self.win.clear()
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
        self.telop_state = TeleopState.MAIN
        # self.headless = False

    def state_callback(self, data):
        self.ual_state = data

    def pose_callback(self, data):
        self.uav_pose = data
        self.uav_yaw = 2.0 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)

    def run(self):
        takeoff_cmd      = ("takeoff", "t")
        land_cmd         = ("land", "l")
        vel_control_cmd  = ("velocity", "v")
        pose_control_cmd = ("pose", "p")
        quit_cmd         = ("quit", "q")
        main_msg = "Main: [{}] [{}] [{}] [{}] [{}]".format(takeoff_cmd[0], land_cmd[0], vel_control_cmd[0], pose_control_cmd[0], quit_cmd[0])
        velocity_msg = "Velocity control: [q] to return to main"
        pose_msg = "Pose control: [q] to return to main"
        joy = KeyJoystick()

        rate = rospy.Rate(10) # [Hz]
        while not rospy.is_shutdown():
            rate.sleep()
            if self.telop_state == TeleopState.MAIN:
                self.console.set_header(main_msg)
                command = self.console.get_box()
                if command in takeoff_cmd:
                    self.console.reset_box()
                    self.console.set_footer("Taking off...")
                    self.take_off(2.0, False)  # TODO(franreal): takeoff height? param mode?
                elif command in land_cmd:
                    self.console.reset_box()
                    self.console.set_footer("Landing...")
                    self.land(False)
                elif command in vel_control_cmd:
                    self.console.reset_box()
                    self.telop_state = TeleopState.VELOCITY_CONTROL
                elif command in pose_control_cmd:
                    self.console.reset_box()
                    self.telop_state = TeleopState.POSE_CONTROL
                elif command in quit_cmd:
                    break
                else:
                    self.console.set_footer("Unknown command [{}]".format(command))
            else:
                keycode = self.console.get_key()
                if keycode == ord('q'):
                    self.telop_state = TeleopState.MAIN
                    self.console.set_footer("")
                else:
                    joy.press(keycode)
                    self.console.set_footer(str(joy))

            if self.telop_state == TeleopState.VELOCITY_CONTROL:
                self.console.set_header(velocity_msg)
                vel_cmd = TwistStamped()
                vel_cmd.header.stamp = rospy.Time.now()
                vel_cmd.header.frame_id = 'map'
                vx = +joy.axis['move_forward'].value
                vy = -joy.axis['move_right'].value
                vz = +joy.axis['move_up'].value
                yaw_rate = -joy.axis['move_yaw'].value
                vel_cmd.twist.linear.x = (vx*math.cos(self.uav_yaw) - vy*math.sin(self.uav_yaw))
                vel_cmd.twist.linear.y = (vx*math.sin(self.uav_yaw) + vy*math.cos(self.uav_yaw))
                vel_cmd.twist.linear.z = vz
                vel_cmd.twist.angular.z = yaw_rate
                self.velocity_pub.publish(vel_cmd)

            if self.telop_state == TeleopState.POSE_CONTROL:
                self.console.set_header(pose_msg)
                pose_cmd = self.uav_pose
                pose_cmd.header.stamp = rospy.Time.now()
                # pose_cmd.header.frame_id = 'map'
                dx = +joy.axis['move_forward'].value
                dy = -joy.axis['move_right'].value
                dz = +joy.axis['move_up'].value
                delta_yaw = -joy.axis['move_yaw'].value
                pose_cmd.pose.position.x += (dx*math.cos(self.uav_yaw) - dy*math.sin(self.uav_yaw))
                pose_cmd.pose.position.y += (dx*math.sin(self.uav_yaw) + dy*math.cos(self.uav_yaw))
                pose_cmd.pose.position.z += dz
                yaw = self.uav_yaw + delta_yaw
                pose_cmd.pose.orientation.z = math.sin(0.5*yaw)
                pose_cmd.pose.orientation.w = math.cos(0.5*yaw)
                self.pose_pub.publish(pose_cmd)

def main(stdscr):
    rospy.init_node('key_teleop')

    console = ConsoleInterface(stdscr)
    teleop = KeyTeleop(console)

    rospy.Subscriber('ual/state', State, teleop.state_callback)
    rospy.Subscriber('ual/pose', PoseStamped, teleop.pose_callback)
    teleop.run()

if __name__ == '__main__':
    curses.wrapper(main)
