#!/usr/bin/env python

import rospy
import curses
import time
import math
import collections
from curses import textpad
from uav_abstraction_layer.srv import TakeOff, Land
from geometry_msgs.msg import TwistStamped, PoseStamped

class NumericalParam(object):
    def __init__(self, name, min_value, max_value, value):
        if min_value > max_value:
            raise ValueError("Min value {} grater than max {}".format(min_value, max_value))
        if (value < min_value) or (value > max_value):
            raise ValueError("Value {} not in range [{}, {}]".format(value, min_value, max_value))
        self.name = name
        self.min = min_value
        self.max = max_value
        self.val = value
        self.msg = "Initial value: {}".format(self.val)

    def set(self, value):
        # TODO: check value is float!
        if value < self.min:
            self.val = self.min
            self.msg = "Value [{}] too low, setting to [{}]".format(value, self.val)
        elif value > self.max:
            self.val = self.max
            self.msg = "Value [{}] too high, setting to [{}]".format(value, self.val)
        else:
            self.val = value
            self.msg = "Value correctly set: {}".format(self.val)

global_params = collections.OrderedDict()
global_params['z_takeoff'] = NumericalParam('z_takeoff', 0.0, 5.0, 2.0)
global_params['vel_mul'] = NumericalParam('vel_mul', 0.0, 3.0, 1.0)
global_params['rate_mul'] = NumericalParam('rate_mul', 0.0, 3.0, 1.0)
global_params['xyz_mul'] = NumericalParam('xyz_mul', 0.0, 0.3, 0.1)
global_params['yaw_mul'] = NumericalParam('xyz_mul', 0.0, 0.3, 0.1)

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
        self.screen.move(self.win_y-1, 0)
        self.screen.clrtoeol()
        textpad.rectangle(self.screen, self.win_y-1, self.win_x-1, self.win_y+1, self.win_x+16)
        self.win.clear()
        self.screen.move(self.win_y, self.win_x)
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
        for axis in self.axis.values():
            axis.press(key)

    def update(self):
        for axis in self.axis.values():
            axis.update()

    def __str__(self):
        output = "[ "
        for axis in self.axis.values():
            output += "{:.3f}".format(axis.value) + " "
        output += "]"
        return output

class State(object):
    def __init__(self):
        self.outcomes = []
        self.out_labels = {}

    def add_outcome(self, name):
        self.outcomes.append(name)
        self.out_labels[name] = name
        self.out_labels[name[0]] = name  # TODO: Check if repeated

    def execute(self):
        raise NotImplementedError()

class MainState(State):
    def __init__(self, console):
        State.__init__(self)
        self.console = console
        self.add_outcome('set_params')
        self.add_outcome('takeoff')
        self.add_outcome('land')
        self.add_outcome('vel')
        self.add_outcome('pose')
        self.add_outcome('quit')

    def execute(self):
        main_msg = '[' + '] ['.join(self.outcomes) + ']'
        self.console.set_header('Main: {}'.format(main_msg))
        self.console.set_footer('')
        self.console.reset_box()
        while not rospy.is_shutdown():
            current_cmd = self.console.get_box()
            if current_cmd in self.out_labels:
                return self.out_labels[current_cmd]
            self.console.set_footer('Unknown command [{}]'.format(current_cmd))

def set_param(param, console):
    console.set_header('[{}] current value: {}. Set a new value:'.format(param.name, param.val))
    console.set_footer('')
    console.reset_box()
    value = console.get_box()
    try:
        new_value = float(value)
        console.reset_box()
        param.set(new_value)
        console.set_footer("[{}]: {}".format(param.name, param.msg))
    except ValueError:
        console.set_footer("Invalid value [{}]".format(value))

class SetParamsState(State):
    def __init__(self, console):
        State.__init__(self)
        self.add_outcome('quit')
        self.console = console

    def execute(self):
        params = State()
        for p in global_params:
            params.add_outcome(p)
        self.console.set_footer('')
        self.console.reset_box()
        while not rospy.is_shutdown():
            param_msg = '[' + '] ['.join(params.outcomes) + ']'
            self.console.set_header('Set param: {} [{}]'.format(param_msg, 'quit'))
            current_cmd = self.console.get_box()
            if current_cmd in ('quit', 'q'):
                return 'quit'
            if current_cmd in params.out_labels:
                cmd = params.out_labels[current_cmd]
                set_param(global_params[cmd], self.console)
            else :
                self.console.set_footer('Unknown command [{}]'.format(current_cmd))

class TakeoffState(State):
    def __init__(self, console):
        State.__init__(self)
        self.add_outcome('quit')
        self.console = console
        take_off_url = 'ual/take_off'
        # rospy.wait_for_service(take_off_url)
        self.take_off = rospy.ServiceProxy(take_off_url, TakeOff)

    def execute(self):
        z_takeoff = global_params['z_takeoff'].val
        self.console.set_header('Taking off at {} [m]...'.format(z_takeoff))
        self.console.set_footer('')
        self.console.reset_box()
        # time.sleep(2)
        try:
            self.take_off(z_takeoff, True)  # TODO: Non-blocking?
        except rospy.ServiceException as exc:
            print("Service takeOff not process request: " + str(exc))
        return 'quit'

class LandState(State):
    def __init__(self, console):
        State.__init__(self)
        self.add_outcome('quit')
        self.console = console
        land_url = 'ual/land'
        # rospy.wait_for_service(land_url)
        self.land = rospy.ServiceProxy(land_url, Land)

    def execute(self):
        self.console.set_header('Landing...')
        self.console.set_footer('')
        self.console.reset_box()
        # time.sleep(2)
        try:
            self.land(True)  # TODO: Non-blocking?
        except rospy.ServiceException as exc:
            print("Service takeOff not process request: " + str(exc))
        return 'quit'

class VelState(State):
    def __init__(self, console):
        State.__init__(self)
        self.add_outcome('quit')
        self.console = console
        self.uav_pose = PoseStamped()
        self.uav_yaw = 0.0
        self.pose_sub = rospy.Subscriber('ual/pose', PoseStamped, self.pose_callback)
        self.velocity_pub = rospy.Publisher('ual/set_velocity', TwistStamped, queue_size=1)

    def pose_callback(self, data):
        self.uav_pose = data
        self.uav_yaw = 2.0 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)

    def execute(self):
        self.console.set_header('Velocity control: [q] to quit to main')
        self.console.set_footer('')
        self.console.reset_box()
        joy = KeyJoystick()
        rate = rospy.Rate(10)  # [Hz]
        while not rospy.is_shutdown():
            rate.sleep()
            keycode = self.console.get_key()
            if keycode == ord('q'):
                return 'quit'
            else:
                joy.press(keycode)
                self.console.set_footer('axes: ' + str(joy))
                vel_mul = global_params['vel_mul'].val
                rate_mul = global_params['rate_mul'].val
                vel_cmd = TwistStamped()
                vel_cmd.header.stamp = rospy.Time.now()
                vel_cmd.header.frame_id = 'map'
                vx = +vel_mul * joy.axis['move_forward'].value
                vy = -vel_mul * joy.axis['move_right'].value
                vz = +vel_mul * joy.axis['move_up'].value
                yaw_rate = -rate_mul * joy.axis['move_yaw'].value
                vel_cmd.twist.linear.x = (vx*math.cos(self.uav_yaw) - vy*math.sin(self.uav_yaw))
                vel_cmd.twist.linear.y = (vx*math.sin(self.uav_yaw) + vy*math.cos(self.uav_yaw))
                vel_cmd.twist.linear.z = vz
                vel_cmd.twist.angular.z = yaw_rate
                self.velocity_pub.publish(vel_cmd)

class PoseState(State):
    def __init__(self, console):
        State.__init__(self)
        self.add_outcome('quit')
        self.console = console
        self.uav_pose = PoseStamped()
        self.uav_yaw = 0.0
        self.pose_sub = rospy.Subscriber('ual/pose', PoseStamped, self.pose_callback)
        self.pose_pub = rospy.Publisher('ual/set_pose', PoseStamped, queue_size=1)

    def pose_callback(self, data):
        self.uav_pose = data
        self.uav_yaw = 2.0 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)

    def execute(self):
        self.console.set_header('Pose control: [q] to quit to main')
        self.console.set_footer('')
        self.console.reset_box()
        joy = KeyJoystick()
        rate = rospy.Rate(10)  # [Hz]
        pose_cmd = self.uav_pose
        while not rospy.is_shutdown():
            rate.sleep()
            keycode = self.console.get_key()
            if keycode == ord('q'):
                return 'quit'
            else:
                joy.press(keycode)
                self.console.set_footer('axes: ' + str(joy))
                xyz_mul = global_params['xyz_mul'].val
                yaw_mul = global_params['yaw_mul'].val
                pose_cmd.header.stamp = rospy.Time.now()
                # pose_cmd.header.frame_id = 'map'
                dx = +xyz_mul * joy.axis['move_forward'].value
                dy = -xyz_mul * joy.axis['move_right'].value
                dz = +xyz_mul * joy.axis['move_up'].value
                delta_yaw = -yaw_mul * joy.axis['move_yaw'].value
                pose_cmd.pose.position.x += (dx*math.cos(self.uav_yaw) - dy*math.sin(self.uav_yaw))
                pose_cmd.pose.position.y += (dx*math.sin(self.uav_yaw) + dy*math.cos(self.uav_yaw))
                pose_cmd.pose.position.z += dz
                yaw = self.uav_yaw + delta_yaw
                pose_cmd.pose.orientation.z = math.sin(0.5*yaw)
                pose_cmd.pose.orientation.w = math.cos(0.5*yaw)
                self.pose_pub.publish(pose_cmd)

class StateMachine(object):
    def __init__(self):
        self.states = {}
        self.transitions = {}
        self.current_state = ''
        self.final_state = ''

    def set_current_state(self, state_label):
        self.current_state = state_label

    def set_final_state(self, state_label):
        self.final_state = state_label

    def add(self, state_label, state_object, transitions):
        self.states[state_label] = state_object
        self.transitions[state_label] = transitions

    def execute(self):
        while not rospy.is_shutdown():
            out = self.states[self.current_state].execute()
            self.current_state = self.transitions[self.current_state][out]
            if self.final_state == self.current_state:
                break


def main(stdscr):
    rospy.init_node('key_teleop')
    
    console = ConsoleInterface(stdscr)
    sm = StateMachine()
    sm.add('MAIN', MainState(console),
            transitions={'set_params':'PARAMS', 
                         'takeoff':'TAKEOFF', 
                         'land':'LAND', 
                         'vel':'VEL', 
                         'pose':'POSE', 
                         'quit':'END'})

    sm.add('PARAMS', SetParamsState(console), 
            transitions={'quit':'MAIN'})

    sm.add('TAKEOFF', TakeoffState(console), 
            transitions={'quit':'MAIN'})

    sm.add('LAND', LandState(console), 
            transitions={'quit':'MAIN'})

    sm.add('VEL', VelState(console), 
            transitions={'quit':'MAIN'})

    sm.add('POSE', PoseState(console), 
            transitions={'quit':'MAIN'})

    sm.set_current_state('MAIN')
    sm.set_final_state('END')
    sm.execute()


if __name__ == '__main__':
    curses.wrapper(main)
