#!/usr/bin/env python

import rospy
import smach
import curses
import time
# import math
import collections
from curses import textpad
# from enum import Enum
# from uav_abstraction_layer.srv import TakeOff, Land
# from uav_abstraction_layer.msg import State
# from geometry_msgs.msg import TwistStamped, PoseStamped

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
global_params['vel_max'] = NumericalParam('vel_max', 0.0, 3.0, 1.0)
global_params['rate_max'] = NumericalParam('rate_max', 0.0, 3.0, 1.0)
global_params['xyz_step'] = NumericalParam('xyz_step', 0.0, 3.0, 1.0)
global_params['yaw_step'] = NumericalParam('xyz_step', 0.0, 3.0, 1.0)

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
        textpad.rectangle(self.screen, self.win_y-1, self.win_x-1, self.win_y+1, self.win_x+16)
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


class StateOutcome(object):
    def __init__(self):
        self.names = []
        self.labels = {}

    def add(self, name):
        self.names.append(name)
        self.labels[name] = name
        self.labels[name[0]] = name  # TODO: Check if repeated

class MainState(smach.State):
    def __init__(self, console):
        self.console = console
        self.out = StateOutcome()
        self.out.add('set_params')
        self.out.add('takeoff')
        self.out.add('land')
        self.out.add('vel')
        self.out.add('pose')
        self.out.add('quit')
        smach.State.__init__(self, outcomes=self.out.names)

    def execute(self, userdata):
        self.console.set_header('Main: {}'.format(self.out.names))
        self.console.set_footer('')
        self.console.reset_box()
        while not rospy.is_shutdown():
            current_cmd = self.console.get_box()
            if current_cmd in self.out.labels:
                return self.out.labels[current_cmd]
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

class SetParamsState(smach.State):
    def __init__(self, console):
        self.console = console
        smach.State.__init__(self, outcomes=['quit'])

    def execute(self, userdata):
        params = StateOutcome()
        for p in global_params:
            params.add(p)
        self.console.set_footer('')
        self.console.reset_box()
        while not rospy.is_shutdown():
            self.console.set_header('Set params: {} or {}'.format(params.names, 'quit'))
            current_cmd = self.console.get_box()
            if current_cmd in ('quit', 'q'):
                return 'quit'
            if current_cmd in params.labels:
                cmd = params.labels[current_cmd]
                set_param(global_params[cmd], self.console)
            else :
                self.console.set_footer('Unknown command [{}]'.format(current_cmd))

class TakeoffState(smach.State):
    def __init__(self, console):
        self.console = console
        smach.State.__init__(self, outcomes=['quit'])

    def execute(self, userdata):
        z_takeoff = global_params['z_takeoff'].val
        self.console.set_header('Taking off at {} [m]...'.format(z_takeoff))
        self.console.set_footer('')
        self.console.reset_box()
        time.sleep(2)
        # Call takeoff (blocking?)
        return 'quit'

class LandState(smach.State):
    def __init__(self, console):
        self.console = console
        smach.State.__init__(self, outcomes=['quit'])

    def execute(self, userdata):
        self.console.set_header('Landing...')
        self.console.set_footer('')
        self.console.reset_box()
        time.sleep(2)
        # Call land (blocking?)
        return 'quit'

class VelState(smach.State):
    def __init__(self, console):
        self.console = console
        smach.State.__init__(self, outcomes=['quit'])

    def execute(self, userdata):
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
                self.console.set_footer(str(joy))

class PoseState(smach.State):
    def __init__(self, console):
        self.console = console
        smach.State.__init__(self, outcomes=['quit'])

    def execute(self, userdata):
        self.console.set_header('Pose control: [q] to quit to main')
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
                self.console.set_footer(str(joy))

# def main(stdscr):
    # rospy.init_node('key_teleop')

    # console = ConsoleInterface(stdscr)
    # teleop = KeyTeleop(console)

    # rospy.Subscriber('ual/state', State, teleop.state_callback)
    # rospy.Subscriber('ual/pose', PoseStamped, teleop.pose_callback)
    # teleop.run()

def main(stdscr):
    rospy.init_node('key_teleop', log_level=rospy.DEBUG)
    
    console = ConsoleInterface(stdscr)
    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        smach.StateMachine.add('MAIN', MainState(console), 
                               transitions={'set_params':'PARAMS', 
                                            'takeoff':'TAKEOFF', 
                                            'land':'LAND', 
                                            'vel':'VEL', 
                                            'pose':'POSE', 
                                            'quit':'end'})

        smach.StateMachine.add('PARAMS', SetParamsState(console), 
                               transitions={'quit':'MAIN'})

        smach.StateMachine.add('TAKEOFF', TakeoffState(console), 
                               transitions={'quit':'MAIN'})

        smach.StateMachine.add('LAND', LandState(console), 
                               transitions={'quit':'MAIN'})

        smach.StateMachine.add('VEL', VelState(console), 
                               transitions={'quit':'MAIN'})

        smach.StateMachine.add('POSE', PoseState(console), 
                               transitions={'quit':'MAIN'})

    sm.execute()


if __name__ == '__main__':
    curses.wrapper(main)
