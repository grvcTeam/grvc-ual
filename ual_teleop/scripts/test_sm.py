#!/usr/bin/env python

import rospy
import time

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
    def __init__(self):
        State.__init__(self)
        # self.add_outcome('set_params')
        self.add_outcome('takeoff')
        self.add_outcome('land')
        # self.add_outcome('vel')
        # self.add_outcome('pose')
        self.add_outcome('quit')

    def execute(self):
        while not rospy.is_shutdown():
            current_cmd = raw_input('Main: {}'.format(self.outcomes))
            if current_cmd in self.out_labels:
                return self.out_labels[current_cmd]
            print('Unknown command [{}]'.format(current_cmd))

class TakeoffState(State):
    def __init__(self):
        State.__init__(self)
        self.add_outcome('quit')

    def execute(self):
        z_takeoff = 2.0
        print('Taking off at {} [m]...'.format(z_takeoff))
        time.sleep(2)
        # Call takeoff (blocking?)
        return 'quit'

class LandState(State):
    def __init__(self):
        State.__init__(self)
        self.add_outcome('quit')

    def execute(self):
        print('Executing state LAND')
        time.sleep(2)
        # Call land (blocking?)
        return 'quit'

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

def main():
    rospy.init_node('key_teleop')
    
    sm = StateMachine()
    sm.add('MAIN', MainState(), transitions={'takeoff':'TAKEOFF', 'land':'LAND', 'quit':'END'})
    sm.add('TAKEOFF', TakeoffState(), transitions={'quit':'MAIN'})
    sm.add('LAND', LandState(), transitions={'quit':'MAIN'})

    sm.set_current_state('MAIN')
    sm.set_final_state('END')
    sm.execute()


if __name__ == '__main__':
    main()
