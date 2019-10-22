#!/usr/bin/env python
import yaml
import argparse
import rospy
import rospkg
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from uav_abstraction_layer.msg import State as UalState
from std_srvs.srv import Empty
from joy_handle import JoyHandle

class RCSimulation:
    def __init__(self, rc_topic):
        self.pub = rospy.Publisher(rc_topic, OverrideRCIn, queue_size=1)

        centered_pwm = 1500
        self.rc_in = OverrideRCIn()
        self.rc_in.channels[0] = 900           # throttle (down!)
        self.rc_in.channels[1] = centered_pwm  # yaw
        self.rc_in.channels[2] = centered_pwm  # pitch
        self.rc_in.channels[3] = centered_pwm  # roll
        self.rc_in.channels[4] = 900           # fltmode  (down!)
        self.rc_in.channels[5] = centered_pwm  # NOT MAPPED
        self.rc_in.channels[6] = centered_pwm  # NOT MAPPED
        self.rc_in.channels[7] = centered_pwm  # NOT MAPPED
        rospy.Timer(rospy.Duration(0.1), self.update_callback)  # 10Hz

    def set_channel(self, channel_id, channel_pwm):
        if channel_id < 1 or channel_id > 8:  # channel id in [1, 8] 
            rospy.logerr("Invalid channel_id[%d]", channel_id)
            return
        if channel_pwm < 900 or channel_pwm > 2100:  # pwm in [900, 2100]
            rospy.logerr("PWM value [%d] out of bounds", channel_pwm)
            return
        self.rc_in.channels[channel_id - 1] = channel_pwm  # 0-indexing!

    def get_channel(self, channel_id):
        if channel_id < 1 or channel_id > 8:  # channel id in [1, 8] 
            rospy.logerr("Invalid channel_id[%d]", channel_id)
            return
        return self.rc_in.channels[channel_id - 1]

    def update_callback(self, event):
        self.pub.publish(self.rc_in)

class SafetyPilot:
    def __init__(self, joy_name, id=1):
        self.id = id
        self.ns = rospy.get_namespace()
        self.state_url = 'mavros/state'
        self.arming_url = 'mavros/cmd/arming'
        self.rc_url = 'mavros/rc/override'
        self.mavros_state = MavrosState()
        self.joy_is_connected = False
        action_file = rospkg.RosPack().get_path('ual_teleop') + '/config/simulate_safety_pilot.yaml'
        with open(action_file, 'r') as action_config:
            action_map = yaml.load(action_config)['joy_actions']
        self.joy_handle = JoyHandle(joy_name, action_map)
        self.rc_simulation = RCSimulation(self.rc_url)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_state = rospy.Subscriber(self.state_url, MavrosState, self.state_callback)
        rospy.wait_for_service(self.arming_url)  # TODO(franreal): wait here?
        self.arming_proxy = rospy.ServiceProxy(self.arming_url, CommandBool)
        rospy.Timer(rospy.Duration(1), self.arming_callback)  # 1Hz

    def arming_callback(self, event):
        if self.joy_is_connected:  # Disable auto-arming
            return

        if not self.mavros_state.armed and self.mavros_state.connected:
            try:
                rospy.loginfo("Safety pilot simulator for robot id [%d]: arming [%s]", self.id, self.ns + self.arming_url)
                self.arming_proxy(True)
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: %s", str(exc))

    def state_callback(self, data):
        self.mavros_state = data

    def joy_callback(self, data):
        if not self.joy_is_connected:
            rospy.loginfo("Joystick detected, RC simulation is now enabled (and auto-arming disabled)")
            self.joy_is_connected = True

        self.joy_handle.update(data)
        # print self.joy_handle  # DEBUG
        self.rc_simulation.set_channel(1, 1500 + 600*self.joy_handle.get_action_axis('throttle'))
        self.rc_simulation.set_channel(2, 1500 + 600*self.joy_handle.get_action_axis('yaw'))
        self.rc_simulation.set_channel(3, 1500 + 600*self.joy_handle.get_action_axis('pitch'))
        self.rc_simulation.set_channel(4, 1500 + 600*self.joy_handle.get_action_axis('roll'))
        if self.joy_handle.get_action_button('secure'):
            fltmode_pwm = self.rc_simulation.get_channel(5)
            if (self.joy_handle.get_action_button('set_mode_pwm_2100')):
                fltmode_pwm = 2100
            if (self.joy_handle.get_action_button('set_mode_pwm_1500')):
                fltmode_pwm = 1500
            if (self.joy_handle.get_action_button('set_mode_pwm_900')):  # This button has maximum priority (stabilized)
                fltmode_pwm = 900
            self.rc_simulation.set_channel(5, fltmode_pwm)                                        # fltmode

class BackendMavlinkAutoArm:
    def __init__(self, id=1):
        self.id = id
        self.ns = rospy.get_namespace()
        self.ual_state = UalState()
        self.arming_url = "ual/arm"
        self.sub_state = rospy.Subscriber("ual/state", UalState, self.state_callback)
        rospy.wait_for_service(self.arming_url)
        self.arming_proxy = rospy.ServiceProxy(self.arming_url, Empty)
        rospy.Timer(rospy.Duration(2), self.arming_callback)  # 0.5Hz

    def arming_callback(self, event):
        if self.ual_state.state == UalState.LANDED_DISARMED:
            try:
                rospy.loginfo("Safety pilot simulator for robot id [%d]: arming [%s]", self.id, self.ns + self.arming_url)
                self.arming_proxy()
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: %s", str(exc))
    
    def state_callback(self, data):
        self.ual_state = data


def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Simulate safety pilot. WARNING: use only in simulation!')
    parser.add_argument('-joy_name', type=str, default=None,
                        help='Joystick name, must have a equally named .yaml file in ual_teleop/config/joysticks folder')
    parser.add_argument('-id', type=int, default=1,
                        help='robot id')
    parser.add_argument('-use_mavros', type=str2bool, default=True,
                        help='Use mavros (True) or mavlink (False)')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    # Init ros node
    rospy.init_node('simulate_safety_pilot_{}'.format(args.id))

    # Check we are in a simulation
    is_sim = rospy.get_param('/use_sim_time', False)
    if not is_sim:
        rospy.logerr("Param /use_sim_time is false: Use safety pilot simulation only in simulation!")
        return

    if not args.use_mavros:
        rospy.loginfo("Safety pilot for BackendMavlink [%d]", args.id)
        safety_pilot = BackendMavlinkAutoArm(args.id)
        rospy.spin()
        return

    if args.joy_name is None:
        default_joy = 'saitek_p3200'
        rospy.loginfo("Using default joy [%s]", default_joy)
        args.joy_name = default_joy

    safety_pilot = SafetyPilot(args.joy_name, args.id)
    rospy.spin()

if __name__ == '__main__':
    main()
