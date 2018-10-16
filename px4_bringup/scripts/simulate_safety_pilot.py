#!/usr/bin/env python
import argparse
import utils
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool

mavros_state = State()
def state_callback(data):
    global mavros_state
    mavros_state = data

def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description='Simulate safety pilot, arming the uav as soon as possible. WARNING: use only in simulation!')
    parser.add_argument('-id', type=int, default=1,
                        help='robot id')
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Init ros node
    rospy.init_node('simulate_safety_pilot_{}'.format(args.id))

    is_sim = rospy.get_param('/use_sim_time', False)
    if not is_sim:
        rospy.logerr("Param /use_sim_time is false: Use safety pilot simulation only in simulation!")
        return

    ns = rospy.get_namespace()
    state_url = 'mavros/state'
    arming_url = 'mavros/cmd/arming'
    # rospy.loginfo("Safety pilot simulator for robot id [%d]: listening to %s", args.id, ns + state_url)
    # rospy.loginfo("Safety pilot simulator for robot id [%d]: ready to call %s", args.id, ns + arming_url)
    # TODO(franreal): Check consistency of id and namespace?

    rospy.Subscriber(state_url, State, state_callback)

    rospy.wait_for_service(arming_url)
    arming_proxy = rospy.ServiceProxy(arming_url, CommandBool)
    # time.sleep(0.5)  # TODO(franreal): Sleep?

    rate = rospy.Rate(1)  # [Hz]
    while not rospy.is_shutdown():
        global mavros_state
        if not mavros_state.armed and mavros_state.connected:
            try:
                rospy.loginfo("Safety pilot simulator for robot id [%d]: arming [%s]", args.id, ns + arming_url)
                arming_proxy(True)
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: %s", str(exc))
        try:
            rate.sleep()
        except KeyboardInterrupt:
            pass

    return

if __name__ == "__main__":
    main()
