#!/usr/bin/env python
import argparse
import sys
import yaml
import rospy
import rospkg
from uav_abstraction_layer.srv import TakeOff, GoToWaypoint, Land
from geometry_msgs.msg import PoseStamped

def track_waypoints():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Track waypoints defined in a yaml file')
    parser.add_argument('-plan_package', type=str, default='uav_abstraction_layer',
                        help='Name of the package where plan to track is stored')
    parser.add_argument('-plan_file', type=str, default='wp_default.yaml',
                        help='Name of the file inside plan_package/plans')
    parser.add_argument('-wait_for', type=str, default='path',
                        help='Wait for human response: [none]/[path]/[wp]')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('waypoint_tracker')

    file_name = args.plan_file
    # Autocomplete file extension
    if not file_name.endswith('.yaml'):
        file_name = file_name + '.yaml'

    file_url = rospkg.RosPack().get_path(args.plan_package) + '/plans/' + file_name
    with open(file_url, 'r') as wp_file:
        wp_data = yaml.load(wp_file)

    if 'frame_id' not in wp_data:
        rospy.logerr("Must specify frame_id in waypoints file")  # TODO: default to ''?
        return

    wp_list = []
    for wp_id in range(1000):
        if 'wp_' + str(wp_id) in wp_data:
            wp_raw = wp_data['wp_' + str(wp_id)]
            waypoint = PoseStamped()
            waypoint.header.frame_id = wp_data['frame_id']
            waypoint.pose.position.x =    wp_raw[0]
            waypoint.pose.position.y =    wp_raw[1]
            waypoint.pose.position.z =    wp_raw[2]
            waypoint.pose.orientation.x = wp_raw[3]
            waypoint.pose.orientation.y = wp_raw[4]
            waypoint.pose.orientation.z = wp_raw[5]
            waypoint.pose.orientation.w = wp_raw[6]
            wp_list.append(waypoint)

    go_to_waypoint_url = 'ual/go_to_waypoint'
    rospy.wait_for_service(go_to_waypoint_url)

    try:
        go_to_waypoint = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)

        # TODO: Check we're flying!
        print "Ready to track " + str(len(wp_list)) + " waypoints from " + file_url
        if args.wait_for == 'path' or args.wait_for == 'wp':
            answer = raw_input("Continue? (y/N): ").lower().strip()
            if answer != 'y' and answer != 'yes':
                print "Aborted"
                return

        for waypoint in wp_list:
            print "Go to waypoint:"
            print waypoint
            go_to_waypoint(waypoint, True)
            if args.wait_for == 'wp':
                raw_input("Arrived. Press Enter to continue...")

        return

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    track_waypoints()
