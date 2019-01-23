#!/usr/bin/env python
import argparse
import subprocess
import sys
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped

current_pose = PoseStamped()
def pose_callback(data):
    global current_pose
    current_pose = data

def save_waypoints():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Save waypoints to a yaml file')
    parser.add_argument('-plan_package', type=str, default='uav_abstraction_layer',
                        help='Name of the package where generated plan will be stored')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('waypoint_saver')

    # Subscribe to pose topic
    pose_url = 'ual/pose'
    rospy.Subscriber(pose_url, PoseStamped, pose_callback)

    # Saved waypoints file name
    file_name = raw_input("Enter saved waypoints file name (leave empty to use default): ")
    if file_name is '':
        file_name = 'wp_default.yaml'

    # Autocomplete file extension
    if not file_name.endswith('.yaml'):
        file_name = file_name + '.yaml'

    # Check we are actually receiving pose
    rospy.loginfo('Reading pose from: %s', pose_url)
    if current_pose.header.frame_id is '':
        rospy.logerr('Unable to read pose, assure it is being published!')
        return

    # Assure that plans folder exists in plan_package
    plans_dir = rospkg.RosPack().get_path(args.plan_package) + '/plans/'
    subprocess.call("mkdir -p " + plans_dir, shell=True)

    # Open yaml file and save frame_id
    frame_id = current_pose.header.frame_id
    rospy.loginfo('All waypoints in frame_id: %s', frame_id)
    file_url = plans_dir + file_name
    yaml_file = open(file_url, 'w')
    yaml_file.write('frame_id: ' + frame_id + '\n')
    rospy.loginfo('Waypoints will be saved to: %s', file_url)

    for wp_id in range(1000):
        if raw_input("Press Enter to save current pose or q to quit... ") is 'q':
            break

        # print(current_pose) # debug!
        wp_to_save = 'wp_' + str(wp_id) + ': [' + \
            str(current_pose.pose.position.x) + ', ' + \
            str(current_pose.pose.position.y) + ', ' + \
            str(current_pose.pose.position.z) + ', ' + \
            str(current_pose.pose.orientation.x) + ', ' + \
            str(current_pose.pose.orientation.y) + ', ' + \
            str(current_pose.pose.orientation.z) + ', ' + \
            str(current_pose.pose.orientation.w) + ']'

        # Check waypoint frame_id
        current_frame_id = current_pose.header.frame_id
        if current_frame_id != frame_id:
            rospy.logerr("Waypoint NOT SAVED: Current frame_id [%s] differs from expected [%s]", current_frame_id, frame_id)
            continue
        rospy.loginfo('Saving current pose as %s', wp_to_save)
        yaml_file.write(wp_to_save + '\n')

    if wp_id > 0:
        rospy.loginfo('All waypoints saved to: %s', file_url)
    yaml_file.close()
    return


if __name__ == "__main__":
    save_waypoints()
