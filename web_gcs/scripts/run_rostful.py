#!/usr/bin/env python
import argparse
import subprocess
import rospkg
import rospy


def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description='Run rostful script')
    parser.add_argument('-ual_ns', type=str, default='',
                        help='UAL namespace')
    args, unknown = parser.parse_known_args()

    ual_ns = []
    rospy.init_node('run_rostful')
    if rospy.has_param('~ual_ns'):
        ual_ns = rospy.get_param('~ual_ns')
    else:
        ual_ns = ['/']

    # Set topics and services
    topics = ''
    services = ''
    first_pass = True
    for ns in ual_ns:
        if not(first_pass):
            topics += ","
        topics += "'"+ ns +"/ual/state'"
        topics += ",'"+ ns +"/ual/pose'"
        topics += ",'"+ ns +"/ual/velocity'"
        if not(first_pass):
            services += ","
        services += "'"+ ns +"/ual/take_off'"
        services += ",'"+ ns +"/ual/land'"
        services += ",'"+ ns +"/ual/go_to_waypoint'"
        first_pass = False
    
    rospy.loginfo("topics: "+topics+"\t services: "+services)
    
    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Modify config file
    config_file = rospack.get_path("web_gcs") + "/config/rostful.cfg"
    modified_config = "/tmp/px4_sitl_files/rostful.cfg"
    with open(config_file, 'r') as origin, open(modified_config, 'w') as modified:
        for line in origin:
            modified_line = line\
            .replace("UAL_TOPICS", topics)\
            .replace("UAL_SERVICES", services)
            modified.write(modified_line)
    
    # Run rostful
    rostful_args = "python -m rostful run -c " + modified_config

    try:
        subprocess.call(rostful_args, shell=True)
    except KeyboardInterrupt:
        pass
    finally:
        pass


if __name__ == "__main__":
    main()
