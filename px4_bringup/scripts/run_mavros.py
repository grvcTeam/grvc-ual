#!/usr/bin/env python
import subprocess
import argparse
import utils
import rospkg
import rospy

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
    parser = argparse.ArgumentParser(description='Spawn mavros node')
    parser.add_argument('-id', type=int, default=1,
                        help='robot id, used to compute udp ports')
    parser.add_argument('-mode', type=str, default="sitl",
                        help='robot mode, used to set proper fcu_url')
    parser.add_argument('-fcu_url', type=str, default="udp://:14550@localhost:14556",
                        help='set fcu_url manually in custom mode')
    parser.add_argument('-gcs_url', type=str, default="",
                        help='set gcs_url manually in custom mode')
    parser.add_argument('-rtcm_topic', type=str, default="",
                        help='set topic for gps rtk corrections')
    parser.add_argument('-remap_tfs', type=str2bool, default=True,
                        help='remap /tf and /tf_static to namespaced topics')
    parser.add_argument('-mavros_config_pkg', type=str, default="px4_bringup",
                        help='set the package in which a config folder contains the files px4_pluginlists.yaml and px4_config.yaml')
    #!TODO: Add yaml location as arguments
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Create temporary directory for robot sitl stuff
    temp_dir = utils.temp_dir(args.id)
    subprocess.call("mkdir -p " + temp_dir, shell=True)

    # Namespace  
    ns = rospy.get_namespace()

    # Set a param to tell the system current spawn mode
    run_ns = ns + "/run_mavros"
    subprocess.call("rosparam set " + run_ns + "/mode " + args.mode, shell=True)

    # Get udp configuration, depending on id
    udp_config = utils.udp_config(args.id)

    # Set params for mavros...
    node_name = ns + "/mavros"
    if args.mode == "sitl":
        fcu_url = "udp://:" + str(udp_config["udp_offboard_port_remote"]) + \
        "@127.0.0.1:" + str(udp_config["udp_offboard_port_local"])
        subprocess.call("rosparam set " + node_name + "/fcu_url " + fcu_url, shell=True)
        subprocess.call("rosparam set " + node_name + "/gcs_url " + \
        udp_config["gcs_url"], shell=True)
        # Set target MAV_SYSTEM_ID, only on sitl. Consider extension to other modes.
        subprocess.call("rosparam set " + node_name + "/target_system_id " + str(args.id), shell=True)
    elif args.mode == "serial":
        fcu_url = "serial:///dev/ttyUSB0:921600"
        subprocess.call("rosparam set " + node_name + "/fcu_url " + fcu_url, shell=True)
        if args.gcs_url:
            subprocess.call("rosparam set " + node_name + "/gcs_url " + args.gcs_url, shell=True)
    elif args.mode =="custom":
        subprocess.call("rosparam set " + node_name + "/fcu_url " + args.fcu_url, shell=True)
        if args.gcs_url:
            subprocess.call("rosparam set " + node_name + "/gcs_url " + args.gcs_url, shell=True)

    # ...and load blacklist, config (as seen in mavros node.launch)
    yaml_path = rospack.get_path( args.mavros_config_pkg ) + "/config/"
    subprocess.call("rosparam load " + yaml_path + "px4_pluginlists.yaml " + \
    node_name, shell=True)
    subprocess.call("rosparam load " + yaml_path + "px4_config.yaml " + \
    node_name, shell=True)

    # Finally rosrun mavros
    rosrun_args = "rosrun mavros mavros_node __name:=" + "mavros" + " __ns:=" + ns
    if args.rtcm_topic:
        rosrun_args = rosrun_args + " mavros/gps_rtk/send_rtcm:=" + args.rtcm_topic
    if args.remap_tfs:
        rosrun_args = rosrun_args + " /tf:=mavros/tf /tf_static:=mavros/tf_static"
    rosrun_out = open(temp_dir+"/mavros.out", 'w')
    try:
        subprocess.call(rosrun_args, shell=True, stdout=rosrun_out)
    except KeyboardInterrupt:
        pass
    finally:
        rosrun_out.close()


if __name__ == "__main__":
    main()
