#!/usr/bin/env python
import subprocess
import argparse
import utils
import rospkg
import rospy


def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description='Spawn mavros node')
    parser.add_argument('-id', type=int, default=1,
                        help='robot id, used to compute udp ports')
    parser.add_argument('-mode', type=str, default="sitl",
                        help='robot mode, used to set proper fcu_url')
    parser.add_argument('-target_ip', type=str, default="localhost",
                        help='IP address of the device running px4, used to set proper fcu_url')
    parser.add_argument('-own_ip', type=str, default="localhost",
                        help='IP address of this device, used to set proper fcu_url')
    parser.add_argument('-ns_prefix', type=str, default="uav_",
                        help='namespace prefix')
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Create temporary directory for robot sitl stuff
    temp_dir = utils.temp_dir(args.id)
    subprocess.call("mkdir -p " + temp_dir, shell=True)

    # Set a param to tell the system current spawn mode
    run_ns = "run_mavros/" + args.ns_prefix + str(args.id)
    subprocess.call("rosparam set " + run_ns + "/mode " + args.mode, shell=True)

    # Namespace
    if rospy.get_namespace()=="/":
        ns = args.ns_prefix + str(args.id)
    else:
        ns = rospy.get_namespace() + "/" + args.ns_prefix + str(args.id)

    # Get udp configuration, depending on id
    udp_config = utils.udp_config(args.id)

    # Set params for mavros...
    node_name = ns + "/mavros"
    if args.mode == "sitl":
        fcu_url = "udp://:" + str(udp_config["o_port"][1]) + \
        "@127.0.0.1:" + str(udp_config["u_port"][1])
        subprocess.call("rosparam set " + node_name + "/fcu_url " + fcu_url, shell=True)
        subprocess.call("rosparam set " + node_name + "/gcs_url " + \
        udp_config["gcs_url"], shell=True)
    elif args.mode == "serial":
        fcu_url = "serial:///dev/ttyACM0:57600"
        subprocess.call("rosparam set " + node_name + "/fcu_url " + fcu_url, shell=True)
    elif args.mode == "udp":
        # TODO: get ports from args?
        fcu_url = "udp://:14550@{}:14556".format(args.target_ip)
        subprocess.call("rosparam set " + node_name + "/fcu_url " + fcu_url, shell=True)
        subprocess.call("rosparam set " + node_name + "/gcs_url " + "udp://@{}".format(args.own_ip), shell=True)

    # ...and load blacklist, config (as seen in mavros node.launch)
    yaml_path = rospack.get_path("px4_bringup") + "/config/"
    subprocess.call("rosparam load " + yaml_path + "px4_pluginlists.yaml " + \
    node_name, shell=True)
    subprocess.call("rosparam load " + yaml_path + "px4_config.yaml " + \
    node_name, shell=True)

    # Finally rosrun mavros
    rosrun_args = "rosrun mavros mavros_node __name:=" + "mavros" + " __ns:=" + ns
    rosrun_out = open(temp_dir+"/mavros.out", 'w')
    rosrun_err = open(temp_dir+"/mavros.err", 'w')
    try:
        subprocess.call(rosrun_args, shell=True, stdout=rosrun_out, stderr=rosrun_err)
    except KeyboardInterrupt:
        pass
    finally:
        rosrun_out.close()
        rosrun_err.close()


if __name__ == "__main__":
    main()
