#!/usr/bin/env python
import subprocess
import argparse
import os
import time
import signal
import utils
import rospkg
import rospy


def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description="Launch gazebo world simulation, \
    similar to roslaunch gazebo_ros empty_world.launch with some needed extras for px4 sitl")
    parser.add_argument('-physics', type=str, default='ode', help="Gazebo physics engine to use")
    parser.add_argument('-world', type=str, default='worlds/empty.world',
                        help="World file name with respect to GAZEBO_RESOURCE_PATH")
    parser.add_argument('-add_model_path', type=str, default='',
                        help="Path to add to GAZEBO_MODEL_PATH")
    parser.add_argument('-description_package', type=str, default="robots_description",
                        help='robot description package, must follow robots_description file structure')
    parser.add_argument('-debug', type=bool, default=False,
                        help='run gzserver with gdb')
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    px4_dir = rospack.get_path('px4')

    # Set environment variables as in px4/Firmware/Tools/setup_gazebo.bash
    gz_env = os.environ.copy()
    current_gz_plugin_path = gz_env.get('GAZEBO_PLUGIN_PATH', '')
    gz_env['GAZEBO_PLUGIN_PATH'] = px4_dir + '/build/px4_sitl_default/build_gazebo' + \
                                   ':' + current_gz_plugin_path
    current_gz_model_path = gz_env.get('GAZEBO_MODEL_PATH', '')
    # Always include robots_description parent path
    robots_description_parent_path = os.path.abspath(os.path.join(\
                                              rospack.get_path('robots_description'), os.pardir))
    gz_env['GAZEBO_MODEL_PATH'] = px4_dir + '/Tools/sitl_gazebo/models' + \
                                 ':' + robots_description_parent_path + \
                                 ':' + current_gz_model_path
    # Include description_package parent path if it is not robots_description
    if args.description_package is not "robots_description":
        description_package_parent_path = os.path.abspath(os.path.join(\
                                              rospack.get_path(args.description_package), os.pardir))
        gz_env['GAZEBO_MODEL_PATH'] += ':' + description_package_parent_path

    # TODO: add_model_path should be a list of paths? Or just a string separated by ':'?
    if args.add_model_path:
        gz_env['GAZEBO_MODEL_PATH'] += ':' + args.add_model_path

    # print "gz_env['GAZEBO_MODEL_PATH'] = [%s]" % gz_env['GAZEBO_MODEL_PATH']  # debug

    # Get map origin lat-lon-alt from rosparam
    rospy.init_node('gazebo_world')
    if rospy.has_param('~sim_origin'):
        latlonalt = rospy.get_param('~sim_origin')
    else:
        latlonalt = [0.0, 0.0, 0.0]

    # Set gazebo origin for px4 sitl mavlink interface plugin
    gz_env['PX4_HOME_LAT'] = str(latlonalt[0])
    gz_env['PX4_HOME_LON'] = str(latlonalt[1])
    gz_env['PX4_HOME_ALT'] = str(latlonalt[2])

    # Set use_sim_time flag to true
    subprocess.call("rosparam set /use_sim_time true", shell=True)

    # Create temporary directory for robot sitl stuff (id=None)
    temp_dir = utils.temp_dir(None)
    subprocess.call("mkdir -p " + temp_dir, shell=True)

    # Start gazebo server
    server_args = "rosrun gazebo_ros gzserver -e " + args.physics + ' ' + args.world + ' __name:=gazebo'
    server_out = open(temp_dir + '/gzserver.out', 'w')
    server_err = open(temp_dir + '/gzserver.err', 'w')
    if not args.debug:
        server = subprocess.Popen(server_args, stdout=server_out, stderr=server_err, cwd=temp_dir, \
                                           env=gz_env, shell=True, preexec_fn=os.setsid)
    else:
        server = subprocess.Popen('gnome-terminal -- ' + server_args, cwd=temp_dir, \
                                            env=gz_env, shell=True, preexec_fn=os.setsid)

    # Start gazebo client
    time.sleep(0.2)
    client_args = "rosrun gazebo_ros gzclient __name:=gazebo_gui"
    client_out = open(temp_dir + '/gzclient.out', 'w')
    client_err = open(temp_dir + '/gzclient.err', 'w')
    client = subprocess.Popen(client_args, stdout=client_out, stderr=client_err, cwd=temp_dir, \
                                           env=gz_env, shell=True, preexec_fn=os.setsid)

    rospy.spin()  # Now I'm a ros node, just wait

    # Kill'em all
    if client.poll() is None:
        os.killpg(os.getpgid(client.pid), signal.SIGTERM)  # TODO: SIGKILL?
    if server.poll() is None:
        os.killpg(os.getpgid(server.pid), signal.SIGTERM)  # TODO: SIGKILL?
    # Close log files
    client_out.close()
    client_err.close()
    server_out.close()
    server_err.close()


if __name__ == '__main__':
    main()
