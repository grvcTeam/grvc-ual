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
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    px4_dir = rospack.get_path('px4')

    # Set environment variables as in px4/Firmware/Tools/setup_gazebo.bash
    gz_env = os.environ.copy()
    current_gz_plugin_path = gz_env.get('GAZEBO_PLUGIN_PATH', '')
    gz_env['GAZEBO_PLUGIN_PATH'] = px4_dir + '/build_posix_sitl_default/build_gazebo' + \
                                   ':' + current_gz_plugin_path
    current_gz_model_path = gz_env.get('GAZEBO_MODEL_PATH', '')
    description_parent_path = os.path.abspath(os.path.join(\
                                              rospack.get_path('robots_description'), os.pardir))
    gz_env['GAZEBO_MODEL_PATH'] = px4_dir + '/Tools/sitl_gazebo/models' + \
                                 ':' + description_parent_path + \
                                 ':' + current_gz_model_path
    # TODO: add_model_path should be a list of paths?
    if args.add_model_path:
        gz_env['GAZEBO_MODEL_PATH'] += ':' + args.add_model_path

    # Get map origin lat-lon-alt from rosparam
    if rospy.has_param('/map_frame'):
        map_frame_dict = rospy.get_param('/map_frame')
        if map_frame_dict['units']=='GPS':
            latlonalt = map_frame_dict['translation']
        else:
            #TODO UTM to LAT-LON
            pass
    elif rospy.has_param('/sim_origin'):
        latlonalt = rospy.get_param('/sim_origin')
    else:
        latlonalt = [0.0, 0.0, 0.0]

    # Set gazebo origin for px4 sitl mavlink interface plugin
    gz_env['PX4_HOME_LAT'] = str(latlonalt[0])
    gz_env['PX4_HOME_LON'] = str(latlonalt[1])
    gz_env['PX4_HOME_ALT'] = str(latlonalt[2])

    # Set set use_sim_time flag to true
    subprocess.call("rosparam set /use_sim_time true", shell=True)

    # Create temporary directory for robot sitl stuff (id=None)
    temp_dir = utils.temp_dir(None)
    subprocess.call("mkdir -p " + temp_dir, shell=True)

    # Start gazebo server
    server_args = "rosrun gazebo_ros gzserver -e " + args.physics + ' ' + args.world
    server_out = open(temp_dir + '/gzserver.out', 'w')
    server_err = open(temp_dir + '/gzserver.err', 'w')
    server = subprocess.Popen(server_args, stdout=server_out, stderr=server_err, cwd=temp_dir, \
                                           env=gz_env, shell=True, preexec_fn=os.setsid)

    # Start gazebo client
    client_args = "rosrun gazebo_ros gzclient"
    client_out = open(temp_dir + '/gzclient.out', 'w')
    client_err = open(temp_dir + '/gzclient.err', 'w')
    client = subprocess.Popen(client_args, stdout=client_out, stderr=client_err, cwd=temp_dir, \
                                           env=gz_env, shell=True, preexec_fn=os.setsid)

    # Wait for server and client
    try:
        client.wait()
        server.wait()
    except KeyboardInterrupt:
        time.sleep(1)
        if server.poll() is None:
            os.killpg(os.getpgid(server.pid), signal.SIGTERM)
        if client.poll() is None:
            os.killpg(os.getpgid(client.pid), signal.SIGTERM)
    finally:
        # Clean up
        client_out.close()
        client_err.close()
        server_out.close()
        server_err.close()


if __name__ == '__main__':
    main()
