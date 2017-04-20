#!/usr/bin/env python
import xml.etree.ElementTree as xml
import subprocess
import argparse
import os
import time
import signal
import utils
import rospkg


def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description="Launch gazebo world simulation, \
    similar to roslaunch gazebo_ros empty_world.launch with some needed extras for px4 sitl")
    parser.add_argument('-physics', type=str, default='ode', help="Gazebo physics engine to use")
    parser.add_argument('-world', type=str, default='worlds/empty.world',
                        help="World file name with respect to GAZEBO_RESOURCE_PATH")
    parser.add_argument('-frames', type=str, default="/config/map_simulation.xml",
                        help='path to frames xml file')
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

    # Get map origin lat-lon-alt from frames xml file
    frames_tree = xml.parse(rospack.get_path("px4_bringup") + args.frames)
    frames_root = frames_tree.getroot()
    origin_element = frames_root.find('origin')
    lat_element = origin_element.find('lat')
    lon_element = origin_element.find('lon')
    alt_element = origin_element.find('alt')

    # Set gazebo origin for px4 sitl mavlink interface plugin
    gz_env['PX4_HOME_LAT'] = lat_element.text
    gz_env['PX4_HOME_LON'] = lon_element.text
    gz_env['PX4_HOME_ALT'] = alt_element.text

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
