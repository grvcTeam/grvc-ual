#!/usr/bin/env python
import xml.etree.ElementTree as xml
import subprocess
import argparse
import utils
import numpy
import rospkg


def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description='Spawn robot in Gazebo for SITL')
    parser.add_argument('-model', type=str, default="mbzirc",
                        help='robot model name, must match xacro description folder name')
    parser.add_argument('-id', type=int, default=1,
                        help='robot id, used to compute sim_port')
    parser.add_argument('-frames', type=str, default="/config/map_simulation.xml",
                        help='path to frames xml file')
    parser.add_argument('-material', type=str, default="DarkGrey",
                        help='robot Gazebo/material; \
                        see materials/scripts/gazebo.material (at your gazebo version)')
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Xacro description must be in robots_description package
    description_dir = rospack.get_path("robots_description")
    xacro_description = description_dir + "/models/" + args.model + "/model.xacro"

    # Create temporary directory for robot sitl stuff
    temp_dir = utils.temp_dir(args.id)
    subprocess.call("mkdir -p " + temp_dir, shell=True)

    # Get udp configuration, depending on id
    udp_config = utils.udp_config(args.id)

    # Create urdf from xacro description
    temp_urdf = temp_dir + "/" + args.model + ".urdf"
    xacro_args = "xacro --inorder -o " + temp_urdf + " " + \
    xacro_description + \
    " enable_mavlink_interface:=true" + \
    " enable_ground_truth:=false" + \
    " enable_logging:=false" + \
    " enable_camera:=false" + \
    " enable_wind:=false" + \
    " mavlink_udp_port:=" + str(udp_config["sim_port"]) + \
    " visual_material:=" + args.material
    xacro_out = open(temp_dir+"/xacro.out", 'w')
    xacro_err = open(temp_dir+"/xacro.err", 'w')
    subprocess.call(xacro_args, shell=True, stdout=xacro_out, stderr=xacro_err)
    xacro_out.close()
    xacro_err.close()

    # Create sdf from urdf
    temp_sdf = temp_dir + "/" + args.model + ".sdf"
    subprocess.call("gz sdf -p " + temp_urdf + " > " + temp_sdf, shell=True)

    # Get robot home position from frames xml file
    frames_tree = xml.parse(rospack.get_path("px4_bringup") + args.frames)
    frames_root = frames_tree.getroot()
    for robothome in frames_root.findall('robothome'):
        if robothome.get('id') == str(args.id):
            x_element = robothome.find('x')
            y_element = robothome.find('y')
            z_element = robothome.find('z')
            yaw_element = robothome.find('yaw')

    # Get rotation from frames xml file
    game_element = frames_root.find('game')
    rxx_element = game_element.find('rxx')
    rxy_element = game_element.find('rxy')
    ryx_element = game_element.find('ryx')
    ryy_element = game_element.find('ryy')
    u_matrix = numpy.matrix([[float(rxx_element.text), \
                              float(rxy_element.text)], \
                             [float(ryx_element.text), \
                              float(ryy_element.text)]])

    # Calculate robot x-y position in map frame
    robot_xy = numpy.matrix([[float(x_element.text)], \
                             [float(y_element.text)]])
    robot_xy = numpy.dot(u_matrix, robot_xy)

    # Spawn robot sdf in gazebo
    gzmodel_args = "gz model -f " + temp_sdf + \
    " -m " + args.model + "_" + str(args.id) + \
    " -x " + str(robot_xy.item(0)) + \
    " -y " + str(robot_xy.item(1)) + \
    " -z " + str(float(z_element.text)) + \
    " -Y " + str(float(yaw_element.text))
    subprocess.call(gzmodel_args, shell=True)

if __name__ == "__main__":
    main()
