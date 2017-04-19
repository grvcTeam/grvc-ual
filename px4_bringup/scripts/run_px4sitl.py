#!/usr/bin/env python
import subprocess
import argparse
import utils
import rospkg


def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description='Spawn px4 controller for SITL')
    parser.add_argument('-model', type=str, default="mbzirc",
                        help='robot model name, must match xacro description folder name')
    parser.add_argument('-id', type=int, default=1,
                        help='robot id, used to compute udp ports')
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Create temporary directory for robot sitl stuff
    temp_dir = utils.temp_dir(args.id)
    subprocess.call("mkdir -p " + temp_dir, shell=True)

    # Get udp configuration, depending on id
    udp_config = utils.udp_config(args.id)

    # Modify commands file to fit robot ports
    commands_file = rospack.get_path("robots_description") + "/models/" + args.model + "/px4cmd"
    modified_cmds = temp_dir + "/cmds"
    with open(commands_file, 'r') as origin, open(modified_cmds, 'w') as modified:
        for line in origin:
            modified_line = line\
            .replace("_SIMPORT_", str(udp_config["sim_port"]))\
            .replace("_MAVPORT_", str(udp_config["u_port"][0]))\
            .replace("_MAVPORT2_", str(udp_config["u_port"][1]))\
            .replace("_MAVOPORT_", str(udp_config["o_port"][0]))\
            .replace("_MAVOPORT2_", str(udp_config["o_port"][1]))
            modified.write(modified_line)

    # Spawn px4
    px4_src = rospack.get_path("px4")
    px4_bin = px4_src + "/build_posix_sitl_default/src/firmware/posix/px4"
    px4_args = px4_bin + " " + px4_src + " " + modified_cmds
    px4_out = open(temp_dir+"/px4.out", 'w')
    px4_err = open(temp_dir+"/px4.err", 'w')
    px4 = subprocess.Popen(px4_args, shell=True, stdout=px4_out, stderr=px4_err, cwd=temp_dir)

    # Wait for it!
    try:
        px4.wait()
    except KeyboardInterrupt:
        pass
    finally:
        px4_out.close()
        px4_err.close()


if __name__ == "__main__":
    main()
