#!/usr/bin/env python
import subprocess
import argparse
import utils
import rospkg
import os


def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description='Spawn px4 controller for SITL')
    parser.add_argument('-model', type=str, default="mbzirc",
                        help='robot model name, must match xacro description folder name')
    parser.add_argument('-estimator', type=str, default="ekf2",
                        help='estimator to use')
    parser.add_argument('-id', type=int, default=1,
                        help='robot id, used to compute udp ports')
    parser.add_argument('-description_package', type=str, default="robots_description",
                        help='robot description package, must follow robots_description file structure')
    args, unknown = parser.parse_known_args()
    utils.check_unknown_args(unknown)

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Create temporary directory for robot sitl stuff
    temp_dir = utils.temp_dir(args.id)
    subprocess.call("rm -rf " + temp_dir + "/rootfs", shell=True)
    subprocess.call("mkdir -p " + temp_dir + "/rootfs", shell=True)

    # Get udp configuration, depending on id
    udp_config = utils.udp_config(args.id)

    # Modify commands file to fit robot ports
    commands_file = rospack.get_path(args.description_package) + "/models/" + args.model + "/px4cmd"
    modified_cmds = temp_dir + "/rootfs/cmds"

    # Create symlink to the PX4 command file
    px4_src = rospack.get_path("px4")
    custom_model_name = "custom_" + args.model
    px4_commands_dst = px4_src + "/ROMFS/px4fmu_common/init.d-posix/" + str(hash(custom_model_name) % 10**8) + "_" + custom_model_name
    subprocess.call("ln -sf " + commands_file + " " + px4_commands_dst, shell=True)
    if os.path.exists(commands_file + '.post'):
        subprocess.call("ln -sf " + commands_file + ".post " + px4_commands_dst + ".post", shell=True)

    # Set PX4 environment variables
    px4_env = os.environ.copy()
    px4_env['PX4_SIM_MODEL'] = custom_model_name
    px4_env['PX4_ESTIMATOR'] = args.estimator

    # Spawn px4
    px4_bin = px4_src + "/build/px4_sitl_default/bin/px4"
    px4_rootfs = px4_src + "/ROMFS/px4fmu_common"
    rcs_file = "etc/init.d-posix/rcS"
    px4_args = px4_bin + " " + px4_rootfs + " -s " + rcs_file + " -i " + str(args.id-1) + " -w " + temp_dir + "/rootfs"
    px4_out = open(temp_dir+"/px4.out", 'w')
    px4_err = open(temp_dir+"/px4.err", 'w')
    px4 = subprocess.Popen(px4_args, env=px4_env, shell=True, stdout=px4_out, stderr=px4_err, cwd=temp_dir)

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
