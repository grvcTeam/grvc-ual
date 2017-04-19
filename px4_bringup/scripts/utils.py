#!/usr/bin/env python


def temp_dir(id):
    path = "/tmp/px4_sitl_files"
    if id:
        path += "/id_" + str(id)
    return path

def udp_config(id):
    init_port = 14560 + 10*(id-1)
    config = {}
    config["sim_port"] = init_port
    config["u_port"] = [init_port+1, init_port+2]
    config["o_port"] = [init_port+3, init_port+4]
    config["gcs_url"] = "udp://:" + str(init_port+5) + \
    "@127.0.0.1:" + str(init_port+6)
    return config

def check_unknown_args(unknown):
    for arg in unknown:
        if arg[0] == '-':
            raise SyntaxWarning("Unexpected argument " + arg)

# def fcu_url(id, mode)


if __name__ == "__main__":
    print "This is a utils collection, not a script!"
