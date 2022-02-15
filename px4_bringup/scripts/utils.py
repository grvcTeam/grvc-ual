#!/usr/bin/env python


def temp_dir(id):
    path = "/tmp/px4_sitl_files"
    if id:
        path += "/id_" + str(id)
    return path

def udp_config(id):
    px4_id = id-1
    config = {}
    config["gcs_url"] = "udp://@127.0.0.1:" + str(18570+px4_id)
    config["simulator_tcp_port"] = 4560+px4_id
    config["simulator_udp_port"] = 14560+px4_id
    config["udp_offboard_port_local"] = 14580+px4_id
    config["udp_offboard_port_remote"] = 14540+px4_id
    config["udp_onboard_payload_port_local"] = 14280+px4_id
    config["udp_onboard_payload_port_remote"] = 14030+px4_id
    return config

def check_unknown_args(unknown):
    for arg in unknown:
        if arg[0] == '-':
            raise SyntaxWarning("Unexpected argument " + arg)

# def fcu_url(id, mode)


if __name__ == "__main__":
    print("This is a utils collection, not a script!")
