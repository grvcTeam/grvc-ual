#!/usr/bin/env python
import subprocess

class bcolors:    
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def main():
    backends = {
        "mavros" : ["MAVROS", "    $ sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras \n    $ sudo geographiclib-get-geoids egm96-5"],
        "mavlink" : ["MAVLink", "    Download and install MAVSDK .deb: https://github.com/mavlink/MAVSDK/releases"],
        "gazebo_light" : ["Gazebo Light", ""],
        "dji_ros" : ["DJI ROS", "    Download and install DJI SDK core library: https://github.com/dji-sdk/Onboard-SDK \n    Download in your catkin workspace the DJI Onboard SDK ROS: https://github.com/dji-sdk/Onboard-SDK-ROS"],
        "crazyflie" : ["Crazyflie", "    Download and install crazyflie_ros: https://github.com/whoenig/crazyflie_ros"],
        "ue" : ["Unreal Engine", "    TBD"]
    }
    instructions = ""

    print bcolors.OKBLUE + bcolors.BOLD + """   ____ ______     ______    _   _   _    _     
  / ___|  _ \\ \\   / / ___|  | | | | / \\  | |    
 | |  _| |_) \\ \\ / / |      | | | |/ _ \\ | |    
 | |_| |  _ < \\ V /| |___   | |_| / ___ \\| |___ 
  \\____|_| \\_\\ \\_/  \\____|   \\___/_/   \\_\\_____|                                      
    """ + bcolors.ENDC
    print bcolors.BOLD + "> Select the backends that you want to use:" + bcolors.ENDC
    for b in backends:
        selected = raw_input("  - " + backends[b][0] + " [y/N]: ")
        if (selected == 'y' or selected == 'Y'):
            instructions += "\n* Backend " + backends[b][0] + ":\n" + backends[b][1] + "\n"
            subprocess.call("rm -f ual_backend_" + b + "/CATKIN_IGNORE", shell=True)
        else:
            subprocess.call("touch ual_backend_" + b + "/CATKIN_IGNORE", shell=True)
    
    # Print instructions
    print bcolors.BOLD + "\n> Instructions to build the selected backends:" + bcolors.ENDC
    print instructions
    print "* Further and detailed build instructions in https://github.com/grvcteam/grvc-ual/wiki ;)\n"

if __name__ == "__main__":
    main()
