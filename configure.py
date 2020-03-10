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
        "mavros" : ["MAVROS", "    $ sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras \n    $ sudo geographiclib-get-geoids egm96-5", False],
        "mavlink" : ["MAVLink", "    Download and install MAVSDK .deb: https://github.com/mavlink/MAVSDK/releases", False],
        "gazebo_light" : ["Gazebo Light", "",False],
        "dji_ros" : ["DJI ROS", "    Download and install DJI SDK core library: https://github.com/dji-sdk/Onboard-SDK \n    Download in your catkin workspace the DJI Onboard SDK ROS: https://github.com/dji-sdk/Onboard-SDK-ROS", False],
        "crazyflie" : ["Crazyflie", "    Download and install crazyflie_ros: https://github.com/whoenig/crazyflie_ros", False],
        "ue" : ["Unreal Engine", "    TBD", False]
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
            if b != "mavros" and backends[b][1]:
                instructions += "\n* Backend " + backends[b][0] + ":\n" + backends[b][1] + "\n"
            subprocess.call("rm -f ual_backend_" + b + "/CATKIN_IGNORE", shell=True)
            backends[b][2] = True
        else:
            subprocess.call("touch ual_backend_" + b + "/CATKIN_IGNORE", shell=True)
    
    selected = raw_input(bcolors.BOLD + "\n> Would you like to install needed dependencies? " + bcolors.ENDC + "[y/N]: ")
    if (selected == 'y' or selected == 'Y'):
        subprocess.call("sudo apt install -y libeigen3-dev", shell=True)
        subprocess.call("sudo apt install -y ros-$(rosversion -d)-joy", shell=True)
        subprocess.call("sudo apt install -y ros-$(rosversion -d)-geodesy", shell=True)

        if backends["mavros"][2]:
            subprocess.call("sudo apt install -y ros-$(rosversion -d)-mavros", shell=True)
            subprocess.call("sudo apt install -y ros-$(rosversion -d)-mavros-extras", shell=True)
            subprocess.call("sudo geographiclib-get-geoids egm96-5", shell=True)
            subprocess.call("sudo usermod -a -G dialout $USER", shell=True)
            subprocess.call("sudo apt remove modemmanager", shell=True)
    else:
        if backends["mavros"][2]:
            instructions += "\n* Backend " + backends["mavros"][0] + ":\n" + backends["mavros"][1] + "\n"

    # Print instructions
    print bcolors.BOLD + "\n> Instructions to build the selected backends:" + bcolors.ENDC
    print instructions
    print "* Further and detailed build instructions in https://github.com/grvcteam/grvc-ual/wiki ;)\n"

if __name__ == "__main__":
    main()
