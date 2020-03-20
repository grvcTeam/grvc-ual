#!/usr/bin/env python3.5
import rospy
import time
import numpy as np
import math
import copy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_srvs.srv import *
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import *

class FwQgc(object):
    def __init__(self):

        rospy.init_node('fw_qgc', anonymous=True)

        # self.listener()

        mission_wps = self.createMission()

        self.sendMission(mission_wps)


    def createMission(self):

        wps = []
        header = std_msgs.msg.Header()
        take_off_wps = WaypointSet()
        take_off_wps.type = WaypointSet.TAKEOFF_AUX
        take_off_wps.posestamped_list = [PoseStamped(header,Pose(Point(100,0,20),Quaternion(0,0,0,1)))]
        take_off_wps.params = self.dictToListOfParamFloat({"minimum_pitch": 0.0,"yaw_angle": 0.0,
            "aux_distance": 100.0,"aux_angle":0.0,"aux_height": 10.0})
        wps.append(take_off_wps)

        pass_wps = WaypointSet()
        pass_wps.type = WaypointSet.PASS
        pass_wps.posestamped_list = [PoseStamped(header,Pose(Point(100,-100,20),Quaternion(0,0,0,1)))]
        pass_wps.posestamped_list.append(PoseStamped(header,Pose(Point(-100,-100,20),Quaternion(0,0,0,1))))
        pass_wps.posestamped_list.append(PoseStamped(header,Pose(Point(-100,100,20),Quaternion(0,0,0,1))))
        pass_wps.posestamped_list.append(PoseStamped(header,Pose(Point(100,100,20),Quaternion(0,0,0,1))))
        pass_wps.params = self.dictToListOfParamFloat({"acceptance_radius": 10.0,"orbit_distance": 0.0, "yaw_angle": 0.0})
        wps.append(pass_wps)

        land_wps = WaypointSet()
        land_wps.type = WaypointSet.LAND_AUX
        land_wps.posestamped_list = [PoseStamped(header,Pose(Point(0,0,0),Quaternion(0,0,0,1)))]
        land_wps.params = self.dictToListOfParamFloat({"aux_distance": 100.0, "aux_angle": 0.0, "aux_height": 10.0})
        wps.append(land_wps)

        return wps

        
    def sendMission(self, wps):

        mission_srv_request = SetMissionRequest()
        mission_srv_request.waypoint_sets = wps
        mission_srv_request.blocking = True

        self.serverClient(mission_srv_request, "/ual/set_mission", SetMission)



    def serverClient(self, request, address, Type, print_request = False, print_response = False):

        if print_request == True:
            print(request)

        rospy.wait_for_service(address)

        try:
            client = rospy.ServiceProxy(address, Type)
            response = client(request)
            if print_response == True:
                print(response)

            return response
        
        except rospy.ServiceException:
            return "Error"


    def dictToListOfParamFloat(self, dict):

        paramfloat_list = []
        for key in dict.keys():
            paramfloat_list.append(ParamFloat(key, dict[key]))

        return paramfloat_list

if __name__ == "__main__":
    FwQgc()