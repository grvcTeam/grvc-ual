#!/usr/bin/env python
import rospy
import time
import numpy as np
import math
import copy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from geographic_msgs.msg import *
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

        reference = "empty" ### Change to test different reference frames and geo: empty, custom, geo
        wps = []
        header_empty = std_msgs.msg.Header()
        header_geo = std_msgs.msg.Header(header_empty.seq, header_empty.stamp, "geo")
        header_custom = std_msgs.msg.Header(header_empty.seq, header_empty.stamp, "aux")
        ### TAKE OFF POSE
        take_off_wps = MissionElement()
        take_off_wps.type = MissionElement.TAKEOFF_POSE

        if reference == "empty" or reference == "custom":
            take_off_wps.waypoints = [PoseStamped(header_empty,Pose(Point(100,0,10),Quaternion(0,0,0,1)))]

        elif reference == "geo":
            take_off_wps.waypoints = [PoseStamped(header_geo,GeoPose(Point(37.558570091,-5.92994325434,10.0),Quaternion(0,0,0,1)))]

        take_off_wps.params = self.dictToListOfParamFloat({"minimum_pitch": 0.0})
        wps.append(take_off_wps)


        ## TAKE OFF AUX
        # header_empty = std_msgs.msg.Header()
        # take_off_wps = MissionElement()
        # take_off_wps.type = MissionElement.TAKEOFF_AUX
        # take_off_wps.params = self.dictToListOfParamFloat({"minimum_pitch": 0.0,
        #     "aux_distance": 100.0,"aux_angle":0.0,"aux_height": 10.0})
        # wps.append(take_off_wps)


        ### PASS
        pass_wps = MissionElement()
        pass_wps.type = MissionElement.PASS
        if reference == "empty":
            pass_wps.waypoints = [PoseStamped(header_empty,Pose(Point(150,-150,20),Quaternion(0,0,0,1)))]
            pass_wps.waypoints.append(PoseStamped(header_empty,Pose(Point(-150,-150,20),Quaternion(0,0,0,1))))
            pass_wps.waypoints.append(PoseStamped(header_empty,Pose(Point(-150,150,20),Quaternion(0,0,0,1))))

        elif reference == "custom":
            pass_wps.waypoints = [PoseStamped(header_custom,Pose(Point(150,-150,20),Quaternion(0,0,0,1)))]
            pass_wps.waypoints.append(PoseStamped(header_custom,Pose(Point(-150,-150,20),Quaternion(0,0,0,1))))
            pass_wps.waypoints.append(PoseStamped(header_custom,Pose(Point(-150,150,20),Quaternion(0,0,0,1))))

        elif reference == "geo":
            pass_wps.waypoints = [PoseStamped(header_geo,GeoPose(Point(37.5572521152,-5.92858999857,20.0),Quaternion(0,0,0,1)))]
            pass_wps.waypoints.append(PoseStamped(header_geo,GeoPose(Point(37.5571495841,-5.93271715684,20.0),Quaternion(0,0,0,1))))
            pass_wps.waypoints.append(PoseStamped(header_geo,GeoPose(Point(37.5598501062,-5.93282309407,20.0),Quaternion(0,0,0,1))))

        pass_wps.params = self.dictToListOfParamFloat({"acceptance_radius": 10.0,"orbit_distance": 0.0, "speed" : 10.0})
        wps.append(pass_wps)

        ### LOITER HEIGHT
        loit_wps1 = MissionElement()
        loit_wps1.type = MissionElement.LOITER_HEIGHT
        if reference == "empty":
            loit_wps1.waypoints = [PoseStamped(header_empty,Pose(Point(150,150,100),Quaternion(0,0,0,1)))]
            loit_wps1.waypoints.append(PoseStamped(header_empty,Pose(Point(150,150,20),Quaternion(0,0,0,1))))

        elif reference == "custom":
            loit_wps1.waypoints = [PoseStamped(header_custom,Pose(Point(150,150,100),Quaternion(0,0,0,1)))]
            loit_wps1.waypoints.append(PoseStamped(header_custom,Pose(Point(150,150,20),Quaternion(0,0,0,1))))

        elif reference == "geo":
            loit_wps1.waypoints = [PoseStamped(header_geo,GeoPose(Point(37.559934397,-5.92943078861,100.0),Quaternion(0,0,0,1)))]
            loit_wps1.waypoints.append(PoseStamped(header_geo,GeoPose(Point(37.559934397,-5.92943078861,20.0),Quaternion(0,0,0,1))))

        loit_wps1.params = self.dictToListOfParamFloat({"heading": 0.0,"radius": 10.0, "forward_moving" : 1.0})
        wps.append(loit_wps1)

        ### LOITER TURNS
        # loit_wps2 = MissionElement()
        # loit_wps2.type = MissionElement.LOITER_TURNS
        # if reference == "empty":
        #     loit_wps2.waypoints = [PoseStamped(header_empty,Pose(Point(200,0,20),Quaternion(0,0,0,1)))]

        # if reference == "custom":
        #     loit_wps2.waypoints = [PoseStamped(header_custom,Pose(Point(200,0,20),Quaternion(0,0,0,1)))]

        # elif reference == "geo":
        #     loit_wps2.waypoints = [PoseStamped(header_geo,GeoPose(Point(37.559934397,-5.92943078861,20.0),Quaternion(0,0,0,1)))]

        # loit_wps2.params = self.dictToListOfParamFloat({"turns": 5.0,"radius": 50.0, "forward_moving" : 1.0})
        # wps.append(loit_wps2)

        ### LAND POSE
        land_wps = MissionElement()
        land_wps.type = MissionElement.LAND_POSE

        if reference == "empty":
            land_wps.waypoints = [PoseStamped(header_empty,Pose(Point(100,0,10),Quaternion(0,0,0,1)))]
            land_wps.waypoints.append(PoseStamped(header_empty,Pose(Point(0,0,0),Quaternion(0,0,0,1))))

        elif reference == "custom":
            land_wps.waypoints = [PoseStamped(header_custom,Pose(Point(100,0,10),Quaternion(0,0,0,1)))]
            land_wps.waypoints.append(PoseStamped(header_custom,Pose(Point(0,0,0),Quaternion(0,0,0,1))))

        elif reference == "geo":
            land_wps.waypoints = [PoseStamped(header_geo,GeoPose(Point(37.558570091,-5.92994325434,10.0),Quaternion(0,0,0,1)))]
            land_wps.waypoints.append(PoseStamped(header_geo,GeoPose(Point(37.5585420009,-5.93107400325,0.0),Quaternion(0,0,0,1))))

        land_wps.params = self.dictToListOfParamFloat({"loit_heading": 0.0, "loit_radius": 0.0,
                                                       "loit_forward_moving": 1.0,"abort_alt": 0.0, "precision_mode": 0.0})
        wps.append(land_wps)

        ### LAND AUX
        # land_wps = MissionElement()
        # land_wps.type = MissionElement.LAND_AUX
        # land_wps.waypoints = [PoseStamped(header_empty,Pose(Point(0,0,0),Quaternion(0,0,0,1)))]
        # land_wps.params = self.dictToListOfParamFloat({"aux_distance": 100.0, "aux_angle": 0.0, "aux_height": 10.0,
        #                                                "loit_heading": 0.0, "loit_radius": 0.0,
        #                                                "loit_forward_moving": 1.0,"abort_alt": 0.0, "precision_mode": 0.0})
        # wps.append(land_wps)

        return wps

        
    def sendMission(self, wps):

        mission_srv_request = SetMissionRequest()
        mission_srv_request.mission_elements = wps
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