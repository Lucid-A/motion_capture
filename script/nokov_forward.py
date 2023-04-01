#! /usr/bin/python3

import numpy as np
import rospy as ros
from geometry_msgs.msg import PoseStamped
from nokov.nokovsdk import *
from math import asin

class rosnokov():
    def __init__(self, serverIp : str):
        self.nokov = "nokov"
        self.map = "map"

        names = ros.get_param("~name")
        self.tf_broadcast = ros.get_param("~tf_broadcast")
        self.T_nokov_map = ros.get_param("~static_transform")

        self.pub = dict()
        print("Names from yaml")
        for name in names:
            print(name)
            self.pub.update({name : ros.Publisher(f"{name}/nokov", PoseStamped, queue_size = 10)})
        print(f"tf_broadcast: {self.tf_broadcast}")
        
        if not self.T_nokov_map:
            self.frame_id = self.nokov
        else:
            self.frame_id = self.map
            self.T_nokov_map = np.array(self.T_nokov_map)
            print(self.T_nokov_map)
        print(self.frame_id)

        self.client = PySDKClient()
        self.client.PySetVerbosityLevel(0)
        self.client.PySetMessageCallback(self.msg_func)
        self.client.PySetDataCallback(self.data_func, None)

        ret = self.client.Initialize(bytes(serverIp, encoding = "utf8"))
        if ret == 0:
            print(f"Connect to the Server at {serverIp} Succeed")
        else:
            print(f"Connect to {serverIp} Failed: [{ret}]")
            exit(0)

        self.preFrmNo = 0
        self.curFrmNo = 0

        self.rate = ros.Rate(120) # Hz
        ros.spin()
        

    def msg_func(self, iLogLevel, szLogMessage):
        szLevel = "None"
        if iLogLevel == 4:
            szLevel = "Debug"
        elif iLogLevel == 3:
            szLevel = "Info"
        elif iLogLevel == 2:
            szLevel = "Warning"
        elif iLogLevel == 1:
            szLevel = "Error"
        print("[%s] %s" % (szLevel, cast(szLogMessage, c_char_p).value))

    def data_func(self, pFrameOfMocapData, pUserData):
        if pFrameOfMocapData == None:  
            print("Not get the data frame.\n")
        else:
            frameData = pFrameOfMocapData.contents
            self.curFrmNo = frameData.iFrame
            if self.curFrmNo == self.preFrmNo:
                return

            self.preFrmNo = self.curFrmNo
            # print( "FrameNo: %d\tTimeStamp:%Ld" % (frameData.iFrame, frameData.iTimeStamp))					
            # print( "nMarkerset = %d" % frameData.nMarkerSets)

        # print(pFrameOfMocapData.contents)
    
        for iBody in range(frameData.nRigidBodies):
            body = frameData.RigidBodies[iBody]
            name = frameData.MocapData[body.ID - 1].szName.decode("utf8")

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            
            pose.pose.position.x = body.x /1000
            pose.pose.position.y = body.y /1000
            pose.pose.position.z = body.z /1000
            pose.pose.orientation.x = body.qx
            pose.pose.orientation.y = body.qy
            pose.pose.orientation.z = body.qz
            pose.pose.orientation.w = body.qw

            self.pub[name].publish(pose)
                


if "__main__" == __name__:
    ros.init_node("nokov_test")
    node = rosnokov("192.168.0.170")
