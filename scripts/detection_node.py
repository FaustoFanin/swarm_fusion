#!/usr/bin/env python

import os
import rospy
import time
from mission_planning.msg import Detection, AgentInfo
import numpy as np

def getRelativeFilePath(relativePath):
    scriptDir = os.path.dirname(__file__)
    absFilePath = os.path.join(scriptDir, relativePath)
    return absFilePath

def detection():
    msg = Detection()
    msg.detectionId = 1
    msg.position = [4,0,0]
    return msg

def detection_node():
    rospy.init_node('detection_node',anonymous=True)

    pubDetect = rospy.Publisher("DetectionInfo", Detection, queue_size=10)
    pubFriend = rospy.Publisher("AgentInfo", AgentInfo, queue_size=10)
    rate = rospy.Rate(10)

    agent = AgentInfo()
    # Init position is [0,0,0] in local
    agent.agentPosition = [52.067487,-0.623818,0]

    detectMsg1 = Detection()
    detectMsg1.detectionId = 1
    detectMsg1.position = [1,0,-15]
    vel = 1.0/10

    detectMsg2 = Detection()
    detectMsg2.detectionId = 2
    detectMsg2.position = [0,0,0]

    f1 = open(getRelativeFilePath("Test_data_field.txt"))
    lines1 = f1.readlines()
    f1.close()

    while not rospy.is_shutdown():
        detectMsg1.position[2] = detectMsg1.position[2] + vel
        #if np.abs(detectMsg1.position[2]) < 5:
        #    detectMsg1.detectionId += 2
        #    detectMsg2.detectionId += 2

        detectMsg2.position = np.random.multivariate_normal([0,0,0], np.diag([1,1,1]))
        pubDetect.publish(detectMsg1)
        pubDetect.publish(detectMsg2)

        #agent.agentPosition = np.random.multivariate_normal([52.067487,-0.623818,0], np.diag([0.0000000001,0.0000000001,0.0000000001]))
        pubFriend.publish(agent)

        rate.sleep()

if __name__ == '__main__':
    try:
        detection_node()
    except rospy.ROSInterruptException:
        pass
