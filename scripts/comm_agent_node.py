#!/usr/bin/env python

import rospy

from comm_module1 import CommNode
from xbee import DigiMesh           # https://github.com/niolabs/python-xbee

def comm_agent_node():
    rospy.init_node('comm_agent_node', anonymous=True)
    rospy.loginfo(":::: COMMUNICATION GROUND NODE INITIALIZATION ::::")

    com = CommNode('AGENT')
    #com = CommNode('GND')

    rospy.spin()

if __name__ == '__main__':
    try:
        comm_agent_node()
    except rospy.ROSInterruptException:
        pass
