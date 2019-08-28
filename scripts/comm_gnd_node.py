#!/usr/bin/env python

import rospy
import serial
from comm_module import CommNode
from xbee import DigiMesh           # https://github.com/niolabs/python-xbee

def comm_gnd_node():
    rospy.init_node('comm_gnd_node', anonymous=True)
    rospy.loginfo(":::: COMMUNICATION GROUND NODE INITIALIZATION ::::")

    #com = CommNode('AGENT')
    com = CommNode('GND')

    rospy.spin()

if __name__ == '__main__':
    try:
        comm_gnd_node()
    except rospy.ROSInterruptException:
        pass
