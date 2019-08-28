#!/usr/bin/env python

import rospy

from fusion_module import FusionManager
from mission_planning.msg import AgentInfo, SwarmInfo, EnemyInfo


def fusion_node():
    rospy.init_node('fusion_node',anonymous=True)
    rospy.loginfo(":::: FU510N N0D3 1N1T14L1Z4T10N :::: ")

    fusion_hz = 20
    fuse_comm = FusionManager(fusion_hz)
    rate = rospy.Rate(fusion_hz)

    while not rospy.is_shutdown():
        if False:
            fuse_comm.publishMsg()
        else:
            fuse_comm.fusion_update()
        rate.sleep()

if __name__ == '__main__':
    try:
        fusion_node()
    except rospy.ROSInterruptException:
        pass
