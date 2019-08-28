#!/usr/bin/env python

import rospy

#from fusion_module import FusionManager
from mission_planning.msg import AgentInfo, SwarmInfo, EnemyInfo

def callback(data):
    print "Received agentinfo!"
    print data


def test():
    rospy.init_node('test',anonymous=True)
    #print(":::: FU510N N0D3 1N1T14L1Z4T10N :::: ")

    #fuse_comm = FusionManager()
    pub = rospy.Publisher("AgentInformation", AgentInfo, queue_size=10)
    rospy.Subscriber("AgentInfo",AgentInfo,callback)
    rate = rospy.Rate(1)
    msg = AgentInfo()
    while not rospy.is_shutdown():
        #fuse_comm.publishMsg()
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
