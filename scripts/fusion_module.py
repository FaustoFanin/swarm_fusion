"""
fusion_module.py contains functions for that sensor fusion magic
"""
import rospy
import numpy as np
from filter_module import FilterManager
from frame_module import FrameConvertor
from gdp_int.msg import Detection, SwarmInfo, AgentInfo, EnemyInfo

class FusionManager:
    def __init__(self,fuse_hz):
        rospy.Subscriber("DetectionInfo", Detection, self.detectionsCallback)
        rospy.Subscriber("AgentInfo", AgentInfo, self.agentCallback)

        self.pubSwarm = rospy.Publisher("SwarmInformation", SwarmInfo, queue_size=10)

        self.dt = 1.0/fuse_hz
        self.swarm = SwarmInfo()

        self.fConv = FrameConvertor()

        self.filtMan = FilterManager(self.dt)
        self.detections_received = 0
        self.tick = 0

    def fusion_update(self):
        self.filtMan.update()

        # Publish at 1Hz for clarity
        if self.tick >= 1/self.dt:
            self.publishMsg()
            self.tick = 0
        else:
            self.tick += 1

    def publishMsg(self):
        # Spoof detection of 2 enemies
        if False:    # Spoofing enemies
            self.swarm.enemies = self.fConv.detections()
            rospy.loginfo("   : PUBLISH FUSED INFO :")
            self.pubSwarm.publish(self.swarm)
        else:       # Spoofing detections
            ########################################
            # New fusion to integrate
            enemy_estimates = self.filtMan.getEstimates()
            enemies = []
            for eIdx, enemy_est in enumerate(enemy_estimates):
                enemy = EnemyInfo()
                enemy.agentId = enemy_est[0]
                enemy.agentPosition = enemy_est[1]
                enemy.agentVelocity = enemy_est[2]
                enemy.confidence = enemy_est[3]
                #if eIdx < len(self.swarm.enemies):
                #    self.swarm.enemies[eIdx] = enemy
                #else:
                #    self.swarm.enemies.append(enemy)
                enemies.append(enemy)
            self.swarm.enemies = enemies
            ###########################################
            self.pubSwarm.publish(self.swarm)

        rospy.loginfo("   : RECEIVED %d DETECTIONS", self.detections_received)
        self.detections_received = 0
        if not len(self.swarm.friendlies):
            rospy.logwarn("   ! NO FRIENDLY INFO RECEIVED !")
        else:
            rospy.loginfo("   : TRACKING %d FRIENDLIES", len(self.swarm.friendlies))
        rospy.loginfo("   : TRACKING %d ENEMIES", len(self.swarm.enemies))
        print "------------------------------------------------------"


    def detectionsCallback(self,data):
        #rospy.loginfo("  :: DETECTION RECEIVED ::")
        self.detections_received += 1
        det_data = data
        # Convert to local frame
        localPos = self.fConv.detectionToLocalFrame(data.position)
        det_data.position = localPos
        # Allocate Detections

        self.filtMan.allocateDetection(det_data,self.swarm)


    def agentCallback(self,data):
        ''' Receive AgentInfo from GNC and package into SwarmInfo. '''
        #rospy.loginfo("  :: AGENT %d INFO RECEIVED ::", data.agentId)

        # Make a copy of data and transform pos and heading to local frame
        gncInfo = data
        localPos, localHead = self.fConv.worldToLocalFrame(data.agentPosition,data.agentHeading)
        gncInfo.agentPosition = localPos
        gncInfo.agentHeading = localHead

        ## ADD NOISE FOR TESTING!!
        gncInfo.agentPosition = gncInfo.agentPosition + np.random.multivariate_normal([0,0,0],np.diag([1,1,1]))

        if len(self.swarm.friendlies) == 0:
            self.swarm.friendlies.append(gncInfo)
        else:
            for fIdx, friendly in enumerate(self.swarm.friendlies,0):
                # Then package into SwarmInfo message
                if gncInfo.agentId == friendly.agentId:
                    self.swarm.friendlies[fIdx] = gncInfo
                    break
                elif fIdx == len(self.swarm.friendlies)-1:
                    self.swarm.friendlies.append(gncInfo)
                    break
