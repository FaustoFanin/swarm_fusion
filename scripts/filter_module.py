"""

"""
import rospy
import numpy as np
from scipy.stats import multivariate_normal
from gdp_int.msg import Detection

class FilterManager:
    def __init__(self,_timestep):
        self.filters = []
        self.timestep = _timestep
        self.friendly_detectionIds = []
    def update(self):
        for filt in self.filters:
            filt.propagate()

        ######################
        # FILTER KILLER CODE #
        #new_filts = []
        #for filt in self.filters:
        #    est = filt.getEstimate()
        #    if est[3] > 0.5:
        #        new_filts.append(filt)
        #self.filters = new_filts

    def newFilt(self,id,pos):
        self.filters.append(Filter(id,self.timestep,pos))
        self.filters.sort(key = lambda filt: filt.filterId)

    def getEstimates(self):
        estimates = []
        for filt in self.filters:
            estimates.append(filt.getEstimate())
        return estimates

    def allocateDetection(self, data, swarm):
        pos = data.position

        if data.detectionId in self.friendly_detectionIds:
            # Detection ID assigned to a friendly, ignore
            return

        if data.detectionId in [filt.filterId for filt in self.filters]:
                self.filters[filt.filterId-1].store_measurement(pos)
        else:
            prob_friend = []
            prob_enemy = []

            # Create probs of being a friend
            for frd_idx, friend in enumerate(swarm.friendlies):
                prob_friend.append(self.detectProb(pos, friend.agentPosition, True))
            # Create probs of being an enemy
            for enemy in self.filters:
                prob_enemy.append(self.detectProb(pos, enemy.pos, False, enemy.pos_cov))

            # Find max probabilities
            if len(prob_friend):
                friend_idx, friend_prob = max(enumerate(prob_friend))
            else:
                # No friendly info received, ignore until we get some Info
                # FRIENDLY info more import than having detections
                return


            if len(prob_enemy):
                enemy_idx, enemy_prob = max(enumerate(prob_enemy))
            else:
                # No enemy trackers so far. Assign detection as potential first detection
                enemy_idx = 1
                enemy_prob = 0.0


            #rospy.loginfo(" F: %f",friend_prob)
            #rospy.loginfo(" E: %f",enemy_prob)
            #print prob_friend
            #print prob_enemy

            # Friendly threshold is based on a detection [2,2,2] away from mean
            # Enemy threshold is based on a detection [3,3,3] away from mean
            if (friend_prob < 0.0001) and (enemy_prob < 0.0001):
                # If both probabilities are too small then this is likely a new drone (hopefully...)
                self.newFilt(data.detectionId, data.position)
                #rospy.loginfo("   : DETECTION REJECTED :")
            elif enemy_prob > friend_prob:
                # Probably an enemy, assign enemy Id instead of descriptor
                #rospy.loginfo("   : ENEMY DETECTED :")
                self.filters[enemy_idx].store_measurement(pos)
            else:   # friend_prob > enemy_prob
                self.friendly_detectionIds.append(data.detectionId)
                #rospy.loginfo("   : FRIENDLY DETECTED :")

    def detectProb(self, pos, GPS, isFriend, cov=2):
        # Return multivariate Gaussion pdf of
        if isFriend:
            covariance = np.array([[cov,0,0],[0,cov,0],[0,0,cov]])
        else:
            covariance = cov

        return multivariate_normal.pdf(pos,mean=GPS,cov=covariance)


class Filter:
    def __init__(self,id,timestep,pos_ini):
        '''
        Current implementaion of filter is a LKF in local coordiantes.
        Possible room for improvement to modify to EKF in detection coordinates
        with final conversion to local.
        '''
        self.filterId = id

        self.pos = pos_ini
        self.vel = np.array([0,0,0])

        self.x_k = np.array([self.pos[0],self.vel[0],
                             self.pos[1],self.vel[1],
                             self.pos[2],self.vel[2]])
        self.P_ini = 5
        self.P_k = np.diag([3,6,3,6,3,6])

        dt = timestep
        self.F_k = np.array([[1,dt,0, 0,0, 0],
                             [0, 1,0, 0,0, 0],
                             [0, 0,1,dt,0, 0],
                             [0, 0,0, 1,0, 0],
                             [0, 0,0, 0,1,dt],
                             [0, 0,0, 0,0, 1]])
        self.H_k = np.array([[1,0,0,0,0,0],
                             [0,0,1,0,0,0],
                             [0,0,0,0,1,0]])
        #self.Q_k = np.diag([0.005, 0.0001, 0.005, 0.0001, 0.005, 0.0001])
        self.Q_k = np.diag([1*dt, 2*dt*dt, 1*dt, 2*dt*dt, 2*dt, 2*dt*dt])
        self.pos_cov = np.diag([np.diag(self.P_k)[0], np.diag(self.P_k)[2], np.diag(self.P_k)[4]])
        #q = 5
        #self.Q_k = np.array([[q,0,0],
        #                     [0,q,0],
        #                     [0,0,q]])
        self.R_k = np.diag([2,2,2])
        #r = 2
        #self.R_k = np.array([[r,0,0],
        #                     [0,r,0],
        #                     [0,0,r]])
        self.measurement_available = False
        self.measurement = np.array([0,0,0])

    def store_measurement(self,measure):
        self.measurement = measure
        self.measurement_available = True

    def propagate(self):
        x_k_p = self.F_k.dot(self.x_k)
        P_k_p = np.matmul(self.F_k,np.matmul(self.P_k,np.transpose(self.F_k))) + self.Q_k

        if self.measurement_available:
            try:
                innov = self.measurement - self.H_k.dot(x_k_p)
                S_k = self.R_k + np.matmul(np.matmul(self.H_k,P_k_p),np.transpose(self.H_k))
                K_k = np.matmul(np.matmul(P_k_p,np.transpose(self.H_k)),np.linalg.inv(S_k))
                self.x_k = x_k_p + K_k.dot(innov)
                self.P_k = (np.eye(np.shape(K_k.dot(self.H_k))[0]) - K_k.dot(self.H_k)).dot(P_k_p)
            except:
                self.x_k = x_k_p
                self.P_k = P_k_p
                self.pos = np.array([self.x_k[0],self.x_k[2],self.x_k[4]])
                self.vel = np.array([self.x_k[1],self.x_k[3],self.x_k[5]])
                self.pos_cov = np.array([[self.P_k[0][0],self.P_k[0][2],self.P_k[0][4]],
                                         [self.P_k[2][0],self.P_k[2][2],self.P_k[2][4]],
                                         [self.P_k[4][0],self.P_k[4][2],self.P_k[4][4]]])
                rospy.logwarn("   : KALMAN CALCULATION FAILED :")
                return
            self.measurement_available = False
        else:
            self.x_k = x_k_p
            self.P_k = P_k_p
            self.pos = np.array([self.x_k[0],self.x_k[2],self.x_k[4]])
            self.vel = np.array([self.x_k[1],self.x_k[3],self.x_k[5]])
            self.pos_cov = np.array([[self.P_k[0][0],self.P_k[0][2],self.P_k[0][4]],
                                     [self.P_k[2][0],self.P_k[2][2],self.P_k[2][4]],
                                     [self.P_k[4][0],self.P_k[4][2],self.P_k[4][4]]])

    def getEstimate(self):
        try:
            conf = np.linalg.norm(np.linalg.inv(self.pos_cov))
            #conf /= 2.5
            conf = (3.0/2.5)*(conf/(0.5+conf))
        except:
            rospy.logwarn("   : CONFIDENCE INCALCULABLE :")
            #covariance = np.mean([self.P_k[0][0],self.P_k[2][2],self.P_k[4][4]])
            #conf = 1.0/covariance

        return [self.filterId,self.pos, self.vel, min(conf,1)]
