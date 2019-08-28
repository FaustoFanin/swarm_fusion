"""
frame_module.py contains functions to convert between local battlefield
co-coordinates and GPS co-coordinates
"""

import os
import time
import rospy
import numpy as np
from pyproj import Proj
from mission_planning.msg import EnemyInfo

class FrameConvertor:
    def __init__(self):
        # Read battlefield waypoints from file
        f = open(self.getRelativeFilePath("Battlefield.txt"))
        lines = f.readlines()
        f.close()

        x0y0_GPS = np.array([float(i) for i in lines[0].split(',')])
        x100y0_GPS = np.array([float(i) for i in lines[1].split(',')])
        detection_GPS = np.array([float(i) for i in lines[2].split(',')])
        #x100y100_GPS = np.array([float(i) for i in lines[3].split(',')])

        self.world = Proj(proj='tmerc', lon_0=x0y0_GPS[1], lat_0=x0y0_GPS[0], ellps='airy')
        #self.x_off,self.y_off = self.world(0,0,inverse=True)

        #x0y0_lcl = self.world(x0y0_GPS[1],x0y0_GPS[0])
        x100y0_lcl = list(self.world(x100y0_GPS[1],x100y0_GPS[0]))
        #x0y100_lcl = self.world(x0y100_GPS[1],x0y100_GPS[0])
        #x100y100_lcl = self.world(x100y100_GPS[1],x100y100_GPS[0])

        self.tilt = np.arctan2(x100y0_lcl[1],x100y0_lcl[0]) - np.pi/2
        self.battle_rot = np.array([[np.cos(self.tilt),-np.sin(self.tilt)],
                                    [np.sin(self.tilt), np.cos(self.tilt)]])
        self.detection_local = list(self.world(detection_GPS[1], detection_GPS[0]))
        self.det_head = np.radians(float(lines[3])) - self.tilt
        self.det_elev = np.radians(float(lines[4]))
        self.detect_T = np.array([[np.cos(self.det_head),-np.sin(self.det_head),0,self.detection_local[0]],
                                  [np.cos(self.det_elev)*np.sin(self.det_head),np.cos(self.det_elev)*np.cos(self.det_head),-np.sin(self.det_elev),self.detection_local[1]],
                                  [np.sin(self.det_elev)*np.cos(self.det_head),np.sin(self.det_elev)*np.cos(self.det_head),np.cos(self.det_elev),detection_GPS[2]],
                                  [0,0,0,1]])
        rospy.loginfo("  :: BATTLEFIELD INITIALISED - TILT %.1f", np.degrees(self.tilt))
        rospy.loginfo("   : SITUATIONAL AWARENESS - %.1f, %.1f", self.detection_local[0], self.detection_local[1])


    def localToWorldFrame(self,pos,orient):
        ''' To convert from the local battlefield coordinates to GPS'''
        pos_temp = self.battle_rot.dot(pos[0:2])
        lon, lat = self.world(pos_temp[0],pos_temp[1],inverse=True)   # For AIRY ellipse

        worldPos = [lat, lon, pos[2]]
        worldHead = orient + self.tilt

        return worldPos,worldHead


    def worldToLocalFrame(self,gps,orient):
        ''' To convert from GPS to the local battlefield coordinates'''
        pos_temp = list(self.world(gps[1], gps[0]))    # For AIRY ellipse
        pos = np.transpose(self.battle_rot).dot(pos_temp)

        localPos = [pos[0], pos[1], gps[2]]
        localHead = orient - self.tilt

        return localPos,localHead


    def detections(self):
        ''' Function to spoof detection of an enemy. '''
        enemy1 = EnemyInfo()
        enemy2 = EnemyInfo()

        enemy1.agentId = 1
        enemy2.agentId = 2

        ################################################
        ## Change the following parameters if desired ##
        ################################################
        # Enemy 1 goes around in a circle
        rad1 = 2.0            # Radius of enemy 1 path
        freq1 = 1.0/rad1        # Rot speed of enemy 1, ground speed = 1.0m/s
        xc1,yc1 = [5.0,5.0]   # Center of enemy 1 path
        min_conf1, max_conf1 = [0.5,1]  # Min & max confidences of detection
        # Enemy 2 goes forwards and backwards along a lines
        amp2 = 4.0            # Y-Amplitude of enemy 2 path
        freq2 = 1.0/amp2        # Freq of enemy 2, ground speed = 1.0m/s
        xc2,yc2 = [10.0,10.0]   # Center of enemy 2 path
        min_conf2, max_conf2 = [0.65,0.95]  # Min & max confidences of detection
        phase2 = 0.0            # Phase relative to enemy 1 in radians

        time_val = time.time()
        x1,y1 = [rad1 * np.cos(freq1 * time_val), rad1 * np.sin(freq1 * time_val)]
        y2 = amp2 * np.sin(freq2 * time_val + phase2)

        # Change the constants to vary altitude
        enemy1.agentPosition = [xc1 + x1, yc1 + y1, 5]
        enemy2.agentPosition = [xc2, yc2 + y2, 5]

        enemy1.agentVelocity = [-freq1 * rad1 * np.sin(freq1 * time_val), freq1 * rad1 * np.cos(freq1 * time_val), 0]
        enemy2.agentVelocity = [0, freq2 * amp2 * np.cos(freq2 * time_val + phase2), 0]

        enemy1.confidence = min_conf1 + (min_conf1-max_conf1)*(0.5*(y1/rad1-1))
        enemy2.confidence = min_conf2 + (min_conf2-max_conf2)*(0.5*(y2/amp2-1))

        return [enemy1, enemy2]

    def detectionToLocalFrame(self,pos):
        localPos = self.detect_T.dot([pos[0],pos[2],-pos[1],1])
        return localPos[0:3]

    def getRelativeFilePath(self, relativePath):
        scriptDir = os.path.dirname(__file__)
        absFilePath = os.path.join(scriptDir, relativePath)
        return absFilePath
