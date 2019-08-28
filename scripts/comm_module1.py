"""
comm_module.py contains support functions for the communication nodes, both on
the agents and on the ground station. For consistency all frame conversions are
done on the ground station
"""
import time
import rospy
import serial
from frame_module import FrameConvertor
#from xbee_module import stringToMsg, msgToString, stringToBinary
from xbee_module import msgToString, stringToBinary
from xbee import DigiMesh

try:
    from mission_planning.msg import EnemyInfo, SwarmInfo, AgentInfo, InitMessage, TaskMessage
except ImportError:
    try:
        from gnc.msg import EnemyInfo, SwarmInfo, AgentInfo, InitMessage, TaskMessage
    except ImportError:
        rospy.logwarn("!!!! MESSAGE FILES NOT IMPORTED !!!!")


class CommNode:
    def __init__(self, location):
        self.serial_port = '/dev/ttyUSB1'
        self.baud_rate = 57600
        self.broadcast_addr = b'\x00\x00\x00\x00\x00\x00\xff\xff'

        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        time.sleep(1)

        try:
            self.xbee_device = DigiMesh(self.ser, callback = self.xbeeCallback)
        except:
            rospy.logwarn("!!!! ERROR - DigiMesh OBJECT NOT INITIALIZED !!!!")

        if location == 'AGENT':
            # Comm node onboard agent
            rospy.loginfo(" ::: NODE INITIALIZED AS AGENT COMM NODE ::: ")

            rospy.Subscriber("AgentInformation", AgentInfo, self.agentCallback)
            self.pubTasks = rospy.Publisher("AgentTasks",TaskMessage,queue_size=10)
            self.pubInit = rospy.Publisher("AgentInit",InitMessage,queue_size=10)
        elif location == 'GND':
            # Comm node on ground station
            rospy.loginfo(" ::: NODE INITIALIZED AS GCS COMM NODE ::: ")

            rospy.Subscriber("TaskActions", TaskMessage, self.tasksCallback)
            rospy.Subscriber("InitInformation", InitMessage, self.initCallback)
            self.pubEnemy = rospy.Publisher("EnemyInfo",EnemyInfo,queue_size=10)
            self.pubAgent = rospy.Publisher("AgentInfo",AgentInfo,queue_size=10)
        else:
            rospy.logwarn("!!!! ERROR - COMM NODE TYPE NOT RECOGNISED - %s !!!!", location)

        self.fConv = FrameConvertor()

    ''' Python implementation of switch statement to publish to ROS topics '''
    # These functions are shared for all comm nodes, only the relevant ones will be used
    def xbeeCallback(self,data):
        '''def pubInitMsg(self,msg):
            self.pubInit(msg)
        def pubTaskMsg(self,msg):
            self.pubTasks(msg)
        def pubAgentMsg(self,msg):
            # convert frame
            self.pubAgent(msg)
        def pubEnemyMsg(self,msg):
            # convert frame
            self.pubEnemy(msg)
        '''

        def StringToROSTaskMsg(string):
            msg = TaskMessage()
            msgStr = string.split('|')  # Split string into array of strings
            msg.agentId = int(msgStr[1])
            msg.taskId = int(msgStr[2])
            msg.targetId = int(msgStr[3])
            msg.taskLocation[0] = float(msgStr[4])
            msg.taskLocation[1] = float(msgStr[5])
            msg.taskLocation[2] = float(msgStr[6])
            msg.taskDeadline = float(msgStr[7])
            msg.timestamp = float(msgStr[8])
            return msg
        def StringToROSInitMsg(string):
            msg = InitMessage()
            msgStr = string.split('|')
            msg.agentId = int(msgStr[1])
            msg.homeLocation[0] = float(msgStr[2])
            msg.homeLocation[1] = float(msgStr[3])
            msg.homeLocation[2] = float(msgStr[4])
            msg.nominalHeading = float(msgStr[5])
            return msg
        def StringToROSAgentMsg(string):
            msg = AgentInfo()
            msgStr = string.split('|')
            msg.agentId = int(msgStr[1])
            msg.agentPosition[0] = float(msgStr[2])
            msg.agentPosition[1] = float(msgStr[3])
            msg.agentPosition[2] = float(msgStr[4])
            msg.agentHeading = float(msgStr[5])
            msg.agentBattery = float(msgStr[6])
            if msgStr[7] == 'False':
                msg.agentPayload = False
            elif msgStr[7] == 'True':
                msg.agentPayload = True
            if msgStr[8] == 'False':
                msg.agentTaskStatus = False
            elif msgStr[8] == 'True':
                msg.agentTaskStatus = True
            msg.agentTaskId = int(msgStr[9])
            if msgStr[10] == 'False':
                msg.agentWorkingStatus = False
            elif msgStr[10] == 'True':
                msg.agentWorkingStatus = True
            msg.taskDeadline = float(msgStr[11])
            return msg
        def StringToROSEnemyMsg(string):
            msg = EnemyInfo()
            msgStr = string.split('|')
            msg.agentId = int(msgStr[1])
            msg.confidence = float(msgStr[2])
            msg.agentPosition[0] = float(msgStr[3])
            msg.agentPosition[1] = float(msgStr[4])
            msg.agentPosition[2] = float(msgStr[5])
            return msg
        def StringDefault(string):
            rospy.loginfo("Received message type not known: %s", string[0])
        string = data.get('data')
        #def stringToMsg(string,msg_orig):
        str_type = string[0]
        rospy.loginfo(" :: RECEIVED MESSAGE TYPE - %s ::", str_type)
        switcher1 = {
            "T": StringToROSTaskMsg,
            "I": StringToROSInitMsg,
            "A": StringToROSAgentMsg,
            "E": StringToROSEnemyMsg,
        }
        # See comment at top to module for string formats
        func = switcher1.get(str_type, lambda:StringDefault(string))

        #    return str_type, func(string)
        #msg_type, msg = stringToMsg(data.get('data'))
        msg_type = str_type
        msg = func(string)
        #msg_orig = None
        #stringToMsg(data.get('data'))
        #print msg_type
        #print msg_orig

        rospy.loginfo("  :: RECEIVING STRING - %s ::", msg_type)
        #print 'cucu1'
        #print msg
        if msg_type == 'A':
            print 'pub agents'
            self.pubAgent.publish(msg)
        elif msg_type == 'E':
            print 'pub enemy'
            self.pubEnemy.publish(msg)
        elif msg_type == 'I':
            print 'pub init'
            self.pubInit.publish(msg)
        elif msg_type == 'T':
            print 'pub tasks'
            self.pubTasks.publish(msg)
        else:
            rospy.loginfo("   : IGNORING MESSAGE :")

        '''
        def pubInitMsg(self,msg):
            self.pubInit.publish(msg)
        def pubTaskMsg(self,msg):
            self.pubTasks.publish(msg)
        def pubAgentMsg(self,msg):
            # convert frame
            self.pubAgent.publish(msg)
        def pubEnemyMsg(self,msg):
            # convert frame
            self.pubEnemy.publish(msg)



        switcher = {
            "T": pubTaskMsg,
            "I": pubInitMsg,
            "A": pubAgentMsg,
            "E": pubEnemyMsg,
        }

        print 'cucu'
        if msg_type not in switcher:
            print "   : IGNORING MESSAGE :"
        else:
            print "hell2"
            func = switcher.get(msg_type)
            #print msg
            func(msg)
            print msg'''

    def txString(self,str_msg):
        rospy.loginfo("  :: TRANSMITTING STRING ::")
        bin_str = stringToBinary(str_msg)
        try:
            self.xbee_device.tx(frame_id=b'\x10', dest_addr=self.broadcast_addr, data=bin_str)
        except:
            rospy.logwarn("!!!! ERROR - FAILURE TO TRANSMIT STRING !!!!")


    ''' Agent Comm functions '''
    def agentCallback(self,data):
        rospy.loginfo("Recevived agent msg")
        str_msg = msgToString(data)
        self.txString(str_msg)

    ''' GCS Comm functions '''
    def initCallback(self,data):
        ''' Local -> World Frame'''
        rospy.loginfo("  :: SENDING INITIALIZATION MESSAGE :: ")
        initInfo = data
        worldPos, worldHead = self.fConv.localToWorldFrame(data.homeLocation,data.nominalHeading)
        initInfo.homeLocation = worldPos
        initInfo.nominalHeading = worldHead

        # EDIT: send through xbee instead of publishing
        str_msg = msgToString(initInfo)
        self.txString(str_msg)
        #self.pubInit.publish(initInfo)


    def tasksCallback(self,data):
        ''' Local -> World Frame '''
        rospy.loginfo("  :: SENDING TASK MESSAGE :: ")
        taskInfo = data
        worldPos, worldHead = self.fConv.localToWorldFrame(data.taskLocation,0)
        taskInfo.taskLocation = worldPos

        # EDIT: send through xbee instead of publishing
        str_msg = msgToString(taskInfo)
        self.txString(str_msg)
        #self.pubTasks.publish(taskInfo)

    def enemyCallback(self,data):
        #str_msg = msgToString(data)
        #self.txString(str_msg)
        self.pubEnemy.publish(enemyInfo)
