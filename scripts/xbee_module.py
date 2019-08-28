"""
xbee_module.py contains functions for handling and translating serial strings to
ROS messages & vice versa
"""
import rospy
try:
    from mission_planning.msg import EnemyInfo, SwarmInfo, AgentInfo, InitMessage, TaskMessage
except ImportError:
    try:
        from gnc.msg import AgentInfo, InitMessage, TaskMessage
    except ImportError:
        rospy.logwarn("!!!! MESSAGE FILES NOT IMPORTED !!!!")


################################################################################
#   InitMessage.msg structure is:
# "msg_type|agentId|homeLocation[3]|nominalHeading"
#
#   AgentInfo.msg structure is:
# "msg_type|idAgent|agentPosition[3]|agentHeading|agentBattery|agentPayload|...
#  agentTaskStatus|agentTaskId|agentWorkingStatus|taskDeadline"
#
#   TaskMessage.msg structure is:
# "msg_type|idAgent|taskId|targetId|taskLocation[3]|taskDeadline|timestamp"
#
#   Enemy Info structure is:
# "msg_type|idAgent|confidence|position[3]|timestamp"
################################################################################

''' Convert serial string to binary. Used to pass message to xbee tx function '''
def stringToBinary(string_msg):
    bin_str = bytes(string_msg)
    return bin_str


''' Pythonic implementaion of switch statement to convert ROS msg to serial string '''
def ROSTaskMsgToString(msg):
    stringified = 'T' + '|'
    stringified += str(msg.agentId) + '|'
    stringified += str(msg.taskId) + '|'
    stringified += str(msg.targetId) + '|'
    stringified += str(msg.taskLocation[0]) + '|'
    stringified += str(msg.taskLocation[1]) + '|'
    stringified += str(msg.taskLocation[2]) + '|'
    stringified += str(msg.taskDeadline) + '|'
    stringified += str(msg.timestamp)
    return stringified
def ROSInitMsgToString(msg):
    stringified = 'I' + '|'
    stringified += str(msg.agentId) + '|'
    stringified += str(msg.homeLocation[0]) + '|'
    stringified += str(msg.homeLocation[1]) + '|'
    stringified += str(msg.homeLocation[2]) + '|'
    stringified += str(msg.nominalHeading)
    return stringified
def ROSAgentMsgToString(msg):
    stringified = 'A' + '|'
    stringified += str(msg.agentId) + '|'
    stringified += str(msg.agentPosition[0]) + '|'
    stringified += str(msg.agentPosition[1]) + '|'
    stringified += str(msg.agentPosition[2]) + '|'
    stringified += str(msg.agentHeading) + '|'
    stringified += str(msg.agentBattery) + '|'
    stringified += str(msg.agentPayload) + '|'
    stringified += str(msg.agentTaskStatus) + '|'
    stringified += str(msg.agentTaskId) + '|'
    stringified += str(msg.agentWorkingStatus) + '|'
    stringified += str(msg.taskDeadline)
    return stringified
def ROSEnemyMsgToString(msg):
    stringified = 'E' + '|'
    stringified += str(msg.agentId) + '|'
    stringified += str(msg.confidence) + '|'
    stringified += str(msg.position[0]) + '|'
    stringified += str(msg.position[1]) + '|'
    stringified += str(msg.position[2])
    return stringified
def ROSMsgDefault(msg):
    rospy.loginfo("  :: RECEIVED MESSAGE NOT RECOGNISED: %s ::", str(msg._type))

def msgToString(msg):
    # TODO: Verify max string length is less than max frame length for Digi
    msg_type = str(msg._type).split('/')
    rospy.loginfo("  :: SENDING MESSAGE TYPE - %s ::", msg_type)

    switcher = {
        "TaskMessage": ROSTaskMsgToString,
        "InitMessage": ROSInitMsgToString,
        "AgentInfo": ROSAgentMsgToString,
        "EnemyInfo": ROSEnemyMsgToString,
    }

    # msg_type[-1] will return the last element of a split string similar to this:
    # ".../mission_planning/TaskMessage"
    func = switcher.get(msg_type[-1], lambda:ROSMsgDefault(msg))
    return func(msg)


''' Pythonic implementaion of switch statement to convert serial string to ROS msg '''
def StringToROSTaskMsg(string):
    msg = TaskMessage()
    msgStr = string.split('|')  # Split string into array of strings
    msg.agentId = int(msgStr[1])
    msg.taskId = msgStr[2]
    msg.targetId = msgStr[3]
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
    str2bool = lambda x: True if x=='True' else False

    msgStr = string.split('|')
    msg.agentId = int(msgStr[1])
    msg.agentPosition[0] = float(msgStr[2])
    msg.agentPosition[1] = float(msgStr[3])
    msg.agentPosition[2] = float(msgStr[4])
    msg.agentHeading = float(msgStr[5])
    msg.agentBattery = float(msgStr[6])
    msg.agentPayload = str2bool(msgStr[7])
    #if msgStr[7] == 'False':
    #    msg.agentPayload = False
    #elif msgStr[7] == 'True':
    #    msg.agentPayload = True
    msg.agentTaskStatus = str2bool(msgStr[8])
    #if msgStr[8] == 'False':
    #    msg.agentTaskStatus = False
    #elif msgStr[8] == 'True':
    #    msg.agentTaskStatus = True
    msg.agentTaskId = int(msgStr[9])
    msg.agentWorkingStatus = str2bool(msgStr[10])
    #if msgStr[10] == 'False':
    #    msg.agentWorkingStatus = False
    #elif msgStr[10] == 'True':
    #    msg.agentWorkingStatus = True
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
    rospy.loginfo("  :: RECEIVED MESSAGE NOT RECOGNISED - %s ::", string[0])

def stringToMsg(string,msg_orig):
    str_type = string[0]
    rospy.loginfo(" :: RECEIVED MESSAGE TYPE - %s ::", str_type)
    switcher = {
        "T": StringToROSTaskMsg,
        "I": StringToROSInitMsg,
        "A": StringToROSAgentMsg,
        "E": StringToROSEnemyMsg,
    }
    # See comment at top to module for string formats
    func = switcher.get(str_type, lambda:StringDefault(string))

    return str_type, func(string)
