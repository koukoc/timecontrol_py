import rospy
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class Autopilot:
    
    rollRate = 0
    ecef=0
    autopilotState = False
    def state_cb(self,msg):
        self.current_state = msg
    def __init__(self):
        state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.Subscriber('RollRate',Bool,self.__rollRateCallback)
        rospy.Subscriber('ecef',Bool,self.__ecefCallback)

    def updateAutopilotState(self):
        # Wait for Flight Controller connection
        if(not rospy.is_shutdown() and self.current_state.connected):
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = 'OFFBOARD'
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True
            self.set_mode_client.call(offb_set_mode)
            self.arming_client.call(arm_cmd)
            rospy.loginfo("OFFBOARD enabled")
            rospy.loginfo("Vehicle armed")
            self.autopilotState = True
        else:
            self.autopilotState = False
        return self.autopilotState

    
    def __rollRateCallback(self,data):
        self.rollRate = data.data
    
    def __ecefCallback(self,data):
        self.ecef = data.data

    def checkDespin(self):
        if abs(self.rollRate) < 10:
            return True
        else:
            return False
    
    def startRollCtrl(self):
        # 將除了roll以外的PID參數調為0，進行減滾
        a=0

    def startAttitudeCtrl(self):
        self.ecef
        # 利用現在的座標與目標座標相減求姿態

