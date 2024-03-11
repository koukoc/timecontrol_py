import rospy
from std_msgs.msg import Bool


class Autopilot:
    
    rollRate = 0
    def __init__(self):
        rospy.Subscriber('RollRate',Bool,self.__rollRateCallback)
        rospy.Subscriber('ecef',Bool,self.__ecefCallback)

    
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
        setpara()

    def startAttitudeCtrl(self):
        self.ecef
        # 利用現在的座標與目標座標相減求姿態

