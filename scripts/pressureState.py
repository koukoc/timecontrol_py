#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16

class PressureState:
    FirstEnginePressure = 0
    FirstTankPressure = 0

    SecondEnginePressure = 0
    SecondTankPressure = 0
    
    def __FirstEnginePressureSub(self,data):
        self.FirstEnginePressure = data.data

    def __FirstTankPressureSub(self,data):
        self.FirstTankPressure = data.data

    def __SecondEnginePressureSub(self,data):
        self.SecondEnginePressure = data.data

    def __SecondTankPressureSub(self,data):
        self.SecondTankPressure = data.data

    def __RCSTankPressureSub(self,data):
        self.RCSTankPressure = data.data


    
    def __init__(self):
        # fisrt stage pressure transducer subscriber
        self.FirstEnginePressureSub = rospy.Subscriber("FirstEnginePressure",UInt16,self.__FirstEnginePressureSub)
        self.FirstTankPressureSub = rospy.Subscriber("FirstTankPressure",UInt16,self.__FirstTankPressureSub)

        # second stage pressure transducer subscriber
        self.SecondEnginePressureSub = rospy.Subscriber("SecondEnginePressure",UInt16,self.__SecondEnginePressureSub)
        self.SecondTankPressureSub = rospy.Subscriber("SecondTankPressure",UInt16,self.__SecondTankPressureSub)

        # RCS Pressure transducer subscriber
        self.RCSTankPressureSub1 = rospy.Subscriber("RCSTankPressure",UInt16,self.__RCSTankPressureSub)

    def unscribeFirstStage(self):
        self.FirstEnginePressureSub.unregister()
        self.FirstTankPressureSub.unregister()


    def unscribeSecondStage(self):
        self.FirstEnginePressureSub.unregister()
        self.SecondTankPressureSub.unregister()

    def unsubscribeRCStank(self):
        self.RCSTankPressureSub1.unregister()