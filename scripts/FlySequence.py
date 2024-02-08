#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
OPEN = True
CLOSE = False



class FlightSequence:
    safetySwitch = False
    MissionStartTime = 0
    FirstStageIgnition = False
    SecondStageIgnition = False
    SeparationState = False
    SeparationChecked = True
    MissionPause = False
    RocketSOH = False


    def __firstStageIgnitionCallback(self,data):
        self.FirstStageIgnition = data.data

    def __secondStageIgnitionCallback(self,data):
        self.SecondStageIgnition = data.data

    def __SeparationStateCallback(self,data):
        self.SeparationState = data.data
    
    def __checkRocketSOH(self,event):
        self.RocketSOH = True
        return

    def __init__(self):
        # safetySwitchCallback
        rospy.Subscriber('FirstStageIgnition',Bool,self.__firstStageIgnitionCallback)
        rospy.Subscriber('SecondStageIgnition',Bool,self.__secondStageIgnitionCallback)
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)

        # separation callback
        self.MissionStartTime = rospy.get_time()
        return


    def __setFirstStageIgnite(self):
        print('1st Stage Ignited at',rospy.get_time())
        return
    
    def __setSecondStageIgnite(self):
        print('2nd Stage Ignited at',rospy.get_time())
        return
    
    def __setFirstStageMainValve(self,command):
        if command:
            print('1st Main Valve Opened at',rospy.get_time())
        else:
            print('1st Main Valve Closed at',rospy.get_time())
        return
    
    def __setSecondStageMainValve(self,command):
        if command:
            print('2nd Main Valve Opened at',rospy.get_time())
        else:
            print('2nd Main Valve Closed at',rospy.get_time())
        return
    
    def __closeSecondStageMainValve(self,event):
        self.__setSecondStageMainValve(CLOSE)


    def __setSeparation(self):
        print('Separation activate at',rospy.get_time())
        return
    
    
    def __checkSeparation(self,event):
        if self.SeparationState:
            self.SeparationChecked = True
            print('Separation successful at',rospy.get_time())
            return True
        self.SeparationChecked = False
        return False
    
    def __RCSActivation(self):
        print('RCS activate at',rospy.get_time())
        return
    
    def Hold(self):
        holdStart = rospy.get_time()
        rospy.Timer(rospy.Duration(1),self.__checkRocketSOH,oneshot=False)
        while (rospy.get_time()-holdStart) < 597.0:
            if not self.RocketSOH:
                print('Rocket SOH not healthy at',rospy.get_time())
                holdStart = rospy.get_time()
                print('Countdown Reset to t-10:00')
            rospy.sleep(1)       
        rospy.Timer.shutdown()
        return True
    
    def LiftOffMode(self):
        liftOffStart = rospy.get_time()
        self.__setFirstStageIgnite()
        rospy.sleep(2.5)

        while ((rospy.get_time()-liftOffStart) < 3):
            if self.safetySwitch or self.MissionPause:
                # if safetySwitch failed or MissionPause stop and return False
                return False
            rospy.sleep(0.1)
        
        # wait for 3 second
        if not self.FirstStageIgnition:
        # Check ignition state before opening main valves
            print('First Stage Ignition Failed at',rospy.get_time())
            return False
        self.__setFirstStageMainValve(OPEN)

        rospy.sleep(4.5)
        while((rospy.get_time()-liftOffStart) < 8):
            rospy.sleep(0.1)
        
        return True
    
    def Separate(self):
        SeparationStart = rospy.get_time()
        self.__setFirstStageMainValve(CLOSE)
        rospy.sleep(0.5)

        while((rospy.get_time()-SeparationStart) < 1):
            rospy.sleep(0.1)
        # wait for 1 sec
        self.__setSeparation()

        rospy.Timer(rospy.Duration(1.5),self.__checkSeparation,oneshot=True)
        # TODO check separation if failed return

        rospy.sleep(0.5)

        while((rospy.get_time()-SeparationStart) < 2):
            rospy.sleep(0.1)
            
        
        self.__RCSActivation()
        now = rospy.get_time()
        while((now - SeparationStart) < 4):
            if not self.SeparationChecked and (now - SeparationStart) > 2.5:
                print('Separation Failed at',now)
                return False
            rospy.sleep(0.1)
            now = rospy.get_time()

        self.__setSecondStageIgnite()

        rospy.sleep(2.5)
        while((rospy.get_time()-SeparationStart) < 7):
            rospy.sleep(0.1)

        return True
    
    def Autopilot(self):
        if not self.SecondStageIgnition:
        # Check ignition state before opening main valves
            print('Second Stage Ignition Failed at',rospy.get_time())
            return False
        self.__setSecondStageMainValve(OPEN)
        rospy.Timer(rospy.Duration(15),self.__closeSecondStageMainValve,oneshot=True)
        rospy.spin()
        return True
    

if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)

    Seq=FlightSequence()
    Seq.LiftOffMode()
    Seq.Separate()
    Seq.Autopilot()
