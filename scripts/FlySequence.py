#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from Autopilot import Autopilot
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
    
    def __SafetySwitchStateCallback(self,data):
        self.safetySwitch = data.data

    def checkRocketSOH(self,event):
        self.RocketSOH = True
        return
    
    def checkSafetySwitch(self):
        if self.safetySwitch:
            return True
        else:
            return False

    def __init__(self):
        # safetySwitchCallback
        rospy.Subscriber('FirstStageIgnition',Bool,self.__firstStageIgnitionCallback)
        rospy.Subscriber('SecondStageIgnition',Bool,self.__secondStageIgnitionCallback)
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)
        rospy.Subscriber('SafetySwitchState',Bool,self.__SafetySwitchStateCallback)
        self.FirstStageIgnitePub = rospy.Publisher('FirstStageIgnite', Bool, queue_size=10)
        self.SecondStageIgnitePub = rospy.Publisher('SecondStageIgnite', Bool, queue_size=10)
        self.FirstStageMainValvePub = rospy.Publisher('FirstStageMainValveOpened', Bool, queue_size=10)
        self.SecondStageMainValvePub = rospy.Publisher('SecondStageMainValveOpened', Bool, queue_size=10)
        self.SeparatePub = rospy.Publisher('Separate', Bool, queue_size=10)

        self.Mission = Autopilot()
        while not self.Mission.updateAutopilotState():
            rospy.loginfo('Autpilot Not Ready')
            rospy.sleep(0.5)
        # separation callback
        self.MissionStartTime = rospy.get_time()
        rospy.sleep(1)
        return


    def __setFirstStageIgnite(self):
        self.FirstStageIgnitePub.publish(True)
        print('1st Stage Ignited at',rospy.get_time())
        return
    
    def __setSecondStageIgnite(self):
        self.SecondStageIgnitePub.publish(True)
        print('2nd Stage Ignited at',rospy.get_time())
        return
    
    def __setFirstStageMainValve(self,command):
        if command:
            self.FirstStageMainValvePub.publish(True)
            print('1st Main Valve Opened at',rospy.get_time())
        else:
            self.FirstStageMainValvePub.publish(False)
            print('1st Main Valve Closed at',rospy.get_time())
        return
    
    def __setSecondStageMainValve(self,command):
        if command:
            self.SecondStageMainValvePub.publish(True)
            print('2nd Main Valve Opened at',rospy.get_time())
        else:
            self.SecondStageMainValvePub.publish(False)
            print('2nd Main Valve Closed at',rospy.get_time())
        return
    
    def __closeSecondStageMainValve(self,event):
        self.__setSecondStageMainValve(CLOSE)


    def __setSeparation(self):
        self.SeparatePub.publish(True)
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
        self.Mission.rollCtrl()
        print('RollRate Control activate at',rospy.get_time())
        return
    
    def Hold(self):
        holdStart = rospy.get_time()
        checkSOHTimer=rospy.Timer(rospy.Duration(1),self.checkRocketSOH,oneshot=False)
        # t-10:00
        while (rospy.get_time()-holdStart) < 598.0:
            if not self.RocketSOH:
                print('Rocket SOH not healthy at',rospy.get_time())
                holdStart = rospy.get_time()
                print('Countdown Reset to t-10:00')
            # t-60
            if (rospy.get_time()-holdStart) > 538.0:
                if self.checkSafetySwitch():
                    print('Safety Switch ARMED at',rospy.get_time())
                else:
                    print('Safety Switch Disarmed restart count down at t-60')
                    holdStart = rospy.get_time()-538

            rospy.sleep(0.5)       
        checkSOHTimer.shutdown()
        return True
    
    def LiftOffMode(self):
        liftOffStart = rospy.get_time()
        # t-2
        self.__setFirstStageIgnite()

        while ((rospy.get_time()-liftOffStart) < 2):
            if self.safetySwitch or self.MissionPause:
                # if safetySwitch failed or MissionPause stop and return False
                return False
            rospy.sleep(0.1)
        
        # wait for 2 second
        if not self.FirstStageIgnition:
        # Check ignition state before opening main valves
            print('First Stage Ignition Failed at',rospy.get_time())
            return False
        # t Launch
        self.__setFirstStageMainValve(OPEN)

        rospy.sleep(2.5)
        while((rospy.get_time()-liftOffStart) < 7):
            rospy.sleep(0.1)
        # t+5
        return True
    
    def Separate(self):
        SeparationStart = rospy.get_time()
        # t+5
        self.__setFirstStageMainValve(CLOSE)
        rospy.sleep(0.5)

        while((rospy.get_time()-SeparationStart) < 1):
            rospy.sleep(0.1)
        # wait for 1 sec
        # t+6
        self.__setSeparation()

        SeparationCheck = rospy.Timer(rospy.Duration(0.3),self.__checkSeparation)
        # TODO check separation if failed return


        
        rcslaunchOnce = 0
        now = rospy.get_time()
        while((now - SeparationStart) < 5):
            if (now - SeparationStart) > 2.5:
                if not self.SeparationChecked:
                    # t+7.5
                    print('Separation Failed at',now)
                elif(rcslaunchOnce == 0):
                    # t+7.5
                    self.__RCSActivation()
                    rcslaunchOnce = 1
                    SeparationCheck.shutdown()
                
            rospy.sleep(0.1)
            now = rospy.get_time()
        if not self.SeparationChecked:
            print('Separation Failed at t+10 mission abort')
            return False
        # t+10
        self.__setSecondStageIgnite()

        while((rospy.get_time()-SeparationStart) < 7):
            rospy.sleep(0.1)
        # t+12
        return True
    
    def Autopilot(self):
        if not self.SecondStageIgnition:
        # Check ignition state before opening main valves
            print('Second Stage Ignition Failed at',rospy.get_time())
            return False
        # t+12 無論減滾有無成功先開主閥，再檢查減滾是否成功，減滾成功則進入導航模式，失敗則繼續飛行
        self.__setSecondStageMainValve(OPEN)
        if self.Mission.checkDespin():
            self.Mission.startAttitudeCtrl()
        # TODO PX4 set rocket destination with MavROS
        # 第二節火箭不關閉
        # rospy.Timer(rospy.Duration(15),self.__closeSecondStageMainValve,oneshot=True)
        rospy.spin()
        return True
    

if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)

    Seq=FlightSequence()
    Seq.LiftOffMode()
    Seq.Separate()
    Seq.Autopilot()
