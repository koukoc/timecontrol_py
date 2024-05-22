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
    LiftOffModeTime = 0

    def __firstStageIgnitionStateCallback(self,data):
        self.FirstStageIgnitionState = data.data

    def __secondStageIgnitionStateCallback(self,data):
        self.SecondStageIgnitionState = data.data

    def __SeparationStateCallback(self,data):
        self.SeparationState = data.data
    
    def __SafetySwitchStateCallback(self,data):
        self.safetySwitch = data.data

    def __FirstStageMainValveStateCallback(self,data):
        self.FirstStageMainValveState = data.data

    def __leavetheRackStateCallback(self,data):
        self.leavetheRackState = data.data

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
        rospy.Subscriber('FirstStageIgnitionState',Bool,self.__firstStageIgnitionStateCallback)
        rospy.Subscriber('SecondStageIgnitionState',Bool,self.__secondStageIgnitionStateCallback)
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)
        rospy.Subscriber('SafetySwitchState',Bool,self.__SafetySwitchStateCallback)
        rospy.Subscriber('FirstStageMainValveState',Bool,self.__FirstStageMainValveStateCallback)
        rospy.Subscriber('leavetheRackState',Bool,self.__leavetheRackStateCallback)
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
        self.Mission.rollCtrlStart()
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
        for groundFire in range(100):
            # if and only if first stage valve open and rocket leave rack then continous mission
            if self.FirstStageMainValveState and self.leavetheRackState:
                self.LiftOffModeTime = rospy.get_time()
                break
            # check every 0.1 second after countdown end
            rospy.sleep(0.1)
            if groundFire == 99:
                print('countdown timeout without fire on ground')
                # rospy.loginfo
                return False

        rospy.sleep(2.5)
        while((rospy.get_time()-self.LiftOffModeTime) < 5):
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
        # PX4 set rocket destination with MavROS
        # 第二節火箭不關閉
        # rospy.Timer(rospy.Duration(15),self.__closeSecondStageMainValve,oneshot=True)
        return True
    

if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)

    Seq=FlightSequence()
    Seq.LiftOffMode()
    Seq.Separate()
    Seq.Autopilot()
