#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool,Float32
from Autopilot import Autopilot
from pressureState import PressureState
OPEN = True
CLOSE = False



class FlightSequence:
    safetySwitch = False
    MissionStartTime = 0
    GroundFireSignal = False
    FirstStageMainValveState = False
    FirstStageIgnitionState = False
    leavetheRackState = False
    FirstStageIgnition = False
    SecondStageIgnition = False
    SeparationState = False
    SeparationChecked = True
    MissionPause = False
    RocketSOH = False
    LiftOffModeTime = 0

    def __GroundFireSignalCallback(self,data):
        self.GroundFireSignal = data.data
        
    # def __firstStageIgnitionStateCallback(self,data):
    #     self.FirstStageIgnitionState = data.data

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
        if self.PressureSub.FirstTankPressure > 75 or self.PressureSub.FirstTankPressure < 50:
            self.RocketSOH = False
        
        if self.PressureSub.SecondTankPressure > 70 or self.PressureSub.SecondTankPressure < 50:
            self.RocketSOH = False

        if self.PressureSub.RCSTankPressure > 15 or self.PressureSub.RCSTankPressure < 5: #TODO needs to be check
            self.RocketSOH = False

        self.RocketSOH = True
        return
    
    def checkSafetySwitch(self):
        if self.safetySwitch:
            return True
        else:
            return False

    def __init__(self):
        self.PressureSub = PressureState()
        # safetySwitchCallback
        rospy.Subscriber('GroundFireSignalState',Bool,self.__GroundFireSignalCallback)
        # rospy.Subscriber('FirstStageIgnitionState',Bool,self.__firstStageIgnitionStateCallback)
        # rospy.Subscriber('SecondStageIgnitionState',Bool,self.__secondStageIgnitionStateCallback)
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)
        rospy.Subscriber('SafetySwitchState',Bool,self.__SafetySwitchStateCallback)
        rospy.Subscriber('FirstStageMainValveState',Float32,self.__FirstStageMainValveStateCallback)
        # TODO second stage valve check
        rospy.Subscriber('leavetheRackState',Bool,self.__leavetheRackStateCallback)

        # First Stage igniter publishment
        self.FirstStageIgnitePub = rospy.Publisher('FirstStageIgnite', Bool, queue_size=10)
        self.FirstStageChargedPub = rospy.Publisher('FirstStageCharged', Bool, queue_size=10)
        self.FirstStageDischargedPub = rospy.Publisher('FirstStageDischarged', Bool, queue_size=10)

        # Second stage igniter publishment
        self.SecondStageIgnitePub = rospy.Publisher('SecondStageIgnite', Bool, queue_size=10)
        self.SecondStageChargedPub = rospy.Publisher('SecondStageCharged', Bool, queue_size=10)
        self.SecondStageDischargedPub = rospy.Publisher('SecondStageDischarged', Bool, queue_size=10)

        # oxidizer valve open publishment
        self.FirstStageMainValvePub = rospy.Publisher('FirstStageMainValveOpened', Bool, queue_size=10)
        self.SecondStageMainValvePub = rospy.Publisher('SecondStageMainValveOpened', Bool, queue_size=10)
        self.SeparatePub = rospy.Publisher('Separate', Bool, queue_size=10)

        # Autopilot initialization
        # self.Mission = Autopilot()
        # while not self.Mission.updateAutopilotState():
        #     rospy.loginfo('Autpilot Not Ready')
        #     rospy.sleep(0.5)
        # separation callback
        # self.MissionStartTime = rospy.get_time()
        rospy.sleep(1)
        return


    def __setFirstStageIgnite(self):
        self.FirstStageChargedPub.publish(True)
        # charge Second stage igniter with First stage
        self.SecondStageChargedPub.publish(True)
        print('ignitor charged')
        # 12 sec was verified
        # TODO check 10 sec
        rospy.sleep(10.0)
        self.FirstStageIgnitePub.publish(True)
        print('1st Stage Ignited at',rospy.get_time())
        self.FirstStageIgnitionState=True
        # discharge
        # self.FirstStageDischargedPub.publish(True)
        rospy.Timer(rospy.Duration(1),self.__setFirstStageIgniterDischarge,oneshot=True)
        print('ignitor discharged')

        return

    def __setFirstStageIgniterDischarge(self):
        self.FirstStageDischargedPub.publish(True)
        return
        

    def __setSecondStageIgnite(self):
        self.SecondStageIgnitePub.publish(True)
        self.SecondStageIgnitionState=True
        # discharge Second Stage igniter 1 sec after ignition 
        rospy.Timer(rospy.Duration(1),self.__setSecondStageIgniterDischarge,oneshot=True)
        print('2nd Stage Ignited at',rospy.get_time())
        return
    
    def __setSecondStageIgniterDischarge(self):
        self.SecondStageDischargedPub.publish(True)
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
    
    # def __closeSecondStageMainValve(self,event):
    #     self.__setSecondStageMainValve(CLOSE)


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
        # self.Mission.rollCtrlStart()
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
        igniteOnce = False
        ignitetime = 0 # TODO check ingnite logic
        # TODO change to duration
        for groundFire in range(100):
            rospy.loginfo('%.1f sec to send iginite signal',10-groundFire*0.1)
            # if recieved fire signal from groundstation, start first stage ignite 
            if self.GroundFireSignal and not igniteOnce:
                print('recieved signal')
                rospy.Timer(rospy.Duration(secs=0,nsecs=1),self.__setFirstStageIgnite(),oneshot=True)
                igniteOnce = True
                ignitetime = rospy.get_time()
                break
            # check every 0.1 second after countdown end
            rospy.sleep(0.1)
            if groundFire == 99:
                print('countdown timeout without fire signal on ground')
                # rospy.loginfo
                return False

        while(True):
            if self.FirstStageIgnitionState:
                rospy.loginfo('start waiting for main valve open')
                ignitetime = rospy.get_time()
                break
            rospy.sleep(0.05)
        while(True):
            if (rospy.get_time()-ignitetime)>1.5:
            # Opened valve 1.5 seconds after ignite happened, then open first stage valve 
                self.__setFirstStageMainValve(True)

            # if and only if first stage valve open and then continous mission
            if self.FirstStageMainValveState>0.9 and self.FirstStageMainValveState<1.1:
                self.LiftOffModeTime = rospy.get_time()
                break
            else:
                print('wait for First Stage Main Valve at',rospy.get_time())
            
            if (rospy.get_time()-ignitetime)>8:
                rospy.loginfo('wait for main Valve for too long')
                self.__setFirstStageMainValve(False)
                return False
            rospy.sleep(0.1)

            
        
        while not self.leavetheRackState:
            rospy.sleep(0.01)
            if (rospy.get_time()-self.LiftOffModeTime) > 5:
                self.__setFirstStageMainValve(False)
                # if rocket not leaving the rack after first stage main valve opened 2 sec
                # close main valve and interrupting mission
                return False
            
        self.MissionStartTime = rospy.get_time()
        # Mission Start time defined at when rocket leave the rack

        while((rospy.get_time()-self.LiftOffModeTime) < 5):
            rospy.sleep(0.1)
        self.__setFirstStageMainValve(CLOSE)
        # t+5
        return True
    
    def Separate(self):
        SeparationStart = rospy.get_time()
        # t+5
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
        while((now - SeparationStart) < 6.5):
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
        # t+10.5
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
        # if self.Mission.checkDespin():
        #     self.Mission.startAttitudeCtrl()
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
