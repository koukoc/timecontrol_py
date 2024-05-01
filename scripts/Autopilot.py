#!/usr/bin/env python3
import rospy
import numpy as np
# import math
import torch
import torch.nn as nn
import torch.nn.functional as F
from gpiozero import LED
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped,TwistStamped
import pymap3d as pm

# Define GPIO
MCLOCKWISE = LED(16)
MCONTERCLOCKWISE = LED(20)

class DQN(nn.Module):

    def __init__(self):
        super(DQN, self).__init__()
        self.layer1 = nn.Linear(2, 32)
        self.layer2 = nn.Linear(32, 32)
        self.layer3 = nn.Linear(32, 3)

    # Called with either one element to determine next action, or a batch
    # during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        x = F.leaky_relu(self.layer1(x),0.2)
        x = F.leaky_relu(self.layer2(x),0.2)
        return self.layer3(x)

class Autopilot:
    
    RollAngle = 0
    rollRate = 0
    GlobalPositionecef=[0,0,0]
    GlobalPositionlla=[0,0,0]
    autopilotState = False
    
    def state_cb(self,msg):
        self.current_state = msg
    
    def __init__(self):
        state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.RCSControlPub = rospy.Publisher('/mavros/', Bool, queue_size=10)
        rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.__local_positionCallback)
        rospy.Subscriber('local_position/velocity',NavSatFix,self.__AngularVelCallback)
        rospy.Subscriber('mavros/global_position/global',NavSatFix,self.__global_positionCallback)
        rospy.Publisher('FirstStageIgnite', PoseStamped, queue_size=10)
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


    # def lla2ecef(self):

    #     a = 6378137.0000	# earth semimajor axis in meters
    #     b = 6356752.3142;	# earth semiminor axis in meters	
    #     e = math.sqrt(1-(b/a)**2)

    #     sinphi = math.sin(self.GlobalPositionlla[0])
    #     cosphi = math.cos(self.GlobalPositionlla[0])
    #     coslam = math.cos(self.GlobalPositionlla[1])
    #     sinlam = math.sin(self.GlobalPositionlla[1])
    #     tan2phi = (math.tan(self.GlobalPositionlla[1]))^2
    #     tmp = 1 - e*e
    #     tmpden = math.sqrt( 1 + tmp*tan2phi )

    #     x = (a*coslam)/tmpden + self.GlobalPositionlla[2]*coslam*cosphi

    #     y = (a*sinlam)/tmpden + self.GlobalPositionlla[2]*sinlam*cosphi

    #     tmp2 = math.sqrt(1 - e*e*sinphi*sinphi)
    #     z = (a*tmp*sinphi)/tmp2 + self.GlobalPositionlla[2]*sinphi
    #     self.GlobalPositionecef = [x,y,z]




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

    def __local_positionCallback(self,data):
        self.rocketQuaternion = data.pose.orientation

    def __AngularVelCallback(self,data):
        self.rollRate = data.twist.angular.x
    
    def __global_positionCallback(self,data):
        self.GlobalPositionlla = [data.latitude,data.longitude,data.altitude]
        
    def despin(self,network):
        observation=np.array([self.rollRate], dtype=np.float32)
        
        state = torch.tensor(observation, dtype=torch.float32, device=self.device).unsqueeze(0)
        action = network(state).max(1)[1].view(1, 1)-1
        assert action == 1 or action == 0 or action == -1,'despin command error'
        if action == 1:
            MCONTERCLOCKWISE.on()
            MCLOCKWISE.off()
        
        if action == 0:
            MCONTERCLOCKWISE.off()
            MCLOCKWISE.off()

        if action == -1:
            MCONTERCLOCKWISE.off()
            MCLOCKWISE.on()

    def checkDespin(self):
        if abs(self.rollRate) < 0.1:
            return True
        else:
            return False

    def rollCtrl(self):
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        policy_net = DQN().to(device)
        policy_net.load_state_dict(torch.load('rollRateControl.pt'))
        policy_net.eval()
        rospy.Timer(rospy.Duration(nsecs=5e5),self.__despinpub(policy_net),oneshot=False)
        # 加despin callback

    def despinStop(seif):
        rospy.Timer.shutdown()
    
    def getAltitudeSetpoint(self,Targetlla,):
        enu_err = pm.geodetic2enu(Targetlla[0], Targetlla[1], Targetlla[2], self.GlobalPositionlla[0], self.GlobalPositionlla[1], self.GlobalPositionlla[2])
        azimuthOri = np.arctan2(enu_err[0],enu_err[1])
        elevationOri = np.arctan2(enu_err[2],enu_err[1])

        croll = np.cos(self.RollAngle)
        sroll = np.sin(self.RollAngle)
        se = np.sin(elevationOri)
        ce = np.cos(elevationOri)
        ca = np.cos(azimuthOri)
        sa = np.sin(azimuthOri)

        azimuth = np.arcsin(sroll*se*ca+sa*croll)
        elevation = np.arctan2(croll*se*ca-sa*sroll,ce*ca)
        return azimuth,elevation

    def getRollAngle(self,w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        self.RollAngle = np.degrees(np.arctan2(t0, t1))

    def AttitudeCtrl(self):
        Targetlla = [474977420e-7,85555940e-7,10000]
        self.getRollAngle(self.rocketQuaternion.w,self.rocketQuaternion.x,self.rocketQuaternion.y,self.rocketQuaternion.z) 
                          # update Roll Angle value
        azimuth,elevation = self.getAltitudeSetpoint(Targetlla)

        # send control command
