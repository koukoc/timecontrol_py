#!/usr/bin/env python3
import rospy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from gpiozero import LED
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

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
        self.RCSControlPub = rospy.Publisher('/mavros/', Bool, queue_size=10)
        rospy.Subscriber('RollRate',Bool,self.__rollRateCallback)
        rospy.Subscriber('ecef',Bool,self.__ecefCallback)
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


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
        

    def startAttitudeCtrl(self):
        self.ecef
        # 利用現在的座標與目標座標相減求姿態

