#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs
from numpy import *
from sensor_msgs.msg import JointState

class Acrobot:
    # kinematic, dynamic parameter
    m1 = 1
    m2 = 1
    I1 = 1
    I2 = 1
    l1 = 1
    l2 = 1
    lc1 = 0.5
    lc2 = 0.5
    H = zeros((2,2))
    C = zeros((2,2))
    G = zeros((2,1))
    # state
    q = zeros((2,1))
    dq = zeros((2,1))
    
    # desired value
    ddq1_d = 0
    dq1_d = 0
    q1_d = pi

    # torque
    tau2 = 0.0

    # gain
    kp = 1
    kv = 0.2

    def inv_dynamics(self, q, dq):
        g = 9.81
        self.H[0,0] = self.I1 + self.I2 + self.m2*self.l1*self.l1 + 2*self.m2*self.l1*self.lc2*cos(self.q[1])
        self.H[0,1] = self.I2 + self.m2*self.l1*self.lc2*cos(self.q[1])
        self.H[1,0] = self.I2 + self.m2*self.l1*self.lc2*cos(self.q[1])
        self.H[1,1] = self.I2

        self.C[0,0] = -2*self.m2*self.l1*self.lc2*sin(self.q[1])*self.dq[1]
        self.C[0,1] = -self.m2*self.l1*self.lc2*sin(self.q[1])*self.dq[1]
        self.C[1,0] = self.m2*self.l1*self.lc2*sin(self.q[1])*self.dq[0]
        self.C[1,1] = 0

        self.G[0] = self.m1*g*self.lc1*sin(self.q[0]) + self.m2*g*(self.l1*sin(self.l1*sin(self.q[0]) + self.lc2*sin(self.q[0] + self.q[1])))
        self.G[1] = self.m2*g*self.lc2*sin(self.q[0] + self.q[1])

    def swingup_control(self):
        
        self.inv_dynamics(self.q, self.dq)
        phi = self.C.dot(self.dq) + self.G
        v = self.ddq1_d - self.kv*(self.dq1_d - self.dq[0]) - self.kp*(self.q1_d - self.q[0])
        
        self.tau2 = ( self.H[1,0] - self.H[1,1]*self.H[0,0]/self.H[0,1] ) * v - self.H[1,1]/self.H[0,1]*phi[0] + phi[1]

        return self.tau2

        def lqr_control(self):
            pass

#Joint state publisher
def acrobot_joint_state_subscriber(data):
    acrobot.q[0] = data.position[0]
    acrobot.q[1] = data.position[1]
    acrobot.dq[0] = data.velocity[0]
    acrobot.dq[1] = data.velocity[1]
    

#Define a RRBot joint positions publisher for joint controllers.
def acrobot_control_publisher():

    #Initiate node for controlling joint1 and joint2 positions.
    rospy.init_node('acrobot_control_node', anonymous=True)

    #Define subscribers for each joint state
    rospy.Subscriber('/rrbot/joint_states', JointState, acrobot_joint_state_subscriber)

    #Define publishers for each joint position controller commands.
    pub2 = rospy.Publisher('/rrbot/joint2_torque_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(100) #100 Hz

    #While loop to have joints follow a certain position, while rospy is not shutdown.
    i = 0
    while not rospy.is_shutdown():

        #Publish torque
        pub2.publish(acrobot.swingup_control())

        i = i+1 #increment i

        rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    acrobot = Acrobot()
    try: acrobot_control_publisher()
    except rospy.ROSInterruptException: pass
