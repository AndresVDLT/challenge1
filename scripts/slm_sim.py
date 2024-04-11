#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from math import cos

#Declare Variables to be used
k = 0.01
m = 0.75
l = 0.36
g = 9.8
Tau = 0.0
x1 = 0.0
x2 = 0.0
a = l/2
J = 4/3*m*a*a

# Setup Variables to be used

# Declare the input Message

# Declare the  process output message


#Define the callback functions
def tau_callback(msg):
    global Tau
    Tau = msg.data


  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
    dt = 0.01

    # Setup the Subscribers
    sub = rospy.Subscriber("tau", Float32, callback=tau_callback)

    #Setup de publishers
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)

    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
        
            #WRITE YOUR CODE HERE
        while not rospy.is_shutdown():
            #SLM governing equation 
            x1 += x2 * dt
            x2_dot = (1/(J+m*a*m*a)) * (-m*g*a*cos(x1) - k*x2 + Tau)
            x2 += dt * x2_dot
            pub_msg = JointState()
            pub_msg.header.stamp = rospy.Time.now()
            pub_msg.position = [x1]
            pub_msg.velocity = [x2]
            pub_msg.name = ["joint2"]
            pub.publish(pub_msg)
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node