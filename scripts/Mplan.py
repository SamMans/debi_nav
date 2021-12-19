#!/usr/bin/env python3

# Author: Samer A. Mohamed ()

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from math import atan2

#Define global variables
FSM = 0 #Finite state machine variable "Like pointer to current mission"
com_msg = Float32MultiArray() #Command message "Go to target pose [x, y, z]"
Pose = [0.0,0.0,0.0] #Current pose variable
lin_thres = rospy.get_param('/lin_thres') #Intermediate goal error threshold

#Callbacks
def pose_callback(msg):
  #Associated global variables
  global Pose
  
  #update instantaneous pose
  Pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, 2*atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)]

#Initialize the mission planning node
rospy.init_node('mp_debi')
fsm_pub = rospy.Publisher('/targ',Float32MultiArray,queue_size=1) #Target command publisher
rospy.Subscriber('/odom',Odometry,pose_callback)

#Looping
while not rospy.is_shutdown():
  #First state: Go to the first X sign
  if FSM==0:
    com_msg.data = [1.6,2.8,1.5708]
    if abs(Pose[0]-com_msg.data[0]) <= lin_thres and abs(Pose[1]-com_msg.data[1]) <= lin_thres:
      #Increment FSM variable
      FSM = FSM + 1
      
  #Second state: Go to the second X sign 
  elif FSM==1:
    com_msg.data = [-0.9,2.2,-1.5708]
    if abs(Pose[0]-com_msg.data[0]) <= lin_thres and abs(Pose[1]-com_msg.data[1]) <= lin_thres:
      #Increment FSM variable
      FSM = FSM + 1
      
  #Third state: Go to the third X sign
  elif FSM==2:
    com_msg.data = [1.6,-0.95,1.5708]
    if abs(Pose[0]-com_msg.data[0]) <= lin_thres and abs(Pose[1]-com_msg.data[1]) <= lin_thres:
      #Increment FSM variable
      FSM = FSM + 1
      
  #Fourth state: Go to the fourth X sign
  elif FSM==3:
    com_msg.data = [-2.0,0.6,0.0]
    if abs(Pose[0]-com_msg.data[0]) <= lin_thres and abs(Pose[1]-com_msg.data[1]) <= lin_thres:
      #Increment FSM variable
      FSM = FSM + 1
      
  #Fifth state: Go back to the starting position
  elif FSM==4:
    com_msg.data = [0.0,0.0,0.0]
    
  #Publish mission planner command
  fsm_pub.publish(com_msg)
  rospy.sleep(0.5)
