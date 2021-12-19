#!/usr/bin/env python3

# Author: Samer A. Mohamed ()

import rospy
from std_msgs.msg import Float32MultiArray
from casadi import*
from numpy import eye, array, sqrt, amin, where
from math import atan2, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Twist, Vector3

#Controller settings
T = rospy.get_param('/T') #sampling time (controller)
N = rospy.get_param('/N') #prediction horizon
inst_pose = [] #start with empty instantaneous pose
curr_path = [] #start with empty path 
inst_goal = [] #empty instantaneous goal waiting to be filled
Goal = [] #final pose of current navigation mission

v_max = rospy.get_param('/v_max') #linear velocity maximum limit
v_min = -v_max #linear velocity minimum limit
omega_max = rospy.get_param('/omega_max') #angular velocity maximum limit
omega_min = -omega_max #angular velocity minimum limit

P = SX.sym('P',6) #Parameters array [Initial pose + Target pose]
X = SX.sym('X',3,N+1) #States along prediction horizon "N"
U = SX.sym('U',2,N) #Control actions along prediction horizon "N"

#Initialization
X[:,0] = P[0:3]
for k in range(N):
    st = X[:,k] #Current state
    con = U[:,k] #Current control
    X[0,k+1] = st[0] + T*con[0]*cos(st[2]) #Dynamic model equation for x-coordinate
    X[1,k+1] = st[1] + T*con[0]*sin(st[2]) #Dynamic model equation for y-coordinate
    X[2,k+1] = st[2] + T*con[1] #Dynamic model equation for theta

#Constraints (already defined in path planning "No need" so we send an empty array to Casadi)
g = []

#Objective function (Sum variable-initialize with zero then add losses to sum)
obj = 0
Q = eye(3) #Performance weight 
Q[0][0] = rospy.get_param('/pw')
Q[1][1] = rospy.get_param('/pw')
Q[2][2] = rospy.get_param('/pw')
R = eye(2) #Effort weight
R[0][0] = rospy.get_param('/ew')
R[1][1] = rospy.get_param('/ew')
for k in range(N):
    st = X[:,k] #Current state
    con = U[:,k] #Current control
    err = st-P[3:6] #Error
    obj = obj + (err).T@Q@(err) + (con).T@R@(con)

#Solver configuration
OPT_variables = reshape(U,2*N, 1)
nlp_prob = {'f': obj, 'x': OPT_variables, 'g': g, 'p': P}
opts = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0,
'ipopt.acceptable_tol': 1e-8,'ipopt.acceptable_obj_change_tol': 1e-6}
solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

#Input constraints
LB = [0] * (2*N) #Lower bound limits (minimums)
for k in range(2*N):
    if(k%2 == 0):
      LB[k] = v_min
    else:
      LB[k] = omega_min
UB = [0] * (2*N) #Upper bound limits (maximums)
for k in range(2*N):
    if(k%2 == 0):
      UB[k] = v_max
    else:
      UB[k] = omega_max
 
#Callback functions
def pose_callback(pose_msg):
  #Associated global variable
  global inst_pose
  
  #Update instantaneous pose
  inst_pose = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 2*atan2(pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w)]
   
def path_callback(path_msg):
  #Associated global variable
  global curr_path
  
  #Update current path plan
  curr_path = path_msg.data
  
def goal_callback(msg):
  #Associated global variable
  global Goal
  
  #Final desired pose of current mission
  Goal = msg.data
      
#Initialize your MPC node (publish: vel (to Turtlebot twist), subscribe to: pose (from SLAM), path (from A*))
rospy.init_node('MPC_debi')
pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
rospy.Subscriber('/odom',Odometry,pose_callback)
rospy.Subscriber('/path',Float32MultiArray,path_callback)
rospy.Subscriber('/targ',Float32MultiArray,goal_callback)
 
#Initialize velocity message
Vel = Twist()
 
#Main control loop
while not rospy.is_shutdown():
  if(inst_pose!=[] and len(curr_path)>2):
    #Pick the closest instantaneous target (smallest Eucledian distance)
    init_pose = array(inst_pose) #Initial pose for the control problem "numerical, not symbolic"
    path_x = array(curr_path[0:len(curr_path)-1:2]) #Path (x-coordinates only)
    path_y = array(curr_path[1:len(curr_path):2]) #Path (y-coordinates only)
    Dist = sqrt((init_pose[0]-path_x)*(init_pose[0]-path_x) + (init_pose[1]-path_y)*(init_pose[1]-path_y)) #Get distance between each path point and current pose
    min_Dist = where(Dist == amin(Dist)) #Get index of minimum distance
    I = min_Dist[0][0] + 1 #Compute index of new target
    if I <= (len(curr_path)/2)-1:
      inst_goal = [path_x[I], path_y[I], atan2(path_y[I]-path_y[I-1], path_x[I]-path_x[I-1])] #Formulate next target [next x, next y, next theta "using atan"]
      
    #Pass parameters to solver and solve
    u0 = [0] * (N*2) #Initial solver controls "random, I use zeros by default to initialize the decision variables"
    Param = inst_pose
    Param.extend(inst_goal) #Parameters vector "using actual numbers, not symbols"
    sol = solver(lbx = LB, ubx = UB, p = Param) #Get solution
    u = reshape(sol['x'].full().T,2,N)
    v = u[0][0]
    omega = u[1][0]
    
  elif(inst_pose!=[] and len(curr_path)==2):
    inst_goal = list(Goal) #Formulate next target [next x, next y, next theta "using atan"]
    #Pass parameters to solver and solve
    u0 = [0] * (N*2) #Initial solver controls "random, I use zeros by default to initialize the decision variables"
    Param = inst_pose
    Param.extend(inst_goal) #Parameters vector "using actual numbers, not symbols"
    sol = solver(lbx = LB, ubx = UB, p = Param) #Get solution
    u = reshape(sol['x'].full().T,2,N)
    v = u[0][0]
    omega = u[1][0]
    
  else:
    v = 0
    omega = 0
    
  #Finalize and publish  
  Vel.linear.x = v
  Vel.angular.z = omega
  pub.publish(Vel)  
  
  #Sleep for sampling time period
  rospy.sleep(T)
