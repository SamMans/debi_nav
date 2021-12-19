#!/usr/bin/env python3

# Author: Samer A. Mohamed ()

import rospy
from numpy import array, where, ones, round_, copy, int8
from math import sqrt
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from std_msgs.msg import Float32MultiArray
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from scipy.signal import convolve2d

#Define global variables
origin_x = rospy.get_param('/xmin') #Map origin x-coordinate relative to robot initial pose "initial - subject to change during simulation"
origin_y = rospy.get_param('/ymin') #Map origin y-coordinate relative to robot initial pose "initial - subject to change during simulation"
map_res = rospy.get_param('/delta') #Map resolution
map_h = [] #Map height
map_w = [] #Map width
rob_diam = rospy.get_param('/col_diam') #Robot diameter
kernel = ones((round(rob_diam/(map_res*2)),round(rob_diam/(map_res*2))), dtype = int) #Map dilation kernel size "to account for the entire robot body collision not just the center-point"
path_to_con = Float32MultiArray() #Path coordinates "sent to the MPC control module"
Fmap_grid = OccupancyGrid() #Full map grid message, "solid + visual" obstacles included
Pose_pix = [] #Robot current pose as pixels on the map
Goal_pix = [] #Desired current goal as pixels on the map
Vlayer = [] #Visual layer "shows visual obstacles only"

def vscan_callback(msg):
  #Global variables associated with this function
  global Vlayer
  
  #Check whether the visual layer is still not initialized
  #If not: check whether map width and height are known and initialize a visual layer of the same size as the final map
  if len(Vlayer) == 0:
    if map_h != [] and map_w != []:
      Vlayer = array([[0]*map_w]*map_h) 
      
  #If yes: convert the obstacle coordinates message from meters to pixels 
  if len(Vlayer) != 0 and len(msg.data) >= 2:
    vis_pix = array(msg.data)
    vis_pix[0:len(vis_pix)-1:2] = round_(vis_pix[0:len(vis_pix)-1:2]/map_res - origin_x/map_res + 1/2)
    vis_pix[1:len(vis_pix):2] = round_(vis_pix[1:len(vis_pix):2]/map_res - origin_y/map_res + 1/2)
    vis_pix = vis_pix.astype(int)
    
    #Fill the visual obstacle pixels with ones "Indicating an obstacle"
    Vlayer[vis_pix[1:len(msg.data):2],vis_pix[0:len(msg.data)-1:2]] = 1

def mapdata_callback(msg):
  #Global variables associated with this function
  global origin_x
  global origin_y
  global map_res
  global map_h
  global map_w
  
  #Assign map data like size, resolution, .. etc.
  map_res = msg.resolution
  map_h = msg.height
  map_w = msg.width
  origin_x = msg.origin.position.x
  origin_y = msg.origin.position.y
  
def goal_callback(msg):
  #Global variables associated with this function
  global Goal_pix
  
  #Update robot pose (from pose in meter to pose in pixels transformation)
  Goal_pix = [round(msg.data[0]/map_res - origin_x/map_res + 1/2), round(msg.data[1]/map_res - origin_y/map_res + 1/2)]

def pose_callback(msg):
  #Global variables associated with this function
  global Pose_pix
  
  #Update robot pose (from pose in meter to pose in pixels transformation)
  Pose_pix = [round(msg.pose.pose.position.x/map_res - origin_x/map_res + 1/2), round(msg.pose.pose.position.y/map_res - origin_y/map_res + 1/2)] 
   
def lscan_callback(msg):
  #Global variables associated with this function
  global path_to_con
  
  #Map processing
  if map_h != [] and map_w != []:
    Llayer = array(msg.data) #Convert map to numpy array "Laser scan layer"
    Llayer = where(Llayer<=50,0,Llayer) #Free pixels in laser scan layer (Logic 0 instead of probability percentage)
    Llayer = where(Llayer>50,1,Llayer) #Occupied pixels in laser scan layer (Logic 1 instead of probability percentage)
    Llayer.shape = map_h, map_w #Change laser scan layer from row-major form to 2D row-column form
    Flayer = Llayer #The full map should first include "at least" solid obstacles
    
    #Full map formation
    if len(Vlayer) != 0:
      Flayer = Llayer + Vlayer #Superimpose the visual layer and the laser scan layer to form full scans layer
      Fmap = copy(Flayer) #Copy full layer data into full map
      Fmap.shape = map_h*map_w #Change full map format to row major form
      Fmap = Fmap*100 #Change obstacle probability range from 0-1 to 0-100
      Fmap_grid.info = msg.info #Update message info to suit that of input message map
      Fmap_grid.header = msg.header #Update message header to suit that of input message map
      Fmap_grid.data = Fmap.astype(int8).tolist() #Update messsage data
      fmap_pub.publish(Fmap_grid) #Publish full map
      
    #Process full layer for path planning application
    Flayer = convolve2d(Flayer, kernel, mode='same') #Dilate full layer (obstacles should grow bigger to account for robot radius)
    Flayer = where(Flayer>1,1,Flayer) #Reduce convoluted elements to 1 (since convolution might result in some pixel values growing above 1)
    Flayer = 1-Flayer #Change logic 1 to 0 and logic 0 to 1 [Logic shift, because A* planner sees 1 as empty and 0 as occupied]
    Flayer_list = Flayer.tolist() #Change full scan layer from array format to list format
    
    #Path generation
    if Pose_pix != [] and Goal_pix != []:
      grid = Grid(matrix=Flayer_list) #Create a grid out of the full layer
      start = grid.node(Pose_pix[0], Pose_pix[1]) #Define the start of the A* algorithm
      end = grid.node(Goal_pix[0], Goal_pix[1]) #Define the target of the A* algorithm (coming from the mission planner)
      finder = AStarFinder(diagonal_movement=DiagonalMovement.always) #Apply A*
      path, runs = finder.find_path(start, end, grid) #Get path pixels (in list of tuples format)
      path = [list(elem) for elem in path] #Get path pixels (in list of lists format)
      flat_path = [item for sublist in path for item in sublist] #Flatten the path list of lists to obtain 1-D list [x1,y1,x2,y2,x3,y3,....,xN,yN]
      flat_path[1:len(flat_path):2] = [x*map_res - map_res/2 + origin_x  for x in flat_path[1:len(flat_path):2]] #Change the unit from pixel to meters before sending to controller
      flat_path[0:len(flat_path)-1:2] = [y*map_res - map_res/2 + origin_y  for y in flat_path[0:len(flat_path)-1:2]] #Change the unit from pixel to meters before sending to controller 
      path_to_con.data = flat_path #Embed the new path into the "path to controller" message
      path_pub.publish(path_to_con) #Publish final path message
  
#Initialize the map making node
rospy.init_node('pp_debi')
path_pub = rospy.Publisher('/path',Float32MultiArray,queue_size=1)
fmap_pub = rospy.Publisher('/fmap',OccupancyGrid,queue_size=10)
rospy.Subscriber('/map',OccupancyGrid,lscan_callback)
rospy.Subscriber('/odom',Odometry,pose_callback)
rospy.Subscriber('/targ',Float32MultiArray,goal_callback)
rospy.Subscriber('/map_metadata',MapMetaData,mapdata_callback)
rospy.Subscriber('/vscan',Float32MultiArray,vscan_callback)

#Looping
rospy.spin()
