#!/usr/bin/env python3

# Author: Samer A. Mohamed ()

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Twist, Vector3
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from math import cos, sin, atan2

#Image projection parameters
top_x = rospy.get_param('/top_x')
top_y = rospy.get_param('/top_y')
bottom_x = rospy.get_param('/bottom_x')
bottom_y = rospy.get_param('/bottom_y')

#CV bridge object
cvBridge = CvBridge()

#Visual obstacle message
Vobs = Float32MultiArray()

#Pose vector "initially empty"
Pose = []

#Callbacks
def pose_callback(msg):
  #Associated global variable
  global Pose
  
  #update instantaneous pose
  Pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, 2*atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)]

def img_callback(msg):
  #converts compressed image to opencv image
  np_img_input = np.frombuffer(msg.data, np.uint8)
  cv_img_input = cv2.imdecode(np_img_input, cv2.IMREAD_COLOR)
  
  #adding Gaussian blur to the image of original
  cv_img_input = cv2.GaussianBlur(cv_img_input, (5, 5), 0)
  
  ##homography transform process
  #selecting 4 points from the original image
  pts_src = np.array([[320 - top_x, 360 - top_y], [320 + top_x, 360 - top_y], [320 + bottom_x, 240 + bottom_y], [320 - bottom_x, 240 + bottom_y]])
  
  #selecting 4 points from image that will be transformed
  pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])
  
  #finding homography matrix
  h, status = cv2.findHomography(pts_src, pts_dst)
  
  #homography process
  cv_img_homography = cv2.warpPerspective(cv_img_input, h, (1000, 600))
  
  #fill the empty space with black triangles on left and right side of bottom
  triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
  triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
  black = (0, 0, 0)
  white = (255, 255, 255)
  cv_img_homography = cv2.fillPoly(cv_img_homography, [triangle1, triangle2], black)
  
  #Publish the bird-eye view image
  pub_img_proj.publish(cvBridge.cv2_to_compressed_imgmsg(cv_img_homography, "jpg"))
  
  #Find red obstacles in the image (get the indices of red pixels in the image)
  Blue = cv_img_homography[:,:,0]
  Green = cv_img_homography[:,:,1]
  Red = cv_img_homography[:,:,2]
  obs = np.transpose(np.array(np.where((Blue>=85) & (Blue<=95) & (Green>=85) & (Green<=95) & (Red>=145) & (Red<=155))))
  
  #Get coordinates of red tape obstacles relative to robot frame "from pixels to meters"
  obs = obs.astype(float)
  obs[:,0] = 0.54 - (obs[:,0]*0.0004)
  obs[:,1] = -(obs[:,1]-500)*0.00032
  
  #Get coordinates of red tape obstacles relative to gazebo origin using a transformation matrix
  if Pose != []:
    #Apply rotation
    rot_matrix = np.array([[cos(Pose[2]),-sin(Pose[2])],[sin(Pose[2]),cos(Pose[2])]])
    obs[:] = obs.dot(rot_matrix.T)
    
    #Apply translation
    obs[:,0] = obs[:,0] + Pose[0]
    obs[:,1] = obs[:,1] + Pose[1]
    
    #Prepare the visual obstacle message and publish it "Format: [x1,y1,x2,y2,...,xN,yN]"
    obs = np.reshape(obs, (len(obs)*2), order='C').tolist()
    Vobs.data = obs
    
    #Publish the visual obstacle message
    pub_obs.publish(Vobs)

#Node initialization
rospy.init_node('img_proj_debi')
rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, img_callback) 
rospy.Subscriber('/odom',Odometry,pose_callback)
pub_img_proj = rospy.Publisher('/camera/image_output/compressed', CompressedImage, queue_size=1) #Bird-eye view image publisher
pub_obs = rospy.Publisher('/vscan', Float32MultiArray, queue_size=1) #Visual scan publisher

#Looping
rospy.spin()
