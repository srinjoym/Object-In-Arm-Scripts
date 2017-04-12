#!/usr/bin/env python

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians, atan
from os import listdir
import sys
import rospy
import geometry_msgs.msg
from std_msgs.msg import *
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
from control_msgs.msg import *
from vector_msgs.msg import *
import time
import requests
import tf
import tf_conversions
from arm_moveit import *
from utils import *
import random

class CaptureImages:

  def __init__(self):
    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    rospy.sleep(2)
    self.arm = ArmMoveIt()
    self.markerArray = MarkerArray()
    self.current_execution = 1
    self.lin_act_state = control_msgs.msg.JointTrajectoryControllerState()
    self.file_name=""

  def publish_point(self, pose,color):
    self.markerArray = generate_point(self.markerArray,pose,color)
    

  def log(self,pose):

      with open(self.file_name, 'a+') as f:
        f.write("\nPicture %i"%int(self.get_next_pic())+"\nPosition\n"+pose.pose.position+"\nOrientation\n"+pose.pose.orientation+"\n"+"\nTime\n"+pose.header.stamp)
      with open(self.file_name, 'a+') as f:
            f.write("*************************************")

  def generate_point(self):
    return self.arm.group[0].get_random_pose("right_ee_link")

  def execute_movement(self):

    ik = self.arm.get_IK(self.generate_point().pose)
    plan = self.arm.plan_jointTargetInput(ik)

    if(plan!=None):
      self.arm.group[0].execute(plan)

    pass

  def capture_position(self):

    self.log(self.arm.group[0].get_current_pose())
      


def main():
  
  capture_img = CaptureImages()

  ## ask if integrate object scene from code or not
  
  if not rospy.is_shutdown():
    #r = requests.get("http://10.5.5.9/gp/gpControl/command/shutter?p=1")

    ##   Assigned tarPose the current Pose of the robotlp 
    #tarPose = arm.group[0].get_current_pose().pose
    #arm.auto_circle(0.57,0.35,[1.3,0,-0.35])
    last_file = 0
    for file_name in listdir("../data"):
      if(int(file_name[0:len(file_name)-4])>last_file):
        last_file = int(file_name[0:len(file_name)-4])

    capture_img.file_name="../data/%i.txt"%(last_file+1)

    with open(capture_img.file_name, 'w+') as f:
      pass
    while not rospy.is_shutdown():
      capture_img.execute_movement()
      capture_img.capture_position()

    

    
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  
  main()
