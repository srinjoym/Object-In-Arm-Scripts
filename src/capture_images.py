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
import std_srvs.srv
import random
import json

class CaptureImages:

  def __init__(self):
    #rospy.wait_for_service('image_saver/save')
    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    rospy.sleep(2)
    self.arm = ArmMoveIt()
    self.markerArray = MarkerArray()
    self.current_execution = 1
    self.lin_act_state = control_msgs.msg.JointTrajectoryControllerState()
    self.file_name=""
    self.bounding_box_scale = [0.4,0.6,0.55]
    self.bounding_box_coordinates = [0.7,-0.3,0.9] #relative to base link
    self.bounding_box_mps = [self.bounding_box_coordinates[0]+(self.bounding_box_scale[0]/2),
                            self.bounding_box_coordinates[1]+(self.bounding_box_scale[1]/2),
                            self.bounding_box_coordinates[2]+(self.bounding_box_scale[2]/2)
                            ]
    self.num_points = [3,3,3]
    self.failed_points_new = []
    self.failed_points_old = []

  def publish_point(self, pose,color):
    self.markerArray = generate_point(self.markerArray,pose,color)
    self.publisher.publish(self.markerArray)
    
  def log(self,pose,id,status):
    util_log(self.file_name,pose,id,status)

  def calc_orientation(self,angle,tilt_angle,rotation):
    quaternion = tf.transformations.quaternion_from_euler(radians(angle),radians(-90), 0)
    quaternion2 = tf.transformations.quaternion_from_euler(0,0, radians(rotation))
    quaternion = tf.transformations.quaternion_multiply(quaternion,tf.transformations.quaternion_inverse(quaternion2))
    return quaternion
  
  def is_failure(self,plan):
    return plan is None or not plan.joint_trajectory.header.frame_id
        
  def capture_position(self,id,status):

    self.log(self.arm.group[0].get_current_pose(),id,status)

  def get_failed(self):
    with open('failed_points.json') as json_data:
      self.failed_points_old = json.load(json_data)

  def dump_failed(self):
    
    with open('failed_points.json', 'w') as outfile:
      json.dump(self.failed_points_new, outfile)
    print self.failed_points_new

  def add_failed(self):
    with open('failed_points.json') as json_data:
      failed_points_old = json.load(json_data)
    for point in self.failed_points_new:
      if point not in failed_points_old:
        failed_points_old.append(point)
    with open('failed_points.json', 'w') as outfile:
      json.dump(failed_points_old, outfile)

  def capture_image(self):
    # try:
    #   image = rospy.ServiceProxy('image_saver/save', std_srvs.srv.Empty)
    #   image()
    # except rospy.ServiceException, e:
    #   print "Service call failed: %s"%e
    pass

  def execute_movement(self):
    counter = 0
    image_counter = 0
    depth_ct = -1
    
    self.publisher.publish(publish_box(self.markerArray,self.bounding_box_coordinates,self.bounding_box_scale))
    
    x_limit = self.bounding_box_coordinates[0] + self.bounding_box_scale[0]
    x = self.bounding_box_coordinates[0]
    x_increment = (self.bounding_box_scale[0]/(self.num_points[0]-1))
    while(x<x_limit+0.03):

        depth_ct += 1
        scaling_factor = (-0.5*depth_ct)+2

        z_increment = (self.bounding_box_scale[2]/(self.num_points[2]-1))/scaling_factor
        z = self.bounding_box_mps[2]-z_increment*((self.num_points[2])/2)
        z_limit = self.bounding_box_mps[2]+z_increment*((self.num_points[2])/2)
        
        while(z<z_limit+0.03):

          y_increment = (self.bounding_box_scale[1]/(self.num_points[1]-1))/scaling_factor
          y = self.bounding_box_mps[1]-y_increment*((self.num_points[1])/2)
          y_limit = self.bounding_box_mps[1]+y_increment*((self.num_points[1])/2)
          
          while(y<y_limit+0.03):
            print "y %f, y inc %f, y lim %f, y mid %f"%(y,y_increment,y_limit,self.bounding_box_mps[1])
            for rotation in range(-20,21,20):
            #for rotation in range(0,1,20):
              for angle in range(-45,46,45):
              #for angle in range(0,1,45):
                if(counter not in self.failed_points_old):
                  
                  quaternion = self.calc_orientation(angle,0,rotation)
                  pose = geometry_msgs.msg.Pose()
                  pose.position.x = x
                  pose.position.y = y
                  pose.position.z = z
                  pose.orientation.x = quaternion[0]
                  pose.orientation.y = quaternion[1]
                  pose.orientation.z = quaternion[2]
                  pose.orientation.w = quaternion[3]
                    
                  ik = self.arm.get_IK(pose)
                  plan = self.arm.plan_jointTargetInput(ik)
                  #print plan
                  if(not self.is_failure(plan)):
                    print "executing pose"
                    
                    
                    print pose
                    self.publish_point(pose,[0,1,0])
                    self.capture_position(image_counter,True)
                    #self.arm.group[0].execute(plan)
                    image_counter+=1
                    self.capture_image()
                  else:
                    #self.capture_position(counter,False)
                    self.publish_point(pose,[1,0,0])
                    self.failed_points_new.append(counter)
                    print "failed pose"
                    print pose
                    #image_counter+=1
                  
                counter+=1
                
            y+=y_increment
          z+=z_increment
        x+=x_increment

  


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
    if not rospy.is_shutdown():
      capture_img.get_failed()
      capture_img.execute_movement()
      capture_img.add_failed()
      #capture_img.dump_failed()
      #capture_img.capture_position()

    

    
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  
  main()
