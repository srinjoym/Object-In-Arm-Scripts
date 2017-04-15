#!/usr/bin/env python
import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def generate_point(markerArray, pose,color):
    marker = Marker()
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g= color[1]
    marker.color.b = color[2]
    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z
    marker.pose.orientation.w = pose.orientation.w
    marker.pose.orientation.x = pose.orientation.x
    marker.pose.orientation.y = pose.orientation.y
    marker.pose.orientation.z = pose.orientation.z
    marker.header.frame_id = "/base_link"
    # print self.marker
    # markerArray = MarkerArray()
    markerArray.markers.append(marker)
    # print "marker array"
    
    id = 0
    for m in markerArray.markers:
      m.id = id
      # print str(id)+"\n"
      # print m.pose.position.x
      id += 1
      # self.markerArray.markers.append(m)
    # print self.markerArray

    return markerArray

def publish_box(markerArray,bounding_box_coordinates,bounding_box_scale):
    marker = Marker()
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = bounding_box_scale[0]
    marker.scale.y = bounding_box_scale[1]
    marker.scale.z = bounding_box_scale[2]
    marker.color.a = 0.1
    marker.color.b = 1
    marker.pose.position.x = bounding_box_coordinates[0]+(bounding_box_scale[0]/2.0)
    marker.pose.position.y = bounding_box_coordinates[1]+(bounding_box_scale[1]/2.0)
    marker.pose.position.z = bounding_box_coordinates[2]+(bounding_box_scale[2]/2.0)
    marker.pose.orientation.w = 1
    marker.header.frame_id = "/base_link"
    # print self.marker
    # markerArray = MarkerArray()
    markerArray.markers.append(marker)
    # print "marker array"
    
    id = 0
    for m in markerArray.markers:
      m.id = id
      id += 1

    #self.publisher.publish(self.markerArray)
    return markerArray

def util_log(file_name,pose,id,status):
      if (status):
        with open(file_name, 'a+') as f:
          f.write("\nSuccess Picture %i"%id+"\nPosition\n"+str(pose.pose.position)+"\nOrientation\n"+str(pose.pose.orientation)+"\n")
      else:
        with open(file_name, 'a+') as f:
          f.write("\nFailed Picture %i"%id+"\nPosition\n"+str(pose.pose.position)+"\nOrientation\n"+str(pose.pose.orientation)+"\n")
      with open(file_name, 'a+') as f:
            f.write("*************************************")