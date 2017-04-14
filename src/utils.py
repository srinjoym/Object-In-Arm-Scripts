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

    