#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 10:40:18 2016

@author: Ahmad Zakaria
@author: Radu Alexandru
"""

import rospy
from rostopic import get_topic_type
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from tf.msg import tfMessage
from visualization_msgs.msg import Marker
import csv
import os



file_name="default.csv"

# timestamp tx ty tz qx qy qz qw

def String_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard String %s", data.data)
    
def Path_callback(data):
    print "path received"
    with open(file_name, 'ab') as f:
        writer = csv.writer(f, delimiter=' ')
        for posestamped in data.poses:
            writer.writerow([posestamped.header.stamp.secs,posestamped.pose.position.x,posestamped.pose.position.z,posestamped.pose.position.y,posestamped.pose.orientation.x,posestamped.pose.orientation.y,posestamped.pose.orientation.z,posestamped.pose.orientation.w])
        
    rospy.loginfo(rospy.get_caller_id() + "I heard Path of length: %s", len(data.poses))

def PointStamped_callback(data):
    print "PointStamped received"
    with open(file_name, 'ab') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerow([data.header.stamp.secs,data.point.x,data.point.z,data.point.y,0,0,0,0])
        
    rospy.loginfo(rospy.get_caller_id() + "I heard PointStamped ")

def Marker_callback(data):
    print "Marker received"
    with open(file_name, 'ab') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerow([data.header.stamp.secs,data.pose.position.x,data.pose.position.z,data.pose.position.y,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        
    rospy.loginfo(rospy.get_caller_id() + "I heard Marker ")

def PoseStamped_callback(data):
  
    with open(file_name, 'ab') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerow([data.header.stamp.secs,data.pose.position.x,data.pose.position.z,data.pose.position.y,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        
    rospy.loginfo(rospy.get_caller_id() + "I heard PoseStamped ")

def TransformStamped_callback(data):
  
    with open(file_name, 'ab') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerow([data.header.stamp.secs, data.transform.translation.x,data.transform.translation.z,data.transform.translation.y,data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z,data.transform.rotation.w])
        
    rospy.loginfo(rospy.get_caller_id() + "I heard TransformStamped ")
    
def tfMessage_callback(data):
    
    TransformStamped_callback(data.transforms[0])


def listener(tpath, ttype, cb):
    rospy.Subscriber(tpath, ttype, cb)
    
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('pyreader',anonymous=True)
    topic_path = rospy.get_param('~topic', 'error:invalid topic path')
    topic_type = get_topic_type(topic_path, blocking=True)
    file_name=  rospy.get_param('~outfile', 'error:invalid file path')
    print "file_name is", file_name
    print "path is " + topic_path
    print "type is " + topic_type[0]
    
    try:
        os.remove(file_name)
    except:
        pass
    
    ttype_list = {
    'nav_msgs/Path':Path,
    'std_msgs/String': String,
    'geometry_msgs/PointStamped': PointStamped,
    'visualization_msgs/Marker': Marker,
	'geometry_msgs/PoseStamped' : PoseStamped,
    'geometry_msgs/TransformStamped' : TransformStamped,
    'tf/tfMessage' : tfMessage
    }
    ttype = ttype_list.get(topic_type[0],String)
    
    tcallback_list = {
    Path: Path_callback,
    String: String_callback,
    PointStamped: PointStamped_callback,
    Marker: Marker_callback,
	PoseStamped : PoseStamped_callback,
    TransformStamped: TransformStamped_callback,
    tfMessage: tfMessage_callback
    }
    
    tcallback = tcallback_list.get(ttype,String_callback)
    print "Subscribing to: " + topic_path + ", with type: " + topic_type[0]    

    listener(topic_path,ttype,tcallback)
    