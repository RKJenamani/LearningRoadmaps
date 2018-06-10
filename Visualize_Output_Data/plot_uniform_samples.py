import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
import helper
import networkx as nx
import argparse
import herbpy
import os

def get_eepositions_from_samples(env, robot, p_file_addr):
    eepositions = []
    with open(p_file_addr, 'r') as file:
        lines = file.readlines()
        for line in lines:
            state = line.strip("\n")
            strlist = state.split()
            val_list = [float(s) for s in strlist]
            config = np.array(val_list)
            print("config = ",config)
            robot.SetActiveDOFValues(config)
            ee_trans = robot.right_arm.GetEndEffectorTransform()
            trans = ee_trans[0:3,3]
            eepos = trans.tolist()
            eepositions.append(eepos)
    return eepositions

def load_marker(eepositions, markerArray, c = [0.0, 0.0, 1.0]):

    i = 0
    for e in eepositions:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = c[0]
            marker.color.g = c[1]
            marker.color.b = c[2]
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1] 
            marker.pose.position.z = e[2]
            markerArray.markers.append(marker)
            i += 1

    return eepositions, markerArray

def main():
    rospy.init_node('uniform_node_pos')
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--sample_file',type=str,required=True)
    args = parser.parse_args()
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    eepositions = get_eepositions_from_samples(env, robot, args.sample_file)
    markerArray = MarkerArray()
    eepositions, markerArray = load_marker(eepositions, markerArray)

    topic = 'uniform_node_pos'
    publisher = rospy.Publisher(topic, MarkerArray)
    id = 0
    for m in markerArray.markers:
           m.id = id
           id += 1

    while not rospy.is_shutdown():
        publisher.publish(markerArray)
        rospy.sleep(0.1)





if __name__ == '__main__':
            main()        