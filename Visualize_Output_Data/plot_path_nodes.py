import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
import helper
import networkx as nx
import argparse

def load_marker(eepositions, markerArray):

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
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1] 
            marker.pose.position.z = e[2]
            markerArray.markers.append(marker)
            i += 1

    return eepositions, markerArray

def append_start_goal(eepos_start, eepos_goal, markerArray):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = eepos_start[0]
    marker.pose.position.y = eepos_start[1] 
    marker.pose.position.z = eepos_start[2]
    markerArray.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = eepos_goal[0]
    marker.pose.position.y = eepos_goal[1] 
    marker.pose.position.z = eepos_goal[2]
    markerArray.markers.append(marker)  

    return markerArray  

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--envdir',type=str,required=True)
    args = parser.parse_args()
    
    G = nx.read_graphml(args.graphfile)

    count = 0
    topic = 'path_node_pos'
    publisher = rospy.Publisher(topic, MarkerArray)

    rospy.init_node('path_node_pos')

    markerArray = MarkerArray()

    p_file_addr = args.envdir + "/path_nodes.txt"
    s_node_no_file = args.envdir + "/start_node.txt"
    g_node_no_file = args.envdir + "/goal_node.txt"
    pp_no = 5

    eepositions = helper.get_eepositions(G, p_file_addr, pp_no)
    eepos_start = helper.get_eepositions(G, s_node_no_file, pp_no)[0]
    eepos_goal  = helper.get_eepositions(G, g_node_no_file, pp_no)[0]

    
    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        eepositions, markerArray = load_marker(eepositions, markerArray)
        markerArray = append_start_goal(eepos_start, eepos_goal, markerArray)
        print("len(eepositions) = ",len(eepositions))
        print("eepos_start = ", eepos_start)
        id = 0
        for m in markerArray.markers:
           m.id = id
           id += 1
        # Publish the MarkerArray
        publisher.publish(markerArray)

        count += 1

        rospy.sleep(0.1)


if __name__ == '__main__':
    main()