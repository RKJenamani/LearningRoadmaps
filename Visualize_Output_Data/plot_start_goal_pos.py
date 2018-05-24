import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
import networkx as nx
import argparse
import helper

def load_eepositions(G, s_file_addr, g_file_addr, markerArray):

    eepositions = {}
    i = 0
    with open(s_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            eepositions[str(i)] = np.float32(np.array(line.split(" ")))
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = eepositions[str(i)][0]
            marker.pose.position.y = eepositions[str(i)][1] 
            marker.pose.position.z = eepositions[str(i)][2]
            markerArray.markers.append(marker)
            i += 1
            if(i>10):
                break

    with open(g_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            eepositions[str(i)] = np.float32(np.array(line.split(" ")))
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = eepositions[str(i)][0]
            marker.pose.position.y = eepositions[str(i)][1] 
            marker.pose.position.z = eepositions[str(i)][2]
            markerArray.markers.append(marker)
            i += 1        
            if(i>20):
                break

    return eepositions, markerArray

def load_marker(G, s_eepositions, g_eepositions, markerArray):
    i = 0
    for e in s_eepositions:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1] 
            marker.pose.position.z = e[2]
            markerArray.markers.append(marker)
            i += 1

    for e in g_eepositions:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1] 
            marker.pose.position.z = e[2]
            markerArray.markers.append(marker)
            i += 1        

    return markerArray    

def main():
    print("-----For d6------")
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()
    
    G = nx.read_graphml(args.graphfile)

    s_file_addr = "temp_data/start_node-d6.txt"
    g_file_addr = "temp_data/goal_node-d6.txt"

    s_eepositions = helper.get_eepositions(G, s_file_addr, None)
    g_eepositions = helper.get_eepositions(G, g_file_addr, None)

    count = 0
    topic = 'start_goal_pos'
    publisher = rospy.Publisher(topic, MarkerArray)

    rospy.init_node('start_goal_pos')

    markerArray = MarkerArray()

    
    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        markerArray = load_marker(G, s_eepositions, g_eepositions, markerArray)

        print("len(eepositions) = ",len(s_eepositions) + len(g_eepositions))
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