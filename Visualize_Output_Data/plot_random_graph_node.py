import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
import helper
import networkx as nx
import argparse
from random import choice

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list) 

def get_random_graph_nodes(G):
    eepositions = []
    for i in range(300):
        node = choice(list(G.nodes()))
        temp = G.node[node]['eePos']
        eepos = state_to_numpy(temp)
        eepositions.append(eepos)
    return eepositions    

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
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1] 
            marker.pose.position.z = e[2]
            markerArray.markers.append(marker)
            i += 1

    return eepositions, markerArray

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()
    
    G = nx.read_graphml(args.graphfile)

    count = 0
    topic = 'random_graph_node_pos'
    publisher = rospy.Publisher(topic, MarkerArray)

    rospy.init_node('random_graph_node_pos')

    markerArray = MarkerArray()

    eepositions = get_random_graph_nodes(G)
    
    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        eepositions, markerArray = load_marker(eepositions, markerArray)
        print("len(eepositions) = ",len(eepositions))
        # print("eepos_start = ", eepos_start)
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