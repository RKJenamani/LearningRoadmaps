import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np

def load_eepositions(s_file_addr, g_file_addr, markerArray):

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
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
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
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
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

def main():
    count = 0
    topic = 'start_goal_pos'
    publisher = rospy.Publisher(topic, MarkerArray)

    rospy.init_node('start_goal_pos')

    markerArray = MarkerArray()

    s_file_addr = "data_vis/start_node.txt"
    g_file_addr = "data_vis/goal_node.txt"
    
    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        eepositions, markerArray = load_eepositions(s_file_addr, g_file_addr, markerArray)
        print("len(eepositions) = ",len(eepositions))
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