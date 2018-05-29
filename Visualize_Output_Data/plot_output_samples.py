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

table_pose = np.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

box_pose = np.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

from catkin.find_in_workspaces import find_in_workspaces

package_name = 'pr_ordata'
directory = 'data'
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project=package_name,
    path=directory,
    first_match_only=True)
if len(objects_path) == 0:
    print('Can\'t find directory %s/%s' % (package_name, directory))
    sys.exit()
else:
    print objects_path # for me this is '/home/USERNAME/catkin_workspaces/herb_ws/src/pr-ordata/data/objects'
    objects_path = objects_path[0]

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

def get_table_box_pose(condnsfile):
    t = np.loadtxt(condnsfile)
    print("t = ", t)
    return t[3], t[7], t[19], t[23], t[27]

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

def get_expected_node_nos(path_node_file_addr):
    all_path_nodes = []
    with open(path_node_file_addr, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip('\n')
            if(not line=='-1'):
                nodes = line.split(",")
                for node in nodes:
                    all_path_nodes.append(node)
    return all_path_nodes
    
def get_ee_positions_from_node_no(G, env, robot, all_path_nodes):
    eepositions = []
    for node in all_path_nodes:
        state = G.node[node]['state']
        strlist = state.split()
        val_list = [float(s) for s in strlist]
        config = np.array(val_list)
        robot.SetActiveDOFValues(config)
        ee_trans = robot.right_arm.GetEndEffectorTransform()
        trans = ee_trans[0:3,3]
        eepos = trans.tolist()
        eepositions.append(eepos)
    return eepositions    

def get_markers_expected_samples(G, env, robot, path_node_file_addr):
    all_path_nodes = get_expected_node_nos(path_node_file_addr)
    eepositions = get_ee_positions_from_node_no(G, env, robot, all_path_nodes)
    markerArray = MarkerArray()
    print("len(eepositions) m1 = ", len(eepositions))
    print(eepositions)
    raw_input("all Ok!!")
    markerArray1 = MarkerArray()
    eepositions, markerArray1 = load_marker(eepositions, markerArray1, c = [0.0, 1.0, 0.0])
    return markerArray1

def main():
    print("Working on 12")
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--condnsfile',type=str,required=True)
    parser.add_argument('--graphfile',type=str,required=True)

    args = parser.parse_args()
    G = nx.read_graphml(args.graphfile)
    
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)

    tall_white_box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    tall_white_box = env.ReadKinBodyXMLFile(tall_white_box_file)
    env.AddKinBody(tall_white_box)

    xpos, ypos, xpos1, ypos1, zpos1 = get_table_box_pose(args.condnsfile)
    pp_no = 5
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    box_pose[0,3] = xpos1
    box_pose[1,3] = ypos1
    box_pose[2,3] = zpos1

    table.SetTransform(table_pose)
    tall_white_box.SetTransform(box_pose)

    count = 0
    topic = 'output_node_pos'
    publisher = rospy.Publisher(topic, MarkerArray)

    topic = 'expected_node_pos'
    publisher1 = rospy.Publisher(topic, MarkerArray)

    rospy.init_node('output_node_pos')

    markerArray = MarkerArray()

    p_file_addr = "output_data/output_samples_12.txt"

    eepositions = get_eepositions_from_samples(env, robot, p_file_addr)

    path_node_file_addr = "output_data/expected_path_nodes_12.txt"
    
    markerArray1 = get_markers_expected_samples(G, env, robot, path_node_file_addr)
    
    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        eepositions, markerArray = load_marker(eepositions, markerArray)
        print("len(eepositions) = ",len(eepositions))
        id = 0
        for m in markerArray.markers:
           m.id = id
           id += 1
        # Publish the MarkerArray
        publisher.publish(markerArray)

        for m in markerArray1.markers:
           m.id = id
           id += 1
        publisher1.publish(markerArray1)
        # print("len(markerArray1) = ", len(markerArray1))
        count += 1

        rospy.sleep(0.1)


if __name__ == '__main__':
    main()