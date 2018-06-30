import os
import argparse
import networkx as nx
import math        
import numpy as np
import herbpy
import helper
from itertools import islice, chain 

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

def get_path_nodes(shallow_G_currP, dense_G, start_n, goal_n, env, robot, stick):
    temp_path_nodes = nx.dijkstra_path(dense_G, start_n, goal_n)
    path_nodes = []

    prev = curr = 'o'+start_n
    prev_posn = curr_posn = helper.state_to_numpy(shallow_G_currP.node[curr]['state'])
    
    prev_temp = curr_temp = start_n
    prev_temp_posn = curr_temp_posn = helper.state_to_numpy(dense_G.node[curr_temp]['state'])

    i = 0
    while(i<len(temp_path_nodes)-1):
        i += 1

        curr_temp = temp_path_nodes[i]
        curr_temp_posn = helper.state_to_numpy(dense_G.node[curr_temp]['state'])

        curr = helper.find_closest_node(shallow_G_currP, curr_temp_posn)
        curr_posn = helper.state_to_numpy(shallow_G_currP.node[curr]['state'])

        if(curr==prev):
            continue

        if(helper.satisfy_condition(curr_posn, curr_temp_posn, env, robot, stick)):
            if(helper.satisfy_condition(curr_posn, prev_posn, env, robot, stick)):
                prev_temp = curr_temp
                prev_temp_posn = curr_temp_posn
                prev = curr
                prev_posn = curr_posn
                continue
            elif(helper.satisfy_condition(prev_posn, curr_temp_posn, env, robot, stick)):
                path_nodes.append(curr_temp)
                prev_temp = curr_temp
                prev = 'o'+curr_temp
                shallow_G_currP.add_node(prev, state = dense_G.node[curr_temp]['state'])
                prev_temp_posn = prev_posn = curr_temp_posn
                continue
            else:
                path_nodes.append(prev_temp)
                path_nodes.append(curr_temp)
                shallow_G_currP.add_node('o'+prev_temp, state = dense_G.node[prev_temp]['state'])
                shallow_G_currP.add_node('o'+curr_temp, state = dense_G.node[curr_temp]['state'])
                prev_temp = curr_temp
                prev = 'o'+curr_temp

                prev_temp_posn = prev_posn = curr_temp_posn
                continue
        else:
            if(helper.satisfy_condition(prev_posn, curr_temp_posn, env, robot, stick)):
                path_nodes.append(curr_temp)
                prev_temp = curr_temp
                prev = 'o'+curr_temp
                shallow_G_currP.add_node(prev, state = dense_G.node[curr_temp]['state'])
                prev_temp_posn = prev_posn = curr_temp_posn
                continue
            else:
                path_nodes.append(prev_temp)
                path_nodes.append(curr_temp)
                shallow_G_currP.add_node('o'+prev_temp, state = dense_G.node[prev_temp]['state'])
                shallow_G_currP.add_node('o'+curr_temp, state = dense_G.node[curr_temp]['state'])
                prev_temp = curr_temp
                prev = 'o'+curr_temp

                prev_temp_posn = prev_posn = curr_temp_posn
                continue

    assert (len(path_nodes)>0)          
    return path_nodes

#return RF path nodes
def get_RF_paths(shallow_G_currP, dense_G, source, target, env, robot, stick):

    try:
        nx.dijkstra_path(dense_G, source, target)
    except:
        raise Exception("Dense Graph doesn't have path")
    shallow_G_currP.add_node('o'+source, state = dense_G.node[source]['state'])
    shallow_G_currP.add_node('o'+target, state = dense_G.node[target]['state'])

    # print(" debug: calling get_path_nodes")
    nodes = get_path_nodes(shallow_G_currP, dense_G, source, target, env, robot, stick)
    # print(" debug: nodes = ", nodes)
    return [list(set(nodes))]
    # return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))

def remove_invalid_edges(G1, binary_vec):
    to_remove = []
    
    for bv in binary_vec:
        if(not bv[0]):
            u, v = str(int(bv[1])), str(int(bv[2]))
            to_remove.append((u, v))

    for r in to_remove:
        try:
            G1.remove_edge(r[0], r[1]) 
        except:
            pass
        # print("removing edge = ", r[0], r[1])       
    return G1

def write_to_file(directory, all_paths):
    # print(all_paths)
    # for i in range(len(all_paths)):  
    #     print(i,all_paths[i])
    with open(directory + "/RF_path_nodes.txt", 'w') as file:
        file.writelines(','.join(str(j) for j in i) + '\n' for i in all_paths)

def process_it(shallow_G_currP, dense_G, directory, env, robot, stick, table, box):
    print(" debug: processing directory ", directory)
    shallow_G_currP1 = shallow_G_currP.copy()
    dense_G1 = dense_G.copy()

    start = np.loadtxt(directory+"/start_node.txt")
    goal = np.loadtxt(directory+"/goal_node.txt")
    binary_vec = np.loadtxt(directory+"/binary_vec.txt")
    conditions = np.loadtxt(directory+"/conditions.txt")
    shallow_G_currP1 = helper.remove_invalid_edges(shallow_G_currP1, env, robot, stick)
    dense_G1 = remove_invalid_edges(dense_G1, binary_vec)

    table_pose = conditions[:16].reshape(4,4)
    box_pose = conditions[16:].reshape(4,4)

    table.SetTransform(table_pose)
    box.SetTransform(box_pose)
    
    all_paths = []
    found_path = 0
    for i in range(50):
        src = str(int(start[i]))
        gl = str(int(goal[i]))
        paths = []
        try:
            paths = get_RF_paths(shallow_G_currP1, dense_G1, src, gl, env, robot, stick)
            paths = list(chain.from_iterable(paths))
            all_paths.append(paths)
            found_path += 1 
        except Exception as e:
            print(" debug: ",e)
            all_paths.append(['-1'])
    print("for dir = ", directory)
    print("found_path = ", found_path)
    # print(all_paths) 
    # print("\n\n\n\n") 
    # print(len(all_paths))      
    write_to_file(directory, all_paths)  
    print("written to directory = ",directory)         

def list_all_dir(data_dir):
    task_dirs = os.listdir(data_dir)

    list_dir = []
    for task_dir in task_dirs:
        env_dirs = os.listdir(data_dir+"/"+task_dir)
        for env_dir in env_dirs:
            list_dir.append(data_dir +"/"+ task_dir +"/"+ env_dir)
    return list_dir

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--datadir',type=str,required=True)
    args = parser.parse_args()

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()

    flat_base_file = os.path.join(objects_path,'objects/flat_base.kinbody.xml')
    flat_base = env.ReadKinBodyXMLFile(flat_base_file)
    env.AddKinBody(flat_base)
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)
    box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    box = env.ReadKinBodyXMLFile(box_file)
    env.AddKinBody(box)
    stick_file = os.path.join(objects_path,'objects/stick.kinbody.xml')
    stick = env.ReadKinBodyXMLFile(stick_file)
    env.AddKinBody(stick)

    base_pose = np.array([[  3.29499984e-03,  -5.97027617e-08,   9.99994571e-01,
          7.83268307e-01],
       [  9.99994571e-01,  -5.95063642e-08,  -3.29499984e-03,
         -2.58088849e-03],
       [  5.97027617e-08,   1.00000000e+00,   5.95063642e-08,
          1.19378528e-07],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])
    base_pose[:3,3] = 0,0,-0.04
    flat_base.SetTransform(base_pose)

    dense_G = nx.read_graphml(args.graphfile)
    shallow_G = nx.read_graphml("graphs/halton8D3000_0.graphml")
    data_dir = args.datadir

    directories = list_all_dir(data_dir)
    print(directories)

    for directory in directories:
        process_it(shallow_G, dense_G, directory, env, robot, stick, table, box)
