import herbpy
import networkx as nx 
import numpy as np
import argparse 
from math import sqrt
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

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def get_table_pose(condnsfile):
    t = np.loadtxt(condnsfile)
    print("t = ", t)
    return t[3], t[7], t[19], t[23], t[27]

def generate_graph(nodes, env, robot): #nodes -> key, value pair, value => string with joint angles separated by ' '
    G = nx.Graph()
    for key in nodes:
        value = str(nodes[str(key)]) 
        cc = state_to_numpy(value)
        robot.SetActiveDOFValues(cc)

        if not env.CheckCollision(robot):
            G.add_node(str(key), state = value) 
    return G

def load_output_samples(file_addr, s_node_file_addr, g_node_file_addr, orig_G):
    nodes = {}
    i = 0
    with open(file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes["o"+str(i)] = line
            i += 1
    with open(s_node_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes[line] = orig_G.node[line]['state']
    with open(g_node_file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes[line] = orig_G.node[line]['state']                
    # print(nodes)
    return nodes

def connect_knn(G, K):
    for node in G.nodes():
        state = G.node[node]['state']
        conf = state_to_numpy(state)
        G1 = G.copy()

        for k in range(K):
            w = 1000000
            sn = None
            for node1 in G1.nodes():
                if(node == node1):
                    continue
                state1 = G1.node[node1]['state']
                conf1  = state_to_numpy(state1)
                if(calc_weight(conf, conf1) < w):
                    w = calc_weight(conf, conf1)
                    sn = node1

            # if(check_for_collision(node, sn)==1):
            G.add_edge(node, sn)
            # print("connected edge from ",node, " to ",sn)
            G[node][sn]['weight'] = w
            G1.remove_node(sn)
    return G

if __name__ == '__main__':
    K = 05

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    tall_white_box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)
    tall_white_box = env.ReadKinBodyXMLFile(tall_white_box_file)
    env.AddKinBody(tall_white_box)


    # parser = argparse.ArgumentParser(description='Evaluate Sample')
    # parser.add_argument('--graphfile',type=str,required=True)
    # parser.add_argument('--envdir',type=str,required=True)
    # args = parser.parse_args()
    # print("Checking "+args.envdir)
    # print("K = ",K)

    # s_node_file_addr = args.envdir + "/start_node.txt"
    # g_node_file_addr = args.envdir + "/goal_node.txt"

    # orig_G = nx.read_graphml(args.graphfile)

    # nodes = load_output_samples(args.envdir + "/output_samples_z6_500.txt", s_node_file_addr, g_node_file_addr, orig_G)
    # G = generate_graph(nodes)
    # G = connect_knn(G, K)
    # print("no of edges = ",len(list(G.edges())))
    # print("no of nodes = ",len(list(G.nodes())))
    # nx.write_graphml(G, "output_graph_test.graphml")

    K = ["5", "10", "15"]
    z = 6
    n_samples = ["200", "300", "400", "500"]
    parser = argparse.ArgumentParser(description='Evaluate Sample')
    parser.add_argument('--graphfile',type=str,required=True)
    # parser.add_argument('--envdir',type=str,required=True)
    e_dir = ["test_data_11June/T2/4", "test_data_11June/T2/7", "test_data_11June/T2/14"]
    # e_dir = ["temp_data/T2/5"]

    args = parser.parse_args()

    orig_G = nx.read_graphml(args.graphfile)
    for envdir in e_dir:
        print("Checking "+envdir)

        xpos, ypos, xpos1, ypos1, zpos1 = get_table_pose(envdir + "/conditions.txt")
        table_pose[0,3] = xpos
        table_pose[1,3] = ypos

        table.SetTransform(table_pose)

        box_pose[0,3] = xpos1
        box_pose[1,3] = ypos1
        box_pose[2,3] = zpos1
        tall_white_box.SetTransform(box_pose)
        for no_samples in n_samples:
            for k in K:
                print("no_samples = ",no_samples)
                print("k = ",k)

                s_node_file_addr = envdir + "/start_node.txt"
                g_node_file_addr = envdir + "/goal_node.txt"


                # nodes = load_output_samples(envdir+"/output_samples_z"+str(z)+"_n"+str(no_samples)+".txt", s_node_file_addr, g_node_file_addr, orig_G)
                # print("loading output samples..")
                nodes = load_output_samples("uniform_samples/uniform_samples_"+str(no_samples)+".txt", s_node_file_addr, g_node_file_addr, orig_G)
                print("adding nodes to graph")
                G = generate_graph(nodes, env, robot)
                print("connecting knn")
                G = connect_knn(G, int(k))
                print("no of edges = ",len(list(G.edges())))
                print("no of nodes = ",len(list(G.nodes())))
                nx.write_graphml(G,envdir + "/output_graph_z"+str(z)+"_n"+no_samples+"_k"+k+".graphml")
                # nx.write_graphml(G,envdir + "/output_graph_uniform"+str(no_samples)+"_k"+k+".graphml")