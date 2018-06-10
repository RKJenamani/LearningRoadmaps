import networkx as nx 
import numpy as np 
import herbpy
import os
import argparse
from math import sqrt

EDGE_DISCRETIZATION = 7

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

def get_table_pose(condnsfile):
    t = np.loadtxt(condnsfile)
    print("t = ", t)
    return t[3], t[7], t[19], t[23], t[27]

def edge_to_configs(state1, state2):
    config1 = state_to_numpy(state1)
    config2 = state_to_numpy(state2)

    diff = config2 - config1
    step = diff/EDGE_DISCRETIZATION

    to_check = list()
    to_check.append(config1)

    for i in xrange(EDGE_DISCRETIZATION - 1):
        conf = config1 + step*(i+1)
        to_check.append(conf)

    return to_check

def find_shortest_paths(G, source, target, weight=None):
    path_nodes = (nx.dijkstra_path(G, source, target, weight))
    # print("path_nodes = ", path_nodes)
    # print("path_nodes[0] = ", path_nodes[0])
    
    config1 = state_to_numpy(G.node[path_nodes[0]]['state'])
    path_length = 0
    for i in range(1, len(path_nodes)):
        state = G.node[path_nodes[i]]['state']
        config2 = state_to_numpy(state)
        path_length += sqrt(np.sum((config2-config1)**2))
    return path_nodes, path_length   

def remove_invalid_edges(G, env, robot):
    # print("total no of edges = ", len(list(G.edges())))
    to_remove = []
    for edge in G.edges():
        u, v = edge
        state1 = G.node[u]['state']
        state2 = G.node[v]['state']
        configs_to_check = edge_to_configs(state1,state2)

        edge_free = 1
        for cc in configs_to_check:
            robot.SetActiveDOFValues(cc)
            if env.CheckCollision(robot) == True:
                edge_free = 0
                break
        if(not edge_free):
            to_remove.append((u, v))

    for r in to_remove:
        G.remove_edge(r[0], r[1])

    # print("no of edges in collision = ", len(to_remove))    

    return G       

def main():
    parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--outputgraphfile',type=str,required=True)
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--envdir',type=str,required=True)
    args = parser.parse_args()
    print("Analysing "+args.envdir)

    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    tall_white_box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)
    tall_white_box = env.ReadKinBodyXMLFile(tall_white_box_file)
    env.AddKinBody(tall_white_box)

    G = nx.read_graphml(args.outputgraphfile)
    orig_G = nx.read_graphml(args.graphfile)
    xpos, ypos, xpos1, ypos1, zpos1 = get_table_pose(args.envdir + "/conditions.txt")
    table_pose[0,3] = xpos
    table_pose[1,3] = ypos

    table.SetTransform(table_pose)

    box_pose[0,3] = xpos1
    box_pose[1,3] = ypos1
    box_pose[2,3] = zpos1
    tall_white_box.SetTransform(box_pose)

    G = remove_invalid_edges(G, env, robot)
    orig_G = remove_invalid_edges(orig_G, env, robot)

    start = np.loadtxt(args.envdir + "/start_node.txt")
    goal  = np.loadtxt(args.envdir + "/goal_node.txt")

    found_path = 0
    total_path_length = 0
    for i in range(50):
        src = str(int(start[i]))
        gl  = str(int( goal[i]))
        try:
            path, path_length = find_shortest_paths(G, src, gl, 'weight')
            print(path_length)
            found_path += 1
            total_path_length += path_length
        except Exception as e:
            print(e)
            print("No path between "+src+" and "+gl) 

    print("found_path = ", found_path)   
    if(not found_path == 0):
        print("Average path_length = ", total_path_length/found_path)

    found_path = 0
    total_path_length = 0    
    for i in range(50):
        src = str(int(start[i]))
        gl  = str(int( goal[i]))
        try:
            path, path_length = find_shortest_paths(orig_G, src, gl, 'weight')
            print(path_length)
            found_path += 1
            total_path_length += path_length
        except Exception as e:
            # print(e)
            print("No path between "+src+" and "+gl) 

    print("orig found_path = ", found_path)   
    print("orig Average path_length = ", total_path_length/found_path)        

def main1():
    file = open("Results_Uniform.txt", "w")
    parser = argparse.ArgumentParser(description='Generate environments')
    # parser.add_argument('--outputgraphfile',type=str,required=True)
    parser.add_argument('--graphfile',type=str,required=True)
    # parser.add_argument('--envdir',type=str,required=True)
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.right_arm.SetActive()
    # Load table from pr_ordata
    table_file = os.path.join(objects_path,'objects/table.kinbody.xml')
    tall_white_box_file = os.path.join(objects_path,'objects/tall_white_box.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_file)
    env.AddKinBody(table)
    tall_white_box = env.ReadKinBodyXMLFile(tall_white_box_file)
    env.AddKinBody(tall_white_box)
    args = parser.parse_args()

    e_dir = ["test_data_9June/T1/2", "test_data_9June/T1/4", "test_data_9June/T2/2", "test_data_9June/T2/10", "test_data_9June/T2/14"]
    
    y = {}

    for envdir in e_dir:
        y[str(envdir)] = {}
        print("found_path Analysing "+envdir)
        file.write("Analysing "+envdir+"\n")
        orig_G = nx.read_graphml(args.graphfile)
        xpos, ypos, xpos1, ypos1, zpos1 = get_table_pose(envdir + "/conditions.txt")
        table_pose[0,3] = xpos
        table_pose[1,3] = ypos

        table.SetTransform(table_pose)

        box_pose[0,3] = xpos1
        box_pose[1,3] = ypos1
        box_pose[2,3] = zpos1
        tall_white_box.SetTransform(box_pose)
        K = ["5", "10", "15"]
        n_samples = ["200", "300", "400", "500"]

        # orig_G = remove_invalid_edges(orig_G, env, robot)
        start = np.loadtxt(envdir + "/start_node.txt")
        goal  = np.loadtxt(envdir + "/goal_node.txt")

        found_path = 0
        total_path_length = 0    
        # for i in range(50):
        #     src = str(int(start[i]))
        #     gl  = str(int( goal[i]))
        #     try:
        #         path, path_length = find_shortest_paths(orig_G, src, gl, 'weight')
        #         # print(path_length)
        #         found_path += 1
        #         total_path_length += path_length
        #     except Exception as e:
        #         continue
        #         print("No path between "+src+" and "+gl) 

        # print("halton, found_path: ",found_path, "apl: ",total_path_length/found_path)
        # file.write("Dense Graph: no_path: "+str(found_path)+" APL: "+str(total_path_length/found_path)+"\n") 
        z = 6
        for k in K:
            y[str(envdir)][str(k)] = {}
            for no_samples in n_samples:
                print("found_path output_samples_z"+str(z)+"_"+no_samples)
                outputgraphfile = envdir + "/output_graph_uniform"+no_samples+"_k"+k+".graphml"
                # outputgraphfile = envdir + "/output_graph_z"+str(z)+"_f_"+str(no_samples)+"_k"+str(k)+".graphml"
                print("outputgraphfile = ",outputgraphfile)

                G = nx.read_graphml(outputgraphfile)

                G = remove_invalid_edges(G, env, robot)

                found_path = 0
                total_path_length = 0
                for i in range(50):
                    src = str(int(start[i]))
                    gl  = str(int( goal[i]))
                    try:
                        path, path_length = find_shortest_paths(G, src, gl, 'weight')
                        # print(path_length)
                        found_path += 1
                        total_path_length += path_length
                    except Exception as e:
                        continue
                        print(e)
                        print("No path between "+src+" and "+gl) 

                if(found_path==0):
                    found_path = 1        
                print("envdir = ",envdir)    
                print("k: ",k, " found_path: ",found_path, " apl",total_path_length/found_path)
                file.write("k: "+str(k)+" paths_found = "+str(found_path)+" APL: "+str(total_path_length/found_path))

                y[str(envdir)][str(k)][str(no_samples)] = [found_path, total_path_length/found_path]
    print("Y = ",y)

if __name__ == '__main__':
            main1()      
            

