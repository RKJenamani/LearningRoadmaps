import networkx as nx 
import numpy as np
import argparse 
from math import sqrt

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def generate_graph(nodes): #nodes -> key, value pair, value => string with joint angles separated by ' '
    G = nx.Graph()
    for key in nodes:
        value = str(nodes[str(key)]) 
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
    e_dir = [ "test_data_9June/T2/4", "test_data_9June/T2/10", "test_data_9June/T2/14"]
    # e_dir = ["final_test_data/T2/20"]

    args = parser.parse_args()

    orig_G = nx.read_graphml(args.graphfile)
    for envdir in e_dir:
        print("Checking "+envdir)

        for no_samples in n_samples:
            for k in K:
                print("no_samples = ",no_samples)
                print("k = ",k)

                s_node_file_addr = envdir + "/start_node.txt"
                g_node_file_addr = envdir + "/goal_node.txt"


                # nodes = load_output_samples(envdir+"/output_samples_z"+str(z)+"_n"+str(no_samples)+".txt", s_node_file_addr, g_node_file_addr, orig_G)
                print("loading output samples..")
                nodes = load_output_samples("uniform_samples/uniform_samples_"+str(no_samples)+".txt", s_node_file_addr, g_node_file_addr, orig_G)
                print("adding nodes to Graph..")
                G = generate_graph(nodes)
                print("connecting knn")
                G = connect_knn(G, int(k))
                print("no of edges = ",len(list(G.edges())))
                print("no of nodes = ",len(list(G.nodes())))
                # nx.write_graphml(G,envdir + "/output_graph_z"+str(z)+"_f_"+no_samples+"_k"+k+".graphml")
                nx.write_graphml(G,envdir + "/output_graph_uniform"+str(no_samples)+"_k"+k+".graphml")