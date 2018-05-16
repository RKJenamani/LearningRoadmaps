import networkx as nx
import math        
import numpy as np
from math import sqrt

def connect_knn(G, k):
    pass

def generate_graph(nodes): #nodes -> key, value pair, value => string with joint angles separated by ' '
    G = nx.Graph()
    for key in nodes:
        value = str(nodes[str(key)]) 
        G.add_node(str(key), state = value) 
    return G

def get_shortest_path_length(G, start, goal):
    start_node, goal_node = find_closet_node(start, goal)
    path_length = nx.dijkstra_path_length(g, str(start_node), str(goal_node), 'weight')
    return path_length

def calc_weight(s, g):
    return sqrt(np.sum((s-g)**2))

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list)           

def create_weighted_graph(G):
    for i, edge in enumerate(G.edges()):
        u,v = edge
        state1 = state_to_numpy(G.node[u]['state'])
        state2 = state_to_numpy(G.node[v]['state'])
        G[u][v]['weight'] = calc_weight(state1, state2)
    nx.write_graphml(G, "graphs/weighted_graph.graphml")
    return G

def load_output_samples(file_addr):
    nodes = {}
    i = 0
    with open(file_addr, 'r') as file:
        lines  = file.readlines()
        for line in lines:
            line = line.strip('\n')
            nodes[str(i)] = line
            i += 1
    print(nodes)
    return nodes        

#take ee_transform and plot it
def plot_end_effector_positions(nodes):
    pass

def main():
    K = 5
    file_addr = "output_sample_node_e9.txt"
    nodes = load_output_samples(file_addr)
    G = generate_graph(nodes)
    # G = create_weighted_graph(G)
    G = connect_knn(G, K)
    path_length = get_shortest_path_length(G, start_conf, goal_conf)

if __name__ == '__main__':
            main()      