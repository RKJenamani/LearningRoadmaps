import networkx as nx 
import numpy as np
import argparse

def remove_one_edge(G, src, goal):
	threshold = 10
	Edge_Count = {}
	pass


def get_robus_shortest_paths(G, src, goal, K):
	path_nodes_all = []
	for k in range(K):
		path_nodes_all.append([(nx.dijkstra_path(G, source, target, weight='weight'))])
		G = remove_one_edge(G, src, goal)
	return path_nodes_all	
		
def main():
	parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    G = nx.read_graphml(args.graphfile)

    src_nodes, goal_nodes = get_nodes()

    for i in range(len(src_nodes)):
    	src = src_nodes[i]
    	goal = goal_nodes[i]

    	path_nodes = get_robus_shortest_paths(G, src, goal, k)

if __name__ == '__main__':
	main()