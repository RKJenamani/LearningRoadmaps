import netoworkx as nx
import numpy as np

def get_start_goal(envdir):
	pass

def path_exists(shallow_G, s_node, g_node):
	pass

def main():
	parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    parser.add_argument('--envdir',type=str,required=True)
    args = parser.parse_args()

    shallow_G = nx.read_graphml(args.graphfile)

    start, goal = get_start_goal(args.envdir)
    cond = get_cond(args.envdir)

    count = 0
    for i in range(len(start)):
    	s = start[i]
    	g = goal[i]

    	s_node, g_node = add_start_goal(shallow_G, s, g, K)
    	check = path_exists(shallow_G, s_node, g_node)

    	if(check==False):
    		count++
    print("shallow_G failed in "+`count`+" no of cases")		

if __name__ == '__main__':
	main()