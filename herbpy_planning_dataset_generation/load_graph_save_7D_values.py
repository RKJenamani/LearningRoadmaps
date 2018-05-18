import networkx as nx
import numpy as np 

def main():
	parser = argparse.ArgumentParser(description='Generate environments')
    parser.add_argument('--graphfile',type=str,required=True)
    args = parser.parse_args()

    G = nx.read_graphml(args.graphfile)
    

if __name__ == '__main__':
	main()