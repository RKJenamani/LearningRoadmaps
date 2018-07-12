- To create different planning problem dataset
- Collision checking using herbpy API

## Structure of the data generated:

One directory for each task.
  In each task directory there is one subdirectory for each environment setup containing:  
  start_node.txt  
  goal_node.txt  
  conditions.txt  
  binary_vec.txt  

# Generating dataset:
- Create a folder with name "dataset" and run the following command  
taskid: "T1" : Given a table and tall block in the environment, to move the stick from below to above the table  
taskid: "T2" : Given a table and tall block in the environment, to move the stick across the block  
- python generate_dataset.py --graphfile graphs/modified_graph_8D30000_3lacs.graphml --datadir dataset --taskid T1