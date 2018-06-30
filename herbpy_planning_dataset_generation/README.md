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
- Create a folder with name "dataset" (it should be the same as in write_one_dir function in generate_dataset_new.py)
- python generate_dataset_new.py --graphfile graphs/xyz.graphml (Change the task_id in main function for different env setup)
