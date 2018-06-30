## To Visualize 8D Samples

- Copy the dataset to this directory
- python plot_path_nodes.py --graphfile graphs/xyz.graphml --envdir dataset/TX/Y (To publish markers corresponding to path_nodes and start, goal pair)
- python load_herb_check_conf.py --graphfile graphs/xyz.graphml --envdir dataset/TX/Y (To set HERB conf one by one from start to goal following each path nodes)  
Make sure same planning problem (pp_no) is set in both the files (check main function)