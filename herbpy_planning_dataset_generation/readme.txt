ee_pos_enum_nodes and DOF_values_enum_nodes are in order of items appearing when calling G.nodes()

So, to search for eePos of a specific node say n ( => G.node[n] ), search at G.nodes().index(n) in eePos array

same for start and goal nodes