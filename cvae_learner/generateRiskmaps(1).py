# Import standard python libraries
import networkx as nx
import numpy

# Halton Sequence Generator
def halton_sequence_value(index, base):
    
    result = 0
    f = 1

    while index > 0:
        f = f*1.0/base
        result = result + f*(index % base)
        index = index/base
    
    return result

# Wrap the values around 0 and 1
def wrap_around(coordinate):

    for i in range(numpy.size(coordinate)):
        if coordinate[i] > 1.0:
            coordinate[i] = coordinate[i] - 1.0
        if coordinate[i] < 0:
            coordinate[i] = 1.0 + coordinate[i]

    return coordinate

# Halton Graph Generator
def euclidean_halton_graph(n, radius, bases, lower, upper, offset, space_dim, enable=True):

    G = nx.Graph()

    position = {i-1 : wrap_around(numpy.array([halton_sequence_value(i,base) for base in bases]) + offset) for i in range(1, n+1)}

    if space_dim == 2:
        state = {i: `position[i][0]` + ' ' + `position[i][1]` for i in position.keys()}
    
    if space_dim == 3:
        state = {i: `position[i][0]` + ' ' + `position[i][1]`+ ' ' +`position[i][2]` for i in position.keys()}

    if space_dim == 4:
        state = {i: `position[i][0]` + ' ' + `position[i][1]` + ' ' + `position[i][2]` + ' ' + `position[i][3]` for i in position.keys()}

    if space_dim == 8:
        state = {i: `position[i][0]` + ' ' + `position[i][1]` + ' ' + `position[i][2]` + ' ' + `position[i][3]` + ' ' + `position[i][4]` + ' ' + `position[i][5]` + ' ' + `position[i][6]` + ' ' + `position[i][7]` for i in position.keys()}

    for i in range(n):
        node_id = i
        G.add_node(node_id, state = state[i])

    for i in range(n-1):     
        for j in range(i+1,n):
            if numpy.linalg.norm(position[i]-position[j]) < radius:
                G.add_edge(i, j) 
    return G


def grid_graph():

    G = nx.Graph()
    # Consider a 21x21 gridworld. Add nodes with position attributes in [0,1)
    position = []
    for i in range(441):
        # Obtain the x and y positions
        node_id = i+1
        rem = (node_id%21)
        div = int(node_id/21)
        if rem == 0:
            pos_y = 1
            pos_x = (div-1)*0.05
        else:
            pos_y = (rem-1)*0.05
            pos_x = div*0.05

        position.append([pos_x,pos_y])
        G.add_node(node_id, state = "%f %f" %(pos_x,pos_y))

    position = numpy.array(position)
    # Add edges -> horizontal-right and vertical-up
    for i in range(441):
        node_id = i+1
        if not node_id%21 == 0:
            G.add_edge(node_id,node_id+1)
        if not node_id/21.0 > 20:
            G.add_edge(node_id,node_id+21)

    # Make the graph undirected
    G = G.to_undirected()
    return G


# Main Function
if __name__ == "__main__":

    space_dim = 8

    if space_dim == 2:
        bases = [2,3]
    if space_dim == 3:
        bases = [2,3,5]
    if space_dim == 4:
        bases = [2,3,5,7]
    if space_dim == 8:
        bases = [2,3,5,7,11,13,17,19]

    lower = numpy.ones(space_dim)*0
    upper = numpy.ones(space_dim)

    # Settings
    halton_points = 400
    disc_radius = 4*halton_points**(-0.25)

    for i in range(1):
        print i
        numpy.random.seed()
        offset = numpy.random.random_sample(space_dim,)
        riskmapFile = 'graphs/halton' + `space_dim` + 'D' + `halton_points` + '_' + `i` + '.graphml'
        # Example: halton2D2000_0.graphml

        # Generate the graph
        G = euclidean_halton_graph(halton_points, disc_radius, bases, lower, upper, offset, space_dim)
        nx.write_graphml(G, riskmapFile)
