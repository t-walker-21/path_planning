"""

Utilities to perform graph operations
"""

def generate_neighbors(node, world_size):
    """

    Function to generate neighbors of a node
    """

    neighbors = []
    skip = 5

    # Node shall be (x, y)

    # Append in order left, right, up, down

    # Left
    if not (node[0] == 0):
        neighbors.append((node[0] - skip, node[1]))

    # Right
    if not (node[0] == world_size[0] - skip):
        neighbors.append((node[0] + skip, node[1]))

    # Up
    if not (node[1] == 0):
        neighbors.append((node[0], node[1] - skip))

    # Down
    if not (node[1] == world_size[1] - skip):
        neighbors.append((node[0], node[1] + skip))

    return neighbors

def graph_search(source, goal, grid, dfs=0):
    """

    Graph search
    """
    
    hash_dict = {}

    visited = []
    to_visit = []

    path = []

    # Put source node in open list

    #src = str(source)

    #src_hash = str(hash(src)) + ","

    #hash_dict[src] = src_hash

    to_visit.append(source)
    path.append(source)

    while not (len(to_visit) == 0):

        current_node = to_visit.pop(0)

        path.append(current_node)

        # Are we done?

        if current_node == goal:
            print ("found path to node")

            #print visited

            path.append(goal)

            print path

            return path

        # Get neighbors of current node

        neighbors = generate_neighbors(current_node, grid.world.shape)

        if len(neighbors) == 0:
            path.remove(-1)

        for neighbor in neighbors:

            # Add neighbors to open list if they have not been visited and if it is not an obstacle

            if not ((grid.world[neighbor[0]][neighbor[1]] == 0).all() or neighbor in visited):
                to_visit.insert(-1 * dfs, neighbor)

        # Done, now mark this node as visited

        visited.append(current_node)

        grid.color_node(current_node, (0, 0, 100))