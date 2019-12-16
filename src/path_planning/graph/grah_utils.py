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

def graph_search(graph, source, goal, dfs=0):
    """

    Graph search
    """

    visited = []
    to_visit = []

    path = []

    to_visit.append(source)
    path.append(source)

    while not (len(to_visit) == 0):

        current_node = to_visit.pop(0)

        path.append(current_node)

        print "current node: " , current_node

        # Are we done?

        if current_node == goal:
            print ("found path to node in dfs")

            #print visited

            path.append(goal)

            return visited

        # Get neighbors of current node
        neighbors = graph.edges[current_node]

        if len(neighbors) == 0:
            path.pop(-1)

        for neighbor in neighbors:

            # Add neighbors to open list if they have not been visited 

            if not (neighbor in visited):
                to_visit.insert(-1 * dfs, neighbor)

        # Done, now mark this node as visited

        visited.append(current_node)


def graph_search_recursive(graph, source, goal, path, visited, dfs=0):
    """

    Recursive definition of graph search (maintains backpointer for path)
    """

    # Base case
    
    if source == goal:
        path.append(goal)
        return True
    
    to_visit = []

    neighbors = graph.edges[source]

    for neighbor in neighbors:
        if not (neighbor in visited):
            to_visit.append(neighbor)

    if len(to_visit) == 0:
        return False

    else:

        for next_source in to_visit:
            if (graph_search_recursive(graph, next_source, goal, path, visited)):
                path.append(source)
                return path

            else:
                visited.append(next_source)

    



"""

Class definition for graph
"""

class Graph(object):
    def __init__(self):
        self.vertices = []
        self.edges = {}

    def add_vertex(self, node):

        if (node in self.vertices):
            print "Vertex already in graph"
            return

        self.vertices.append(node)
        self.edges[node] = []

    def add_edge(self, edge):
        source = edge[0]
        current_adj_list = self.edges[source]
        current_adj_list.append(edge[1])

    def print_graph(self):
        for vertex in self.vertices:
            print vertex , [edge  for edge in self.edges[vertex]]

    def get_neighbors(self, vertex):
        return self.edges[vertex]
