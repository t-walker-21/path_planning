from graph.grah_utils import generate_neighbors
from graph.grah_utils import graph_search, Graph
from world.world_utils import World



grid = World((700, 700))

grid.add_obstacle((50, 25), 6, "circle")

grid.add_obstacle((20, 25), 7, "circle")

start_node = (0, 0)
goal_node = (500, 520)

path = graph_search(start_node, goal_node, grid)

grid.color_node(start_node, (0, 200, 0))
grid.color_node(goal_node, (200, 0, 0))

for node in path:
    grid.color_node(node,(0, 100, 0))

grid.show_world(0)