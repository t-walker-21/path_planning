from planner.plan_utils import Planner
from world.world_utils import World

w = World((800, 800))



for _ in range(30):
    w.add_random_obstacle()

start = (20, 20)
goal = (700, 500)

#w.add_obstacle((300,300), 80, "circle", (0, 0, 0))

p = Planner(w)

p.find_path(start, goal, visualize=1)

exit()



for _ in range(10):
    w.add_random_obstacle()

p = Planner(w)

#p.find_path((50, 50), (70, 70))

pt = p.get_random_point(w, (25, 25))

w.color_node(pt, (200, 0, 0))

w.show_world(0)