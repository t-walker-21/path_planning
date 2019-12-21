from world.world_utils import World

map = World((100, 100))

#for _ in range(10):
    #map.add_random_obstacle()


map.color_node((5, 5), (0, 0, 0))
map.show_world(0)