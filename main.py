from Map2D import Map2D
from PathFinding import PathFinding, Node

map2d = Map2D()
map2d.load_path_data("data/FlightPath.csv")
map2d.load_lidar_data("data/LIDARPoints.csv")
map2d.view_obstacles()
map2d.generate_grid()
map2d.discretize_start_end()

pf = PathFinding(map2d)
pf.astar()

input("Press ENTER to exit")