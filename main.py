from Map2D import Map2D
from PathFinding import PathFinding, Node
import time

map2d = Map2D()
map2d.load_path_data("data/FlightPath.csv")
map2d.load_lidar_data("data/LIDARPoints.csv")
map2d.view_obstacles()
map2d.generate_grid()
map2d.discretize_start_end()

t_begin = time.process_time()

pf = PathFinding(map2d)

pf.visualize_path()

t_end = time.process_time()
print("Execution time (s): ", t_end - t_begin)

input("Press ENTER to exit")