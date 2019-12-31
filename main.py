from Map2D import Map2D
from PathFinding import PathFinding, Node
import time

map2d = Map2D()
map2d.load_path_data("data/FlightPath.csv")
map2d.load_lidar_data("data/LIDARPoints.csv")
map2d.view_obstacles("LIDAR Obstacle detection / Path", 5) # View 6th 360-scan data
map2d.generate_grid()


pf = PathFinding(map2d)

t_begin = time.process_time()
pf.generate_path()
t_end = time.process_time()

# pf.visualize_path("Alternative Path - Grid View")
pf.write_path_csv("data/AlternativePath.csv")

# Overwrite original path, visualizes new generated path
map2d.load_path_data("data/AlternativePath.csv")
map2d.view_obstacles("Alternative Path")


print("Path finding execution time (s): ", t_end - t_begin)

input("Press ENTER to exit")