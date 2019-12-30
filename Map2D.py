import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import csv
from collections import defaultdict

class Map2D:
    def __init__(self):
        self.scan_pos = None
        self.obstacle_coords = None

    def load_path_data(self, path):

        raw_data = pd.read_csv(path, header=None)
        csv_index = pd.Index(raw_data[0][::2], dtype='int', name="Scan")

        self.scan_pos = pd.DataFrame(raw_data[1::2])
        self.scan_pos.columns=['X', 'Y']
        self.scan_pos.index = csv_index

    def load_lidar_data(self, path):

        lidar_data = defaultdict(list)

        with open(path, "r") as f:
            reader = csv.reader(f, delimiter=",")
            read_sweep_data = True
            n_sweep_in_scan = 0
            current_sweep = 99999
            current_scan = 0
            for i, line in enumerate(reader):
                if current_sweep < n_sweep_in_scan:             
                    #append
                    #print(i, line, current_scan)
                    lidar_data["Scan"].append(current_scan)
                    lidar_data["Angle"].append(float(line[0]))
                    lidar_data["Distance"].append(float(line[1]))
                    current_sweep += 1
                else:
                    current_scan = int(line[0])
                    n_sweep_in_scan = int(line[1])
                    current_sweep = 0



        obstacle_df = pd.DataFrame(lidar_data)
        self.obstacle_coords = obstacle_df.join(other=self.scan_pos, on="Scan")
        
        # Compute absolute position of LIDAR detection points
        self.obstacle_coords['X'] = self.obstacle_coords['X'] + ((self.obstacle_coords['Distance'] / 1000) * np.cos(np.radians(self.obstacle_coords['Angle'])))
        self.obstacle_coords['Y'] = self.obstacle_coords['Y'] - ((self.obstacle_coords['Distance'] / 1000) * np.sin(np.radians(self.obstacle_coords['Angle'])))

    def view_drone_path(self):

        ax1 = self.scan_pos.plot(x='X', y='Y')

    def view_obstacles(self):
        fig = plt.figure(figsize=(10, 7))
        ax1 = fig.add_subplot(111)

        ax1.scatter(self.obstacle_coords['X'][self.obstacle_coords['Scan'] > 0], self.obstacle_coords['Y'][self.obstacle_coords['Scan'] > 0], marker="s", label='obstacle', alpha=0.3)
        ax1.plot(self.scan_pos['X'], self.scan_pos['Y'], "ok-", label='path', color="orange")

        ax1.legend(loc="lower left")
        plt.show()
    
map = Map2D()
map.load_path_data("data/FlightPath.csv")
map.load_lidar_data("data/LIDARPoints.csv")
map.view_obstacles()

input("Press ENTER to exit")

