import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import csv
from collections import defaultdict

class Map2D:
    def __init__(self):
        self.scan_pos = None
        self.obstacle_coords = None

        #Properties of real world dimensions
        self.X_lidar_offset = None
        self.Y_lidar_offset = None
        self.X_lidar_size = None
        self.Y_lidar_size = None


        # Properties of discretized map
        self.grid = None
        self.X_res = None
        self.Y_res = None
        self.X_bin = None
        self.Y_bin = None
        


        self.tolerance = 0

        self.start_quantified = None
        self.end_quantified = None

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

        self.obstacle_coords = self.obstacle_coords[self.obstacle_coords["Distance"]> 100]
        
        # Compute absolute position of LIDAR detection points
        self.obstacle_coords['X'] = self.obstacle_coords['X'] + ((self.obstacle_coords['Distance'] / 1000) * np.cos(np.radians(self.obstacle_coords['Angle'])))
        self.obstacle_coords['Y'] = self.obstacle_coords['Y'] - ((self.obstacle_coords['Distance'] / 1000) * np.sin(np.radians(self.obstacle_coords['Angle'])))

        self.save_lidar_dimensions()
    
    def save_lidar_dimensions(self):
        self.X_lidar_offset = self.obstacle_coords["X"].min()
        self.Y_lidar_offset = self.obstacle_coords["Y"].min()
        self.X_lidar_size = self.obstacle_coords["X"].max() - self.obstacle_coords["X"].min()
        self.Y_lidar_size = self.obstacle_coords["Y"].max() - self.obstacle_coords["Y"].min()

    def view_drone_path(self):

        ax1 = self.scan_pos.plot(x='X', y='Y')

    def view_obstacles(self, title, scan_id=None):
        fig = plt.figure(figsize=(10, 7))
        fig.canvas.set_window_title(title)
        ax1 = fig.add_subplot(111)
        ax1.set_xlabel("X position")
        ax1.set_xlabel("Y position")

        ax1.plot(self.scan_pos['X'], self.scan_pos['Y'], "ok-", label='path', color="orange")

        if scan_id:
            ax1.scatter(self.obstacle_coords['X'][self.obstacle_coords['Scan'] == scan_id], self.obstacle_coords['Y'][self.obstacle_coords['Scan'] == scan_id], marker="s", label='obstacle', alpha=0.3)
            ax1.plot(self.scan_pos.iloc[scan_id]['X'], self.scan_pos.iloc[scan_id]['Y'], "o", label='drone position', color="red", markersize=10)
        else:
            ax1.scatter(self.obstacle_coords['X'], self.obstacle_coords['Y'], marker="s", label='obstacle', alpha=0.3)
        

        ax1.legend(loc="lower left")
        plt.show()

    def quantize_obstacles_coords(self):

        division_size = 0.1 # 10cm by 10cm subdivisions

        self.X_res = int(self.X_lidar_size / division_size)
        self.Y_res = int(self.Y_lidar_size / division_size)

        self.X_bin = np.linspace(self.obstacle_coords["X"].min(), self.obstacle_coords["X"].max(), self.X_res)
        self.Y_bin = np.linspace(self.obstacle_coords["Y"].min(), self.obstacle_coords["Y"].max(), self.Y_res)

        x_quantized_coords = np.digitize(self.obstacle_coords["X"], bins=self.X_bin)
        y_quantized_coords = np.digitize(self.obstacle_coords["Y"], bins=self.Y_bin)

        grid_obstacles_coords = np.c_[x_quantized_coords, y_quantized_coords]
        grid_obstacles_coords = grid_obstacles_coords - 1

        return grid_obstacles_coords

    def generate_grid(self):
        discrete_obstacle_coords = self.quantize_obstacles_coords()

        self.grid = np.zeros((self.X_res, self.Y_res), int)
        np.add.at(self.grid, tuple(zip(*discrete_obstacle_coords)), 1)

       
        self.grid = self.grid > self.tolerance

        # Store discretized start point and end point
        start_point = self.scan_pos.iloc[0]
        end_point = self.scan_pos.iloc[-1]

        self.start_quantified = self.discretize_point(start_point)
        self.end_quantified = self.discretize_point(end_point)

        # plt.figure(figsize = (20,15))
        # plt.imshow(self.grid)

    def discretize_point(self, point):

        return np.array([np.digitize(point["X"], bins=self.X_bin),np.digitize(point["Y"], bins=self.Y_bin)])