import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(file_path, color):

    x_coords = []
    y_coords = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            x_coords.append(float(parts[2]))
            y_coords.append(float(parts[3]))


    plt.plot(x_coords, y_coords, marker='o', markersize=3, linestyle='-', color=color)

    plt.text(x_coords[0], y_coords[0], 'Start', fontsize=10, ha='right')
    plt.text(x_coords[-1], y_coords[-1], 'End', fontsize=10, ha='right')

   
    plt.title('Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')

    

positions = np.array([
    [0, 0], [1, 0], [1, 1], [1, 2], [1, 3], [2, 3], [3, 3], [3, 4],
    [3, 5], [3, 6], [3, 7], [3, 8], [4, 8], [3, 8], [2, 8], [1, 8],
    [0, 8], [-1, 8]
])


x_coords = positions[:, 0]
y_coords = positions[:, 1]


plt.plot(x_coords, y_coords, marker='o', markersize=3, linestyle='-', color='green')


plt.text(x_coords[0], y_coords[0], 'Start', fontsize=10, ha='right')
plt.text(x_coords[-1], y_coords[-1], 'End', fontsize=10, ha='right')


file_path1 = 'kf.txt' 
file_path2 = 'fg.txt' 
plot_trajectory(file_path1, color='blue')
plot_trajectory(file_path2, color='red')

plt.show()
