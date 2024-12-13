# import csv
# import matplotlib.pyplot as plt

# x_coords = []
# y_coords = []

# with open('turtlebot_path.csv', 'r') as f:
#     reader = csv.reader(f)
#     next(reader)  # skip header
#     for row in reader:
#         x_coords.append(float(row[0]))
#         y_coords.append(float(row[1]))

# plt.plot(x_coords, y_coords, marker='o')
# plt.title('Turtlebot Path')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.grid(True)
# plt.show()


import csv
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np

x_coords = []
y_coords = []

with open('turtlebot_path.csv', 'r') as f:
    reader = csv.reader(f)
    next(reader)  # skip header
    for row in reader:
        x_coords.append(float(row[0]))
        y_coords.append(float(row[1]))

# convert to numpy arrays for easier manipulation
x_coords = np.array(x_coords)
y_coords = np.array(y_coords)

# create segments between consecutive points
points = np.array([x_coords, y_coords]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

# create a colormap based on the index of the points
# t will represent a normalized 'time' from start (0) to end (1)
t = np.linspace(0, 1, len(segments))

fig, ax = plt.subplots()
cmap = plt.cm.viridis
norm = plt.Normalize(0, 1)

# create a line collection, set the array to t which maps to colors
lc = LineCollection(segments, cmap=cmap, norm=norm)
lc.set_array(t)
lc.set_linewidth(2)
line = ax.add_collection(lc)

# add a colorbar to show the progression
cbar = fig.colorbar(line, ax=ax)
cbar.set_label('Normalized Time (0 = start, 1 = end)')

ax.set_title('Turtlebot Path with Time Gradient')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.grid(True)

# set the plot limits
ax.set_xlim(x_coords.min(), x_coords.max())
ax.set_ylim(y_coords.min(), y_coords.max())

plt.show()

