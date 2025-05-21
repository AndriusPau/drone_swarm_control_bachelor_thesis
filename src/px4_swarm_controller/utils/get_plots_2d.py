import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Enables 3D projection
import glob

color_batches = ['blue', 'red', 'green', 'orange', 'purple', 'cyan', 'magenta', 'brown']

# Load all CSVs in the current directory matching this pattern
csv_files = glob.glob("/home/del/drone_logs/*_local_position_log.csv")

fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
ax = fig.add_subplot(111)
ax.set_xlim(-5, 100)
ax.set_ylim(-5, 100)
# ax.set_zlim(-2, 10)

for file in csv_files:
    # print(file)
    df = pd.read_csv(file)
    # print(df)
    label = file
    label = label.replace("_local_position_log.csv", "")  # Get drone name from filename
    label = label.replace("/home/del/drone_logs/", "")  # Get drone name from filename
    drone_number = int(label.replace("px4_", ""))
    # print(label)

    batch_index = (drone_number - 1) // 5  # Group in 5s: 1-5, 6-10, etc.
    color = color_batches[batch_index % len(color_batches)]

    # Plot the X, Y, Z path
    # print(df['x'])
    # print(df['y'])
    # print(df['z'])
    # ax.plot(df['x'].values, df['y'].values, df['z'].values, label=label)
    ax.plot(df['x'].values, df['y'].values, label=label, color=color)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
# ax.set_zlabel('Z [m]')
ax.set_title('Drone Flight Paths')
ax.legend()
plt.tight_layout()
plt.show()
