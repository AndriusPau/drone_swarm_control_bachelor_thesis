import matplotlib.pyplot as plt
import numpy as np

target_slowdown_distance = 15
target_precision = 1
repulsion_max_distance = 17.5
repulsion_min_distance = 3
attraction_drone_max_distance = 15
attraction_drone_min_distance = 1
# Define your custom function here
def get_attraction_destination_formula(distance):
    if distance > target_slowdown_distance:
        return 1
    if distance < target_precision:
        return 0
    return (np.log(distance/target_precision))/(np.log(target_slowdown_distance/target_precision))


def get_repulsion_obstacle_formula(distance):
    if distance <= repulsion_min_distance:
        return 1
    if distance >= repulsion_max_distance:
        return 0
    return 1 - pow(((distance - repulsion_min_distance) / (repulsion_max_distance - repulsion_min_distance)), 2)


def get_attraction_drone_formula(distance):
    if (distance <= attraction_drone_min_distance or 
        distance >= attraction_drone_max_distance):
        return 0
    return np.log(distance) / np.log(attraction_drone_max_distance * 2)


# Create input range
x_values = np.linspace(-1, 20, 500)
y_values = [get_repulsion_obstacle_formula(x) for x in x_values]

# Plot the function
plt.figure(figsize=(8, 4))
plt.plot(x_values, y_values)
plt.axhline(0, color='whitesmoke', lw=0.1)
plt.axvline(0, color='whitesmoke', lw=0.1)
plt.xlabel("distance")
plt.ylabel("get_attraction_destination_formula(distance)")
plt.ylim(-0.25, 1.25)
plt.grid(True)
plt.show()
