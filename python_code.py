import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
m = 1.0           # Mass
b = 1.0           # Damping coefficient
k = 1.0           # Spring constant
F0 = 1.0          # Driving force amplitude
omega = 1.0       # Driving frequency

# Time
dt = 0.2
T = 20
time = np.arange(0, T, dt)

# Initial conditions
x = 0
v = 0
x_values = []

# Simulation
for t in time:
    x_values.append(x)
    a = (1/m) * (F0 * np.cos(omega * t) - b * v - k * x)
    x += v * dt
    v += a * dt

# Create figure and axis
fig, axis = plt.subplots(2,1, figsize=(10, 20))  # Two subplots in a column

# First graph: Dot + Line from extreme left
connecting_line, = axis[0].plot([], [], color="red")
dot_point, = axis[0].plot([], [], markersize=10, marker="o", color="blue")
axis[0].set_xlim(min(x_values) - 1, max(x_values) + 1)  # X-axis range based on displacement
axis[0].set_ylim(-1, 1)  # Fixed Y-axis range for horizontal motion
axis[0].set_xlabel("Displacement")
axis[0].set_title("displace with respect to time")
axis[0].grid(True)

# Second graph: Animated Line Graph
animated_line, = axis[1].plot([], [], color="green", label="Wave")
axis[1].set_xlim(0, T)
axis[1].set_ylim(min(x_values) - 1, max(x_values) + 1)
axis[1].set_xlabel("Time")
axis[1].set_ylabel("Displacement")
axis[1].set_title("osscilation wave")
axis[1].grid(True)

# Update function for animation
def update(frame):
    # First graph: Connecting line and dot
    connecting_line.set_data([min(x_values) - 1, x_values[frame]],[0,0])  # Line grows/shrinks with dot
    dot_point.set_data([x_values[frame]], [0])  # Dot moves horizontally

    # Second graph: Line graph animation
    animated_line.set_data(time[:frame], x_values[:frame])  # Evolving wave

    return connecting_line, dot_point, animated_line

# Create animation
animation = FuncAnimation(fig=fig, func=update, frames=len(time), interval=25)
animation.save('animation.gif', writer='pillow', fps=30)  # Save the animation as a GIF
# Display the plot
plt.show()