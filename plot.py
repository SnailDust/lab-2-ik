import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import random
import math
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d import Axes3D

my_chain = ikpy.chain.Chain.from_urdf_file("./ur5/ur5_gripper.urdf")

x_res = []
y_res = []
z_res = []

num_points = 1000

# Pick random target points
# Record the resulting end effector positions
for i in range(num_points):
    x = random.random()*2-1
    y = random.random()*2-1
    z = random.random()*2-1

    target_position = [(x/math.sqrt(x**2 + y**2 + z**2))*2, 
                       (y/math.sqrt(x**2 + y**2 + z**2))*2, 
                       (z/math.sqrt(x**2 + y**2 + z**2))*2]

    real_frame = my_chain.forward_kinematics(
        my_chain.inverse_kinematics(target_position))

    x_res.append(real_frame[:3, 3][0])
    y_res.append(real_frame[:3, 3][1])
    z_res.append(real_frame[:3, 3][2])

    if i % (num_points/100) == 0:
        print("Progress:", int(i/num_points*100), "%", end="\r")

points = np.column_stack((x_res, y_res, z_res))

hull = ConvexHull(points)

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
ax.scatter(x_res, y_res, z_res, c="blue")
ax.scatter(0,0,0,c="green")
for simplex in hull.simplices:
    ax.plot(points[simplex, 0], 
            points[simplex, 1], 
            points[simplex, 2], c="red")

ax.set_xlim([-1.375,1.375])
ax.set_ylim([-1.375,1.375])
ax.set_zlim([-1,1])

plt.show()