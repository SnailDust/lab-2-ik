import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from ikpy.utils import geometry


# Returns true if the given nodes intersect the plane given
# a vector and position for the plane
def intersection(nodes, vector, position):
    intersect = False
    start = nodes[0]
    end = nodes[1]
    for i in range(len(nodes) - 1):
        offsetA = (vector[0]*(start[0]-position[0]) + 
                   vector[1]*(start[1]-position[1]) + 
                   vector[2]*(start[2]-position[2]))
        offsetB = (vector[0]*(start[0]-position[0]) + 
                   vector[1]*(start[1]-position[1]) + 
                   vector[2]*(start[2]-position[2]))
        if (offsetA >= 0 and offsetB <= 0) or (offsetA <= 0 and offsetB >= 0):
            intersect = True
            break   
    return intersect

my_chain = ikpy.chain.Chain.from_urdf_file("./ur5/ur5_gripper.urdf")
joints = [0, 0, 0, 0, 0, 0, 0, 0]
nodes = []

transformation_matrixes = my_chain.forward_kinematics(joints, full_kinematics=True)
# Get the nodes and the orientation from the transformation matrix
for (index, link) in enumerate(my_chain.links):
    (node, orientation) = geometry.from_transformation_matrix(transformation_matrixes[index])
    nodes.append(node)

# Set up normal vector and point
vector = np.array([0,1,0])
position = np.array([0,-1,0])

# Test 3 different planes
print("Normal:", vector, "Position:", position)#Normal: [0 1 0] Position: [ 0 -1 0]
print(intersection(nodes, vector, position))#False
position = np.array([0,0,-0.1])
print("Normal:", vector, "Position:", position)#Normal: [0 1 0] Position: [ 0. 0. -0.1]
print(intersection(nodes, vector, position))#True
vector = np.array([0,0,1])
print("Normal:", vector, "Position:", position)#Normal: [0 0 1] Position: [ 0. 0. -0.1]
print(intersection(nodes, vector, position))#False

# Convert the normal and point into
# a format that plot_surface under stands.
# These 3 lines were based on this stack overflow post
# https://stackoverflow.com/questions/3461869/plot-a-plane-based-on-a-normal-vector-and-a-point-in-matlab-or-matplotlib
d = -position.dot(vector)
xx, yy = np.meshgrid(range(-1,2), range(-1,2))
z = (-vector[0] * xx - vector[1] - d) * 1. /vector[2]

#Set up plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
my_chain.plot(joints,ax)
ax.plot_surface(xx, yy, z, alpha=0.5)

ax.set_xlim([-1,1])
ax.set_ylim([-1,1])
ax.set_zlim([-0.727,0.727])

plt.show()