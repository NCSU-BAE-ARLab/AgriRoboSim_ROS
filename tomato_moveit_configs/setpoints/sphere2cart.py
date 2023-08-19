import numpy as np
import tf
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def SphericalToCartesian(spherical, z_offset = 0):
    cartesian = np.zeros(spherical.shape)
    cartesian[0] = spherical[0] * np.sin(spherical[1]) * np.cos(spherical[2])
    cartesian[1] = spherical[0] * np.sin(spherical[1]) * np.sin(spherical[2])
    cartesian[2] = spherical[0] * np.cos(spherical[1]) + z_offset
    return cartesian

def SphericalToRotation(spherical, robotID = 0):
    rotations = np.zeros((spherical.shape[1],3))
    if robotID == 0:
        for i in range(spherical.shape[1]):
            print(spherical[:,i])
            #rot_at_i = tf.transformations.quaternion_from_euler(-np.pi+spherical[:,i][1], 0, np.pi/2 - spherical[:,i][2])
            rot_at_i = [-np.pi+spherical[:,i][1], 0, np.pi/2 - spherical[:,i][2]]
            #rot_at_i = so3.from_rpy([0,np.pi/2-spherical[:,i][1],np.pi-spherical[:,i][2]])
            rotations[i] = rot_at_i
    else:
        for i in range(spherical.shape[1]):
            rot_at_i = tf.transformations.quaternion_from_euler(-np.pi+spherical[:,i][1],0, np.pi/2 + spherical[:,i][2])
            rot_at_i = [-np.pi+spherical[:,i][1],0, np.pi/2 + spherical[:,i][2]]
            #rot_at_i = so3.from_rpy([0,np.pi/2-spherical[:,i][1],np.pi+spherical[:,i][2]])
            rotations[i] = rot_at_i
    return rotations

def CylindricalToCartesian(cylinder):
    cartesian = np.zeros(cylinder.shape)
    cartesian[0] = cylinder[0] * np.sin(cylinder[1])
    cartesian[1] = cylinder[0] * np.cos(cylinder[1])
    cartesian[2] = cylinder[2]
    return cartesian

def CylindricalToRotation(cylinder, robotID = 0):
    rotations = np.zeros((cylinder.shape[1],3))
    if robotID == 0:
        for i in range(cylinder.shape[1]):
            print(cylinder[:,i])
            #rot_at_i = tf.transformations.quaternion_from_euler(-np.pi/2,0,cylinder[:,i][1])
            rot_at_i = [-np.pi/2,0,cylinder[:,i][1]]
            #rot_at_i = so3.from_rpy([0,0,np.pi/2+cylinder[:,i][1]])
            rotations[i] = rot_at_i
    else:
        for i in range(cylinder.shape[1]):
            #rot_at_i = tf.transformations.quaternion_from_euler(np.pi/2,0,-cylinder[:,i][1])
            rot_at_i = [np.pi/2,0,-cylinder[:,i][1]]
            #rot_at_i = so3.from_rpy([0,0,3*np.pi/2-cylinder[:,i][1]])
            rotations[i] = rot_at_i
    return rotations

Robot1_Coord = [0,0,1]

# XYZ
depth = -0.4
top_left = [.5,depth,1.7]
width = 1
height = 1
width_n = 2
height_n = 2
offset_width = (width/width_n)/2
offset_height = (height/height_n)/2
y = np.linspace(top_left[0]-offset_width,top_left[0]-width+offset_width, width_n)
z = np.linspace(top_left[2]-offset_height,top_left[2]-height+offset_height, height_n)
yv, zv = np.meshgrid(y,z)
Robot1_Setpoints_Cartesian = np.vstack([yv.ravel(), np.full(yv.ravel().shape, top_left[1]), zv.ravel()])
print(Robot1_Setpoints_Cartesian)
Robot1_Setpoints_RPY = np.zeros(Robot1_Setpoints_Cartesian.shape)
Robot1_Setpoints_RPY[0] = np.pi/2
Robot1_Setpoints_RPY[1] = np.pi
#Robot2_Setpoints_Cartesian_Plant = np.concatenate(( SphericalToCartesian(Robot2_Setpoints_Spherical_Plant, height_2+height/height_N),
#                                                    Robot2_Setpoints_Cartesian_Plant),
#                                                    axis=1)
#Robot2_Setpoints_Cartesian_Plant[1] = -Robot2_Setpoints_Cartesian_Plant[1]
    # transform world to robot local
#Robot2_Setpoints_Cartesian[0] = (Robot2_Setpoints_Cartesian_Plant[0] - Robot2_Coord[0]) 
#Robot2_Setpoints_Cartesian[1] = (Robot2_Setpoints_Cartesian_Plant[1] - Robot2_Coord[1]) 

np.savetxt('setpoints/setpoints1_xyz.csv', Robot1_Setpoints_Cartesian.T, delimiter=',')
np.savetxt('setpoints/setpoints1_rot.csv', Robot1_Setpoints_RPY.T, delimiter=',')

fig = plt.figure()
ax = plt.axes(projection='3d')
#ax.plot3D(Robot1_Setpoints_Cartesian_Plant[0], Robot1_Setpoints_Cartesian_Plant[1], Robot1_Setpoints_Cartesian_Plant[2], 'gray')
#ax.plot3D(Robot2_Setpoints_Cartesian_Plant[0], Robot2_Setpoints_Cartesian_Plant[1], Robot2_Setpoints_Cartesian_Plant[2], 'gray')
ax.plot3D(Robot1_Setpoints_Cartesian[0], Robot1_Setpoints_Cartesian[1], Robot1_Setpoints_Cartesian[2], 'green')

#ax.scatter3D(Robot1_Setpoints_Cartesian_Plant[0], Robot1_Setpoints_Cartesian_Plant[1], Robot1_Setpoints_Cartesian_Plant[2],  cmap='Greens')
#ax.scatter3D(Robot1_Coord[0], Robot1_Coord[1], Robot1_Coord[2],  c='green')
#
#ax.scatter3D(Robot2_Setpoints_Cartesian_Plant[0], Robot2_Setpoints_Cartesian_Plant[1], Robot2_Setpoints_Cartesian_Plant[2],  cmap='Reds')
#ax.scatter3D(Robot2_Coord[0], Robot2_Coord[1], Robot2_Coord[2],  c='red')
ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(1,3)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()