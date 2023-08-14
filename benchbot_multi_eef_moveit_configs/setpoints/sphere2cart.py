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
            rot_at_i = [-np.pi/2,0,np.pi-cylinder[:,i][1]]
            #rot_at_i = so3.from_rpy([0,0,3*np.pi/2-cylinder[:,i][1]])
            rotations[i] = rot_at_i
    return rotations

Robot1_Coord = [0,-1,0]
Robot2_Coord = [0,1,0]
#Rot_Matrix = so3.from_rpy((0,0,np.pi/2))
# cylinder properties
R_cy = .6
Phi_cy_N = 10
height_N = 1
height_1 = .1
height = 0
height_2 = height_1 + height
Phi_cy_Range = [-4*np.pi/10, 4*np.pi/10]
height_Range = [height_1, height_2]
heights = np.linspace(height_Range[1], height_Range[0], height_N)
Phi_cy = np.linspace(Phi_cy_Range[0], Phi_cy_Range[1], Phi_cy_N)
R_cy_Sample, height_Sample, Phi_cy_Sample = np.meshgrid(R_cy, heights, Phi_cy)
for i in range(height_N):
    if i % 2 == 1 and i < len(Phi_cy_Sample):
        print(i)
        Phi_cy_Sample[i] = np.flip(Phi_cy_Sample[i])
#for i in range(height_N):
#    if i % 2 == 1 and i < len(height_Sample):
#        print(i)
#        height_Sample[i] = np.flip(height_Sample[i])
# sphere properties
R = .5
Theta_N = 5
Phi_N = 10
Theta_Range = [np.pi/5, 3*np.pi/7]
Phi_Range = [np.pi/6, np.pi-np.pi/6]
Theta = np.linspace(Theta_Range[0], Theta_Range[1], Theta_N)
Phi = np.linspace(Phi_Range[0], Phi_Range[1], Phi_N)
print(Phi)
R_Sample, Theta_Sample, Phi_Sample = np.meshgrid(R, Theta, Phi)
for i in range(Phi_N):
    if i % 2 == 1 and i < len(Phi_Sample):
        print(i)
        Phi_Sample[i] = np.flip(Phi_Sample[i])

# XYZ
Robot1_Setpoints_Cylinder_Plant = np.vstack((R_cy_Sample.flatten(),Phi_cy_Sample.flatten(),height_Sample.flatten()))
Robot1_Setpoints_Cartesian_Plant = CylindricalToCartesian(Robot1_Setpoints_Cylinder_Plant)
Robot1_Setpoints_Spherical_Plant = np.vstack((R_Sample.flatten(),Theta_Sample.flatten(),Phi_Sample.flatten()))
Robot1_Setpoints_Cartesian_Plant = np.concatenate(( SphericalToCartesian(Robot1_Setpoints_Spherical_Plant, height_2+height/height_N),
                                                    Robot1_Setpoints_Cartesian_Plant),
                                                    axis=1)
Robot1_Setpoints_Cartesian_Plant[1] = -Robot1_Setpoints_Cartesian_Plant[1]

    # transform world to robot local
Robot1_Setpoints_Cartesian = Robot1_Setpoints_Cartesian_Plant.copy()
#Robot1_Setpoints_Cartesian[0] = (Robot1_Setpoints_Cartesian_Plant[0] - Robot1_Coord[0]) 
#Robot1_Setpoints_Cartesian[1] = (Robot1_Setpoints_Cartesian_Plant[1] - Robot1_Coord[1]) 
Robot1_Setpoints_Cartesian[2] = (Robot1_Setpoints_Cartesian_Plant[2] + 1.5) 

# robot 2
Robot2_Setpoints_Cylinder_Plant = np.vstack((R_cy_Sample.flatten(),Phi_cy_Sample.flatten(),height_Sample.flatten()))
Robot2_Setpoints_Cartesian_Plant = CylindricalToCartesian(Robot2_Setpoints_Cylinder_Plant)
Robot2_Setpoints_Spherical_Plant = np.vstack((R_Sample.flatten(),Theta_Sample.flatten(),Phi_Sample.flatten()))
Robot2_Setpoints_Cartesian_Plant = np.concatenate(( SphericalToCartesian(Robot2_Setpoints_Spherical_Plant, height_2+height/height_N),
                                                    Robot2_Setpoints_Cartesian_Plant),
                                                    axis=1)
#Robot2_Setpoints_Cartesian_Plant[1] = -Robot2_Setpoints_Cartesian_Plant[1]
    # transform world to robot local
Robot2_Setpoints_Cartesian = Robot2_Setpoints_Cartesian_Plant.copy()
#Robot2_Setpoints_Cartesian[0] = (Robot2_Setpoints_Cartesian_Plant[0] - Robot2_Coord[0]) 
#Robot2_Setpoints_Cartesian[1] = (Robot2_Setpoints_Cartesian_Plant[1] - Robot2_Coord[1]) 
Robot2_Setpoints_Cartesian[2] = (Robot2_Setpoints_Cartesian_Plant[2] +1.5) 

np.savetxt('setpoints/setpoints1_xyz.csv', Robot1_Setpoints_Cartesian.T, delimiter=',')
np.savetxt('setpoints/setpoints2_xyz.csv', Robot2_Setpoints_Cartesian.T, delimiter=',')

# ROTATION
Robot1_Setpoints_Rotation = CylindricalToRotation(Robot1_Setpoints_Cylinder_Plant, robotID=0)
print(Robot1_Setpoints_Rotation.shape, SphericalToRotation(Robot1_Setpoints_Spherical_Plant, robotID = 0).shape)
Robot1_Setpoints_Rotation = np.concatenate((SphericalToRotation(Robot1_Setpoints_Spherical_Plant, robotID = 0),
                                            Robot1_Setpoints_Rotation), 
                                            axis=0)
Robot2_Setpoints_Rotation = CylindricalToRotation(Robot2_Setpoints_Cylinder_Plant, robotID=1)
Robot2_Setpoints_Rotation = np.concatenate((SphericalToRotation(Robot2_Setpoints_Spherical_Plant, robotID = 1),
                                            Robot2_Setpoints_Rotation), 
                                            axis=0)
np.savetxt('setpoints/setpoints1_rot.csv', Robot1_Setpoints_Rotation, delimiter=',')
np.savetxt('setpoints/setpoints2_rot.csv', Robot2_Setpoints_Rotation, delimiter=',')

fig = plt.figure()
ax = plt.axes(projection='3d')
#ax.plot3D(Robot1_Setpoints_Cartesian_Plant[0], Robot1_Setpoints_Cartesian_Plant[1], Robot1_Setpoints_Cartesian_Plant[2], 'gray')
#ax.plot3D(Robot2_Setpoints_Cartesian_Plant[0], Robot2_Setpoints_Cartesian_Plant[1], Robot2_Setpoints_Cartesian_Plant[2], 'gray')
ax.plot3D(Robot1_Setpoints_Cartesian[0], Robot1_Setpoints_Cartesian[1], Robot1_Setpoints_Cartesian[2], 'green')
ax.plot3D(Robot2_Setpoints_Cartesian[0], Robot2_Setpoints_Cartesian[1], Robot2_Setpoints_Cartesian[2], 'red')

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