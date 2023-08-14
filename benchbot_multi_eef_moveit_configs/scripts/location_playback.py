import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits import mplot3d
import scipy.stats as stats
from dtaidistance import dtw_ndim
import seaborn as sns
TIME_DELAY = 0
out_dir = "/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/3/"
robot1_df = pd.read_csv(out_dir + 'robot1_pos.csv')#, header=None)
robot2_df = pd.read_csv(out_dir + 'robot2_pos.csv')#, header=None)
#joints_1 = pd.read_csv('/home/lixin/catkin_ws/src/BenchBot/logs/robot1_ROS_location.csv')
#joints_2 = pd.read_csv('/home/lixin/catkin_ws/src/BenchBot/logs/robot2_ROS_location.csv')
#robot1_df = pd.read_csv('/home/lixin/catkin_ws/src/BenchBot/logs/robot1_UE_location.csv')#, header=None)
#robot2_df = pd.read_csv('/home/lixin/catkin_ws/src/BenchBot/logs/robot2_UE_location.csv')#, header=None)

robot1_df.rename(columns={robot1_df.columns[0]:"ue_x_1",
                          robot1_df.columns[1]:"ue_y_1",
                          robot1_df.columns[2]:"ue_z_1",
                          robot1_df.columns[3]:"ros_x_1",
                          robot1_df.columns[4]:"ros_y_1",
                          robot1_df.columns[5]:"ros_z_1"}, inplace=True)

robot2_df.rename(columns={robot2_df.columns[0]:"ue_x_2",
                          robot2_df.columns[1]:"ue_y_2",
                          robot2_df.columns[2]:"ue_z_2",
                          robot2_df.columns[3]:"ros_x_2",
                          robot2_df.columns[4]:"ros_y_2",
                          robot2_df.columns[5]:"ros_z_2"}, inplace=True)
min_time = np.min([np.min(robot1_df['t']),np.min(robot2_df['t'])])
robot1_df['t'] -= min_time
robot2_df['t'] -= min_time
print(robot1_df)
robots = pd.concat([robot1_df,robot2_df])
robots.sort_values(by=["t"], inplace=True)
robots.interpolate(inplace=True)
robots.fillna(method='bfill', inplace=True)
robots = robots.reset_index()
print(robots)

#robot_UE.plot(x='Time', y=['x_UE1', 'y_UE1', 'z_UE1','x_UE2', 'y_UE2', 'z_UE2'])

UE_SET = ["ue_x_1","ue_y_1","ue_z_1","ue_x_2","ue_y_2","ue_z_2"]
ROS_SET = ["ros_x_1","ros_y_1","ros_z_1","ros_x_2","ros_y_2","ros_z_2"]

emd = []
dtw_dist = []
#fig, axes = plt.subplots(2,3)
for i in range(6):
    #print(np.isnan(robots[ROS_SET[i]].to_numpy().flatten()).any())
    ros_coords = robots[ROS_SET[i]].to_numpy().flatten()
    ros_coords = ros_coords[~np.isnan(ros_coords)]
    ue_coords = robots[UE_SET[i]].to_numpy().flatten()
    ue_coords = ue_coords[~np.isnan(ue_coords)]
    emd_ = stats.wasserstein_distance(ros_coords, ue_coords)
    emd.append(emd_)
    from scipy import signal
    corr = signal.correlate(ros_coords, ue_coords, mode="full")
    lags = signal.correlation_lags(ros_coords.size, ue_coords.size, mode='full')
    lag = lags[np.argmax(corr)]
    #corr /= np.max(np.abs(corr))
    print("time-lag corrlation: ", np.max(corr), " at, ", lag)
    #dtw_ = dtw.distance(ros_coords, ue_coords, window=25, max_step=25, use_pruning=True)
    #dtw_dist.append(dtw_)
    #print("dtw distance: ", dtw_)
    #robots.plot(x="Time", y=[ROS_SET[i], UE_SET[i]], marker='.',
    #            title=f"EDM = {emd_:.4f}, DTW = {dtw_:.4f}, corr = {time_lags[np.argmax(corr)]}",
    #            ax=axes[int(i/3), int(i%3)])
for i in range(2):
    
    ros_coords = robots[ROS_SET[i*3:i*3+3]].to_numpy()
    ue_coords = robots[UE_SET[i*3:i*3+3]].to_numpy()
    dtw_ = dtw_ndim.distance(ros_coords, ue_coords, window=25, max_step=25, use_pruning=True)
    dtw_dist.append(dtw_)
    print(dtw_)
#plt.show()
#print(emd)
print("average emd (no time shifts): ", np.average(emd))
print("average dtw (no time shifts): ", np.average(dtw_dist))

robots["e_x1"] = abs(robots["ros_x_1"] - robots["ue_x_1"])
robots["e_y1"] = abs(robots["ros_y_1"] - robots["ue_y_1"])
robots["e_z1"] = abs(robots["ros_z_1"] - robots["ue_z_1"])
robots["e_x2"] = abs(robots["ros_x_2"] - robots["ue_x_2"])
robots["e_y2"] = abs(robots["ros_y_2"] - robots["ue_y_2"])
robots["e_z2"] = abs(robots["ros_z_2"] - robots["ue_z_2"])

robots["e_dist1"] = (robots["e_x1"].pow(2) + robots["e_y1"].pow(2) + robots["e_z1"].pow(2)).pow(.5)
robots["e_dist2"] = (robots["e_x2"].pow(2) + robots["e_y2"].pow(2) + robots["e_z2"].pow(2)).pow(.5)

#robots["e_dist1"].fillna(method="ffill", inplace=True)
#robots["e_dist2"].fillna(method="ffill", inplace=True)
robots.plot(x="t", y=["e_dist1", "e_dist2"], title="L2 distance error between ROS and UE5", xlabel="time (s)", ylabel="distance (m)", legend=["robot 1", "robot 2"])
robots.plot(x="t", y=["e_dist1", "e_dist2"], marker = '.')
robots.plot(x="t", y = ["ros_x_1", "ros_y_1", "ros_z_1", "ue_x_1", "ue_y_1", "ue_z_1"], title="Positions of Robot 1", xlabel="time (s)", ylabel="positin (m)", legend=["X_ROS","Y_ROS","Z_ROS","X_UE","Y_UE","Z_UE"], marker = '.', linestyle = 'none')
print(robots[["e_dist1", "e_dist2"]].describe())
robots.to_csv(out_dir+"robots.csv")
plt.figure()
err_cols = ["e_x1","e_y1","e_z1","e_x2","e_y2","e_z2", "e_dist1", "e_dist2"]
sns.boxplot(data=robots[err_cols][robots[err_cols] > 0.005],orient="h")
plt.show()


#### 3D Animation
x_1 = robots["ue_x_1"].iloc[::1].to_numpy().flatten()
y_1 = robots["ue_y_1"].iloc[::1].to_numpy().flatten()
z_1 = robots["ue_z_1"].iloc[::1].to_numpy().flatten()
x_2 = robots["ue_x_2"].iloc[::1].to_numpy().flatten()
y_2 = robots["ue_y_2"].iloc[::1].to_numpy().flatten()
z_2 = robots["ue_z_2"].iloc[::1].to_numpy().flatten()
x1 = robots["ros_x_1"].iloc[::1].to_numpy().flatten()
y1 = robots["ros_y_1"].iloc[::1].to_numpy().flatten()
z1 = robots["ros_z_1"].iloc[::1].to_numpy().flatten()
x2 = robots["ros_x_2"].iloc[::1].to_numpy().flatten()
y2 = robots["ros_y_2"].iloc[::1].to_numpy().flatten()
z2 = robots["ros_z_2"].iloc[::1].to_numpy().flatten()
#import mayavi.mlab as mlab
#mlab.clf()
#mlab.plot3d(x1,y1,z1, color = (0,0,1), line_width=0.1)
#mlab.plot3d(x_1,y_1,z_1, color = (1,0,0), line_width=0.1)
#mlab.plot3d(x2,y2,z2, color = (0,1,0), line_width=0.1)
#mlab.plot3d(x_2,y_2,z_2, color = (1,1,0), line_width=0.1)
#mlab.show()

fig = plt.figure()
ax = plt.axes(projection='3d', computed_zorder = False)
#ax.plot3D(x_1,y_1,z_1)
#ax.plot3D(x_2,y_2,z_2)
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.set_zlim(0,4)
def update(n, x, y, z, points):
    if n % 100 == 0:
        print("frame: ", n)
    ax.view_init(20,n/4)
    for i, point in enumerate(points):
        point.set_data(x[i][:n],y[i][:n])
        point.set_3d_properties(z[i][:n])
        point.set_alpha(0.5)
    return point,

point1, = ax.plot(x1,y1,z1, label = "ROS_1", color = 'b', alpha=0.5)
point2, = ax.plot(x_1,y_1,z_1, label = "UE_1", color = 'r', alpha=0.5)
point3, = ax.plot(x2,y2,z2, label = "ROS_2", color = 'g', alpha=0.5)
point4, = ax.plot(x_2,y_2,z_2, label = "UE_2", color = 'y', alpha=0.5)
ax.legend()
print(int(max(len(x1),len(x_1))))
fps = int(1/(robots["t"].max()/len(x1)))*2

if True:
    ani = animation.FuncAnimation(fig,
                                  update,
                                  frames = int(max(len(x1),len(x_1))),
                                  fargs=([x1, x_1, x2, x_2],
                                         [y1, y_1, y2, y_2],
                                         [z1, z_1, z2, z_2],
                                         [point1,point2, point3, point4]),
                                  interval = 1,
                                  blit=False,
                                  repeat = False)
    FFwriter = animation.FFMpegWriter(fps=fps)

    ani.save(out_dir+"animation.mp4", writer = FFwriter, dpi=600)
plt.show()
