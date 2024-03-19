import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
dir1 = "/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot1_joints.csv"
joints1 = pd.read_csv(dir1, header=None)
print(joints1)
plt.plot(joints1.iloc[:, -1], joints1.iloc[:,6], '+-')
plt.plot(joints1.iloc[:, -1], joints1.iloc[:,7], '+-')
plt.plot(joints1.iloc[:, -1], joints1.iloc[:,8], '+-')
plt.plot(joints1.iloc[:, -1], joints1.iloc[:,9], '+-')
plt.plot(joints1.iloc[:, -1], joints1.iloc[:,10], '+-')
plt.plot(joints1.iloc[:, -1], joints1.iloc[:,11], '+-')
dir2 = "/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot2_joints.csv"
joints2 = pd.read_csv(dir2, header=None)
print(joints1)
plt.plot(joints2.iloc[:, -1], joints2.iloc[:,6], '+-')
plt.plot(joints2.iloc[:, -1], joints2.iloc[:,7], '+-')
plt.plot(joints2.iloc[:, -1], joints2.iloc[:,8], '+-')
plt.plot(joints2.iloc[:, -1], joints2.iloc[:,9], '+-')
plt.plot(joints2.iloc[:, -1], joints2.iloc[:,10], '+-')
plt.plot(joints2.iloc[:, -1], joints2.iloc[:,11], '+-')
plt.show()