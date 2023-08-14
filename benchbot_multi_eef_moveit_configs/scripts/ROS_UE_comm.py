import time
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import copy
import numpy as np
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cosh
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
import matplotlib.pyplot as plt
import pandas as pd
import open3d as o3d
import os
import cv2
MAX_PLANNING_ATTEMPTS = 5

# roslaunch demo.launch
def readSetPoints(filePath = None):
    if filePath is not None:
        pos1 = np.loadtxt(filePath+'/setpoints1_xyz.csv', delimiter=',')
        pos2 = np.loadtxt(filePath+'/setpoints2_xyz.csv', delimiter=',')
        rot1 = np.loadtxt(filePath+'/setpoints1_rot.csv', delimiter=',')
        rot2 = np.loadtxt(filePath+'/setpoints2_rot.csv', delimiter=',')
        print(pos1.shape)
        print(rot1.shape)
        setpoints1 = list(zip(rot1.tolist(),pos1.tolist()))
        setpoints2 = list(zip(rot2.tolist(),pos2.tolist()))
    return [setpoints1, setpoints2]
class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self) -> None:
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        print("Robot Starting...")
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("move_group_python_interface", anonymous=True)
        #rate = rospy.Rate(100)

        robot1_name = "Robot1"
        robot2_name = "Robot2"
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot1_group = moveit_commander.MoveGroupCommander(robot1_name)
        self.robot2_group = moveit_commander.MoveGroupCommander(robot2_name)

        self.robot1_frame = self.robot1_group.get_planning_frame()
        self.robot1_eef_link = self.robot1_group.get_end_effector_link()
        self.robot2_frame = self.robot2_group.get_planning_frame()
        self.robot2_eef_link = self.robot2_group.get_end_effector_link()

        self.robot_group_names = self.robot.get_group_names()
        print("Configuring Robots...")
        self.config_group(self.robot1_group, planner="RRTConnect")
        self.config_group(self.robot2_group, planner="RRTConnect")
        print("Configuring Scene...")
        self.add_scene()
        print("Resetting Robot Position...")
        self.go_to_home_state(self.robot1_group)
        self.go_to_home_state(self.robot2_group)
        
        print("Robot Start Up Complete")
    def config_group(self, group: moveit_commander.MoveGroupCommander,
                    planner = "RRTstar",
                    time = 2,
                    attempts = 400,
                    vel = 1,
                    acc = 1):
        group.set_planner_id(planner)
        group.set_planning_time(time)
        group.set_num_planning_attempts(attempts)
        group.set_max_velocity_scaling_factor(vel)
        group.set_max_acceleration_scaling_factor(acc)
        print(f"Group:{group.get_name()}\n\tplanner:{group.get_planner_id()}\n\tplanning_time:{group.get_planning_time()}")
        return
    def go_to_home_state(self, group : moveit_commander.MoveGroupCommander):
        joint_goal = group.get_current_joint_values()
        joint_goal = [0,-np.pi/2,0,-np.pi/2,0,0]
        group.go(joint_goal, wait=True)
        group.stop()
    def go_to_joint_state(self):
        joint_goal = self.robot1_group.get_current_joint_values()
        joint_goal[0] = pi
        joint_goal[-2] = pi
        self.robot1_group.go(joint_goal, wait=True)
        self.robot1_group.stop()
        new_joint_pos = self.robot1_group.get_current_joint_values()
        print(joint_goal)
        print(new_joint_pos)
        return
    def go_to_pose_goal(self,
                        group : moveit_commander.MoveGroupCommander,
                        xyz = [0,0,0],
                        rpy = [0,0,0],
                        attempt = 0,
                        type_move = "joint"):
        pose_goal = geometry_msgs.msg.Pose()
        #print(rpy)
        xyzw = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        pose_goal.orientation.w = xyzw[3]
        pose_goal.orientation.x = xyzw[0]
        pose_goal.orientation.y = xyzw[1]
        pose_goal.orientation.z = xyzw[2]
        pose_goal.position.x = xyz[0]
        pose_goal.position.y = xyz[1]
        pose_goal.position.z = xyz[2]
        group.set_pose_target(pose_goal)
        #print(pose_goal)
        #print(group.get_name(),'-'*10,attempt)
        found_plan, plan, b, c = group.plan()
        #print(found_plan)
        #print(plan)
        #print(b)
        #print(c)
        if not found_plan:
            print(f"Was unable to find plan, Finding new plan: Attempt{attempt}/{MAX_PLANNING_ATTEMPTS}")
            if attempt < MAX_PLANNING_ATTEMPTS:
                type_move = self.go_to_pose_goal(group, xyz = xyz, rpy = rpy, attempt= attempt+1, type_move= type_move)                
            else:
                self.go_to_home_state(group)
                type_move = "home"
            print(attempt, type_move)
            return type_move
        print("Found Plan")
        success = group.execute(plan, wait=True)
        if not success:
            print(f"Was unable to execute plan, Finding new plan: Attempt{attempt}/{MAX_PLANNING_ATTEMPTS}")
            if attempt < MAX_PLANNING_ATTEMPTS:
                type_move = self.go_to_pose_goal(group, xyz = xyz, rpy = rpy, attempt= attempt+1, type_move= type_move)
            else:
                self.go_to_home_state(group)
                type_move = "home"
        group.stop()
        print("Executed Plan")
        group.clear_pose_targets()
        #print(group.get_current_pose().pose)
        print(attempt, type_move)
        return type_move
    def plan_cartesian_paths(self, 
                            group : moveit_commander.MoveGroupCommander,
                            xyz = [0,0,0],
                            rpy = [0,0,0]):
        waypoints = [] # List of Pose
        pose_goal = geometry_msgs.msg.Pose()
        #print(rpy)
        xyzw = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        pose_goal.orientation.w = xyzw[3]
        pose_goal.orientation.x = xyzw[0]
        pose_goal.orientation.y = xyzw[1]
        pose_goal.orientation.z = xyzw[2]
        pose_goal.position.x = xyz[0]
        pose_goal.position.y = xyz[1]
        pose_goal.position.z = xyz[2]
        waypoints.append(pose_goal)

        (plan, fraction) = group.compute_cartesian_path(
        waypoints, eef_step = 0.01, jump_threshold = 0.0)
        print("-"*20)
        if fraction > 0.9999:
            print("Found and Executing Linear Movement...")
            group.execute(plan, wait=True)
            type_move = "linear"
        else:
            print("No Linear Plan and Executing Joint Movement")
            type_move = self.go_to_pose_goal(group, xyz = xyz, rpy = rpy)
        return type_move
    def add_scene(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.75
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[0]
        box_pose.pose.orientation.z = q[0]
        box_pose.pose.orientation.w = q[0]
        self.scene.add_box("plant_stand", box_pose, size=(0.5, 0.5, .95))
        box_pose.pose.position.z = 1.375
        self.scene.add_box("pot", box_pose, size=(0.3, 0.3, .25))
        box_pose.pose.position.y = 1.0
        box_pose.pose.position.z = 1.0
        self.scene.add_box("rob2_stand", box_pose, size=(0.2, 0.2, .95))
        box_pose.pose.position.y = -1.0
        self.scene.add_box("rob1_stand", box_pose, size=(0.2, 0.2, .95))

class RViz_UE_Interface(object):
    def __init__(self, scene : moveit_commander.PlanningSceneInterface) -> None:
        from sensor_msgs.msg import PointCloud2
        self.scene = scene
        #self.ue_intrinsics = o3d.camera.PinholeCameraIntrinsic(1920,1080,2424.2424877852509, 2424.2424877852509, 960, 540)
        self.ue_intrinsics = o3d.camera.PinholeCameraIntrinsic(1920,1080,1656.661035256662, 1658.711605946089, 960, 540)
        self.frame_id = "world"
        self.pcd_pub = rospy.Publisher('/unreal/point_clouds', PointCloud2, queue_size=1)
        pass
    def UEdepth_to_RViz_pointcloud(self, depth, color):        
        import std_msgs.msg
        import sensor_msgs.point_cloud2 as pcl2
        from sensor_msgs.msg import PointField
        # generate point cloud
        #cv2.imshow('depth', color)
        #cv2.waitKey(1)
        o3d_depth = o3d.geometry.Image(depth)#*87.5)
        #print(np.max(depth/100.))
        o3d_color = o3d.geometry.Image(color)
        o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, self.ue_intrinsics)
        pcd = pcd.scale(87.5,[0,0,0])
        #o3d.visualization.draw_geometries([o3d_rgbd])
        #pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        #print(pcd)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        #print(len(pcd.points))
        #print(np.max(pcd.points))
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]
        #ros_pcd = pcl2.create_cloud_xyz32(header, pcd.points[::10])
        #pcl2.create_cloud(header, fields)
        #print(pcd.colors)
        
        o3d_pcd_colors = (np.array(pcd.colors)*255).astype(np.uint32)
        #o3d_pcd_colors = np.zeros(np.array(pcd.colors).shape, dtype=np.uint32)
        #max_colors = np.argmax(np.array(pcd.colors), axis=1)
        #print(max_colors)
        #o3d_pcd_colors[:,0] = 255 
        #o3d_pcd_colors[:,2] = 255 
        rgba = np.zeros((len(pcd.points),1),dtype=object)
        #print((o3d_pcd_colors[:,0] << 24).shape)
        rgba += 255 << 24#.reshape((len(pcd.points),1)) # a
        rgba += (o3d_pcd_colors[:,0] << 16).reshape((len(pcd.points),1)) # r
        rgba += (o3d_pcd_colors[:,1] << 8).reshape((len(pcd.points),1)) # g
        rgba += o3d_pcd_colors[:,2].reshape((len(pcd.points),1)) #b
        #print(np.binary_repr(rgba[0,0], width=32), rgba.dtype)
        #print(o3d_pcd_colors[0,0],
        #      o3d_pcd_colors[0,1],
        #      o3d_pcd_colors[0,2])
        packed_points = np.hstack((pcd.points,rgba))
        ros_pcd_color = pcl2.create_cloud(header, fields, packed_points[::1000])
        #print(ros_pcd)
        self.pcd_pub.publish(ros_pcd_color)
        #o3d.visualization.draw_geometries([pcd])
    def update_RViz_point_cloud(self, wait_for_image = True, time_out = 5):
        image_dir = "/mnt/d/UnrealProjects/Project1-UR10/Test2 5.2/Saved/Test/"
        temp_images = os.listdir(image_dir+"temp/")
        #print(temp_images)
        found_image = [f for f in temp_images if "_Depth" in f or "_Color" in f]
        if wait_for_image:
            timeout_time = time.time() + time_out
            while len(found_image) < 2:
                temp_images = os.listdir(image_dir+"temp/")
                found_image = [f for f in temp_images if "_Depth" in f or "_Color" in f]
                #print(found_image)
                if time.time() > timeout_time:
                    print("Timed out waiting for Image")
                    return
        else:
            if len(found_image) < 2:
                return
        color = cv2.cvtColor(cv2.imread(image_dir+"temp/"+[i for i in found_image if "_Color" in i][0]), cv2.COLOR_BGRA2RGB)
        depth = cv2.imread(image_dir+"temp/"+[i for i in found_image if "_Depth" in i][0], cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
        import shutil
        shutil.move(image_dir+"temp/"+[i for i in found_image if "_Color" in i][0], image_dir+"Color/"+[i for i in found_image if "_Color" in i][0])
        shutil.move(image_dir+"temp/"+[i for i in found_image if "_Depth" in i][0], image_dir+"Depth/"+[i for i in found_image if "_Depth" in i][0])

        self.UEdepth_to_RViz_pointcloud(depth, color)
        

# elbow sholder lift pan 1 2 3
    #interface.add_box()
def callback_joint(data : JointState, args):#, goalReached, csv_writer):
        # demo.launch
    args[0].publish(Vector3(np.degrees(data.position[0]), np.degrees(data.position[1]), np.degrees(data.position[2])))
        # demo_gazebo.launch
    #args[0].publish(Vector3(np.degrees(data.position[2]), np.degrees(data.position[1]), np.degrees(data.position[0])))
    
    args[1].publish(Vector3(np.degrees(data.position[3]), np.degrees(data.position[4]), np.degrees(data.position[5])))
    
        # demo.launch
    args[2].publish(Vector3(np.degrees(data.position[6]), np.degrees(data.position[7]), np.degrees(data.position[8])))
        # demo_gazebo.launch
    #args[2].publish(Vector3(np.degrees(data.position[8]), np.degrees(data.position[7]), np.degrees(data.position[6])))
    
    args[3].publish(Vector3(np.degrees(data.position[9]), np.degrees(data.position[10]),np.degrees( data.position[11])))

def callback_UE1(data, args):#_ue_pos, _ros_pos, tf_listener):
    #print(data)
    args[0][0] = -data.x/100
    args[0][1] = data.y/100
    args[0][2] = data.z/100
    (trans, quat) = args[2].lookupTransform('world','rob1_ee_link',rospy.Time(0))
    args[1][0] = trans[0]
    args[1][1] = trans[1]
    args[1][2] = trans[2]
    args[3].append(args[0]+args[1]+[time.time()])

def callback_UE2(data, args):#_ue_pos, _ros_pos, tf_listener):
    #print(data)
    args[0][0] = -data.x/100
    args[0][1] = data.y/100
    args[0][2] = data.z/100
    (trans, quat) = args[2].lookupTransform('world','rob2_ee_link',rospy.Time(0))
    args[1][0] = trans[0]
    args[1][1] = trans[1]
    args[1][2] = trans[2]
    args[3].append(args[0]+args[1]+[time.time()])
    #print(time.time())
    #print()
    #print(trans)
    #print(args[0])
if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=8)
    start_time = time.time()
    goals = readSetPoints('setpoints')
    goal_id = [0,0]
    
    max_goal_id = [len(goals[0]), len(goals[1])]
    rospy.init_node('ROS_UE_comm', anonymous=True)
    rate = rospy.Rate(100)
    pub01 = rospy.Publisher('/unreal/0/vec3_1', Vector3, queue_size=1)#0)
    pub02 = rospy.Publisher('/unreal/0/vec3_2', Vector3, queue_size=1)#0)
    pub11 = rospy.Publisher('/unreal/1/vec3_1', Vector3, queue_size=1)#0)
    pub12 = rospy.Publisher('/unreal/1/vec3_2', Vector3, queue_size=1)#0)
    pub_data0 = rospy.Publisher('/unreal/0/takedata', Bool, queue_size=1)
    pub_data1 = rospy.Publisher('/unreal/1/takedata', Bool, queue_size=1)
    rospy.Subscriber('/joint_states', JointState, callback_joint, callback_args=(pub01,pub02,pub11,pub12))
    listener = tf.TransformListener()
    
    time.sleep(1)
    ue_pos = [[0,0,0],[0,0,0]]
    ros_pos = [[0,0,0],[0,0,0]]
    robot1_position_record = []
    robot2_position_record = []
    rospy.Subscriber('/unreal/0/reachedgoal', Vector3, callback_UE1, callback_args=(ue_pos[0],ros_pos[0],listener, robot1_position_record))
    rospy.Subscriber('/unreal/1/reachedgoal', Vector3, callback_UE2, callback_args=(ue_pos[1],ros_pos[1],listener, robot2_position_record))

    interface = MoveGroupPythonInterfaceTutorial()
    ue_rviz_interface = RViz_UE_Interface(interface.scene)
    times_all = []
    dists_all = []
    while not rospy.is_shutdown():
        print("\n")
        print("TF")
        print(len(robot1_position_record))
        print(len(robot2_position_record))
        #now = rospy.Time.now()
        #listener.waitForTransform('rob1_ee_link','world', now, rospy.Duration(4.0))
        #print(listener.lookupTransform('world','rob1_ee_link',rospy.Time(0)))
        #print(listener.lookupTransform('world','rob2_ee_link',rospy.Time(0)))
        #print(listener.lookupTransform('world','rob2_base_link',rospy.Time(0)))
        # execute robot 1 routine
        if goal_id[0] > goal_id[1] and goal_id[1] < max_goal_id[1]:
            goal = goals[1][goal_id[1]]
            success = interface.plan_cartesian_paths(interface.robot2_group, xyz=goal[1], rpy=goal[0])
            #success = interface.go_to_pose_goal(interface.robot2_group, xyz=goal[1], rpy=goal[0])
            
            #plt.plot(times, dists)
            #plt.show()
            if success in ["linear", "joint"]:
                goal_id[1] += 1
                distance = np.linalg.norm([goal[1][0]-ue_pos[1][0], goal[1][1]-ue_pos[1][1], goal[1][2]-ue_pos[1][2]])
                print(f"Immediate Distance: {distance}")
                t_end = time.time() + 1
                times = []
                dists = []
                while time.time() < t_end:
                    distance = np.linalg.norm([goal[1][0]-ue_pos[1][0], goal[1][1]-ue_pos[1][1], goal[1][2]-ue_pos[1][2]])
                    #print(f"Time: {t_end-time.time()} Distance: {distance}") 
                    times.append(time.time()-t_end+1)
                    dists.append(distance)  
                print(f"One Second Distance: {distance}")
                times_all.append(times)
                dists_all.append(dists)
                pub_data1.publish(Bool(True))
                ue_rviz_interface.frame_id = "rob2_cam_link"
        elif goal_id[0] <= goal_id[1] and goal_id[0] < max_goal_id[0]:
            goal = goals[0][goal_id[0]]
            success = interface.plan_cartesian_paths(interface.robot1_group, xyz=goal[1], rpy=goal[0])
            
            #plt.plot(times, dists)
            #plt.show()
            #success = interface.go_to_pose_goal(interface.robot1_group, xyz=goal[1], rpy=goal[0])
            if success in ["linear", "joint"]:
                goal_id[0] += 1
                distance = np.linalg.norm([goal[1][0]-ue_pos[0][0], goal[1][1]-ue_pos[0][1], goal[1][2]-ue_pos[0][2]])
                print(f"Immediate Distance: {distance}")
                t_end = time.time() + 1
                times = []
                dists = []
                while time.time() < t_end:
                    distance = np.linalg.norm([goal[1][0]-ue_pos[0][0], goal[1][1]-ue_pos[0][1], goal[1][2]-ue_pos[0][2]])
                    times.append(time.time()-t_end+1)
                    dists.append(distance)
                print(f"One Second Distance: {distance}")
                times_all.append(times)
                dists_all.append(dists) 
                #print(f"Time: {t_end-time.time()} Distance: {distance}")
                pub_data0.publish(Bool(True))
                ue_rviz_interface.frame_id = "rob1_cam_link"
        else:
            break
        
        time.sleep(0.1)
        pub_data1.publish(Bool(False))
        pub_data0.publish(Bool(False))
        ue_rviz_interface.update_RViz_point_cloud(wait_for_image = True)
        #print(f"Goal (xyz, rpy): {goal}")
        #print(f"Robot Positions (xyz): {ue_pos}")
        rate.sleep()
        #rospy.spin()
        #continue
    
    end_time = time.time()
    print(f"Total Time: {end_time-start_time}")
    print(f"Immediate: {pd.DataFrame([d[0] for d in dists_all]).describe()}")
    print(f"Final: {pd.DataFrame([d[-1] for d in dists_all]).describe()}")
    import more_itertools
    print(f"Flattened: {pd.DataFrame(more_itertools.flatten(dists_all)).describe()}")
    np.savetxt("/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot1_pos.csv",
               np.array(robot1_position_record),delimiter=',',header="ue_x,ue_y,ue_z,ros_x,ros_y,ros_z,t")#,fmt="%s")
    np.savetxt("/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot2_pos.csv",
               np.array(robot2_position_record),delimiter=',',header="ue_x,ue_y,ue_z,ros_x,ros_y,ros_z,t")#,fmt="%s")
    for i in range(len(times_all)):
        plt.step(times_all[i], dists_all[i], where='post')
    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")
    plt.title("Unreal Engine Distance to Goal Setpoints\n1 Second Window after RViz Reached Goal")
    plt.show()
    #plt.plot(dists)
    #plt.show()