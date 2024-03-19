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
DISTANCE_THRESHOLD = 0.05 / 1000 # mm
WAIT_TIME = 5 # seconds to wait for image files
RENDER_TIME = 1
MAX_POINT_CLOUD = 100_000 # exceeding this will downsample the point cloud using voxel_down_sample
INTRINSICS_LIST = { "1080p_90FOV": o3d.camera.PinholeCameraIntrinsic(1920,1080,2424.2424877852509, 2424.2424877852509, 960, 540),
                    "1080p_60FOV": o3d.camera.PinholeCameraIntrinsic(1920,1080,1656.661035256662, 1658.711605946089, 960, 540),
                    "1080p_RaspPiv3Wide": o3d.camera.PinholeCameraIntrinsic(1920,1080,818.60469302023728, 818.60469302023728, 960, 540), # Rasp Cam v3 Wide model 
                    "480p_90FOV": o3d.camera.PinholeCameraIntrinsic(640, 480,320, 320, 320, 240),
                    "960p_90FOV": o3d.camera.PinholeCameraIntrinsic(1280,960,640, 640, 640, 480) # 1280x960, 90 FOV
                   }
#IMAGE_DIR = "/mnt/c/Users/lixin/AppData/Local/AgriRoboSim/Saved/Test/"
IMAGE_DIR ="/mnt/d/UnrealProjects/AgriRoboSim_UE5/AgriRoboSim/Saved/Test/"
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
        self.scene.add_box("plant_stand", box_pose, size=(0.5, 0.5, 0.95))
        box_pose.pose.position.z = 1.375
        self.scene.add_box("pot", box_pose, size=(0.3, 0.3, .25))
        box_pose.pose.position.y = 1.0
        box_pose.pose.position.z = 1.0
        self.scene.add_box("rob2_stand", box_pose, size=(0.25, 0.25, .95))
        box_pose.pose.position.y = -1.0
        self.scene.add_box("rob1_stand", box_pose, size=(0.25, 0.25, .95))

class RViz_UE_Interface(object):
    def __init__(self, scene : moveit_commander.PlanningSceneInterface, listener, ue_intrinsics) -> None:
        from sensor_msgs.msg import PointCloud2
        self.listener = listener
        self.scene = scene
        #self.ue_intrinsics = o3d.camera.PinholeCameraIntrinsic(1920,1080,2424.2424877852509, 2424.2424877852509, 960, 540)
        self.ue_intrinsics = ue_intrinsics
        #self.ue_intrinsics = o3d.camera.PinholeCameraIntrinsic(640,480,320, 320, 320, 240) # 640x480, 90 FOV
        #self.ue_intrinsics = o3d.camera.PinholeCameraIntrinsic(1280,960,640, 640, 640, 480) # 1280x960, 90 FOV
        self.frame_id = "world"
        self.pcd_pub = rospy.Publisher('/unreal/point_clouds', PointCloud2, queue_size=1)
        self.combined_point_cloud = o3d.geometry.PointCloud()
        pass
    def UEdepth_to_RViz_pointcloud(self, depth, color):        
        import std_msgs.msg
        import sensor_msgs.point_cloud2 as pcl2
        from sensor_msgs.msg import PointField
        # generate point cloud

        o3d_depth = o3d.geometry.Image(depth[:,:,2]*10) # UE5 captures depth in cm, depth should be mm
        o3d_color = o3d.geometry.Image(color)
        o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, self.ue_intrinsics)
        #o3d.visualization.draw_geometries([pcd])

        # transform pointcloud from camera frame to world frame, and then concat with previous pointcloud
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        p, q = self.listener.lookupTransform('world',self.frame_id,rospy.Time(0))
        #pcd.transfrom()
        #print(transform)
        norm = np.linalg.norm(q)
        if np.abs(norm - 1.0) > 1e-3:
            raise ValueError(
                "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                    str(q), np.linalg.norm(q)))
        elif np.abs(norm - 1.0) > 1e-6:
            q = q / norm
        g = tf.transformations.quaternion_matrix(q)
        g[0:3, -1] = p
        print(np.asarray(pcd.points).shape)
        print(g)
        pcd = pcd.transform(g)
        self.combined_point_cloud += pcd
        if (len(self.combined_point_cloud.points) > MAX_POINT_CLOUD):
            self.combined_point_cloud = self.combined_point_cloud.voxel_down_sample(0.001)
        self.save_combined_point_cloud()

        # prepare to publish the current point cloud to RViz (not combined point cloud)
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]
        
        o3d_pcd_colors = (np.array(pcd.colors)*255).astype(np.uint32)
        rgba = np.zeros((len(pcd.points),1),dtype=object)
        rgba += 255 << 24 # a
        rgba += (o3d_pcd_colors[:,0] << 16).reshape((len(pcd.points),1)) # r
        rgba += (o3d_pcd_colors[:,1] << 8).reshape((len(pcd.points),1)) # g
        rgba += o3d_pcd_colors[:,2].reshape((len(pcd.points),1)) #b
        packed_points = np.hstack((pcd.points,rgba))
        #print(packed_points.shape)
        ros_pcd_color = pcl2.create_cloud(header, fields, packed_points[::1])
        #print(ros_pcd)
        self.pcd_pub.publish(ros_pcd_color)
    def update_RViz_point_cloud(self, wait_for_image = True, time_out = 5, publish = False):
        #image_dir = "/mnt/d/UnrealProjects/AgriRoboSim_UE5/AgriRoboSim/Saved/Test/"
        image_dir = IMAGE_DIR
        temp_images = os.listdir(image_dir+"temp/")
        #print(temp_images)
        found_image = [f for f in temp_images if "_Depth" in f or "_Color" in f]
        if wait_for_image:
            timeout_time = time.time() + time_out
            while len(found_image) < 3:
                temp_images = os.listdir(image_dir+"temp/")
                found_image = list(set([f for f in temp_images if "_Depth" in f or "_Color" in f or "_Segment" in f])) # sometimes depth gets read twice so only unique entries
                #print(found_image)
                if time.time() > timeout_time:
                    print("Timed out waiting for Image")
                    return
        else:
            if len(found_image) < 3:
                return
        print(found_image)
        assert os.path.exists(image_dir+"temp/"+[i for i in found_image if "_Color" in i][0])
        time.sleep(0.1) # delay time for file to be properly saved or opencv might give an unreadable error
        color = cv2.cvtColor(cv2.imread(image_dir+"temp/"+[i for i in found_image if "_Color" in i][0]), cv2.COLOR_BGRA2RGB)
        segment = cv2.cvtColor(cv2.imread(image_dir+"temp/"+[i for i in found_image if "_Segment" in i][0]), cv2.COLOR_BGRA2RGB)
        depth = cv2.imread(image_dir+"temp/"+[i for i in found_image if "_Depth" in i][0], cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
        #plt.subplot(2, 2, 1)
        #plt.imshow(color)
        #plt.subplot(2, 2, 2)
        #plt.imshow(segment)
        import shutil
        if not os.path.exists(image_dir+"Color/"):
            os.mkdir(image_dir+"Color/")
            os.mkdir(image_dir+"Segmentation/")
            os.mkdir(image_dir+"Depth/")
        shutil.move(image_dir+"temp/"+[i for i in found_image if "_Color" in i][0], image_dir+"Color/"+[i for i in found_image if "_Color" in i][0])
        shutil.move(image_dir+"temp/"+[i for i in found_image if "_Depth" in i][0], image_dir+"Depth/"+[i for i in found_image if "_Depth" in i][0])
        shutil.move(image_dir+"temp/"+[i for i in found_image if "_Segment" in i][0], image_dir+"Segmentation/"+[i for i in found_image if "_Segment" in i][0])
        #shutil.move(image_dir+"temp/"+[i for i in found_image if "_Segment" in i][0], image_dir+"Segmentation/"+[i for i in found_image if "_Segment" in i][0])
        if publish: self.UEdepth_to_RViz_pointcloud(self.filter(segment, depth, 1, 255), color)
        #if publish: self.UEdepth_to_RViz_pointcloud(depth, color)
    def filter(self, 
               segment_image, 
               depth_image, 
               segment_color_channel, 
               segment_color_value, 
               distance = 1, 
               percentage = 0):
        dimension = segment_image.shape[0] * segment_image.shape[1]
        proper_color_counts = sum(sum(segment_image[:,:,segment_color_channel] == segment_color_value))
        proper_percentage = proper_color_counts / dimension * 100
        print(proper_percentage)
        if proper_percentage <= percentage:
            segment_image[:,:,0] = 0
        mask = (segment_image[:,:,segment_color_channel] == segment_color_value).astype(np.uint8)
        depth_image = np.where(depth_image < distance*100, depth_image, 0)
        return cv2.bitwise_and(depth_image,depth_image,mask = mask)
    def save_combined_point_cloud(self):
        o3d.io.write_point_cloud("./outputs/combined_point_cloud.ply", self.combined_point_cloud)

# elbow sholder lift pan 1 2 3
    #interface.add_box()
def callback_joint(data : JointState, args):#, goalReached, csv_writer):
        # demo.launch
    #args[0].publish(Vector3(np.degrees(data.position[0]), np.degrees(data.position[1]), np.degrees(data.position[2])))
        # demo_gazebo.launch
    #args[0].publish(Vector3(np.degrees(data.position[2]), np.degrees(data.position[1]), np.degrees(data.position[0])))
    
    #args[1].publish(Vector3(np.degrees(data.position[3]), np.degrees(data.position[4]), np.degrees(data.position[5])))
    
        # demo.launch
    #args[2].publish(Vector3(np.degrees(data.position[6]), np.degrees(data.position[7]), np.degrees(data.position[8])))
        # demo_gazebo.launch
    #args[2].publish(Vector3(np.degrees(data.position[8]), np.degrees(data.position[7]), np.degrees(data.position[6])))
    
    #args[3].publish(Vector3(np.degrees(data.position[9]), np.degrees(data.position[10]),np.degrees( data.position[11])))

    #args[4].publish(data)
    #args[5].publish(data)
    pass

def callback_UE1EEF(data, args):#_ue_pos, _ros_pos, tf_listener):
    #print(data)
    args[0][0] = -data.x/100
    args[0][1] = data.y/100
    args[0][2] = data.z/100
    (trans, quat) = args[2].lookupTransform('world','rob1_ee_link',rospy.Time(0))
    args[1][0] = trans[0]
    args[1][1] = trans[1]
    args[1][2] = trans[2]
    args[3].append(args[0]+args[1]+[time.time()])

def callback_UE2EEF(data, args):#_ue_pos, _ros_pos, tf_listener):
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
def callback_UE1Joint(data : JointState, args):
    args[0] = data.position
    args[1] = data.velocity
    args[2].append(args[0]+args[1]+tuple([time.time()]))
    pass
def callback_UE2Joint(data : JointState, args):
    args[0] = data.position
    args[1] = data.velocity
    #print(type(args[0]), type(args[1]))
    args[2].append(data.position+data.velocity+tuple([time.time()]))
    #print(args[2])
    pass
def capture(pub: rospy.Publisher,
            ue_rviz: RViz_UE_Interface,
            render_time = RENDER_TIME,
            publish = False): # 0.2 fast, 0.5 medium, 1 high quality
    print("Rendering Images...")
    pub.publish(Bool(True))
    time.sleep(render_time)
    pub.publish(Bool(False))
    time.sleep(0.2)
    print("Visualizing Images as Point Clouds...")
    ue_rviz.update_RViz_point_cloud(wait_for_image = True, publish = publish)

def main():
    os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
    np.set_printoptions(suppress=True, precision=8)
    start_time = time.time()
    goals = readSetPoints('setpoints')
    goal_id = [0,0]
    
    max_goal_id = [len(goals[0]), len(goals[1])]
    rospy.init_node('ROS_UE_comm', anonymous=True)
    rate = rospy.Rate(100)
    pub01 = rospy.Publisher('/unreal/1/vec3_1', Vector3, queue_size=1)#0)
    pub02 = rospy.Publisher('/unreal/1/vec3_2', Vector3, queue_size=1)#0)
    pub11 = rospy.Publisher('/unreal/2/vec3_1', Vector3, queue_size=1)#0)
    pub12 = rospy.Publisher('/unreal/2/vec3_2', Vector3, queue_size=1)#0)
    pub0joints = rospy.Publisher('/unreal/1/joints', JointState, queue_size=1)
    pub1joints = rospy.Publisher('/unreal/2/joints', JointState, queue_size=1)#0)
    pub_data0 = rospy.Publisher('/unreal/1/takedata', Bool, queue_size=1)
    pub_data1 = rospy.Publisher('/unreal/2/takedata', Bool, queue_size=1)
    rospy.Subscriber('/joint_states', JointState, callback_joint, callback_args=(pub01,pub02,pub11,pub12,pub0joints,pub1joints))
    listener = tf.TransformListener()
    
    time.sleep(1)
    ue_pos = [[0,0,0],[0,0,0]]
    ros_pos = [[0,0,0],[0,0,0]]
    robot1_position_record = []
    robot2_position_record = []
    rospy.Subscriber('/unreal/1/reachedgoal', Vector3, callback_UE1EEF, callback_args=(ue_pos[0],ros_pos[0],listener, robot1_position_record))
    rospy.Subscriber('/unreal/2/reachedgoal', Vector3, callback_UE2EEF, callback_args=(ue_pos[1],ros_pos[1],listener, robot2_position_record))
    ue_joints = [[0,0,0,0,0,0],[0,0,0,0,0,0]]
    ue_joints_err = [[0,0,0,0,0,0],[0,0,0,0,0,0]]
    robot1_joints_record = []
    robot2_joints_record = []
    
    interface = MoveGroupPythonInterfaceTutorial()
    ue_rviz_interface = RViz_UE_Interface(interface.scene, listener, INTRINSICS_LIST["1080p_RaspPiv3Wide"])
    times_all = []
    dists_all = []
    success_counts = 0
    
    joint_sub1 = rospy.Subscriber('/unreal/2/joint_states', JointState, callback_UE2Joint, callback_args=[ue_joints[1],ue_joints_err[1],robot2_joints_record])
    while not rospy.is_shutdown():
        print("\n")
        print("-"*20)
        print(f"Goal {sum(goal_id)}/{sum(max_goal_id)}")
        # execute robot 1 routine
        if goal_id[1] < max_goal_id[1]: #and goal_id[0] > goal_id[1]:
            goal = goals[1][goal_id[1]]
            success = interface.plan_cartesian_paths(interface.robot2_group, xyz=goal[1], rpy=goal[0])
            #success = interface.go_to_pose_goal(interface.robot2_group, xyz=goal[1], rpy=goal[0])
            
            #plt.plot(times, dists)
            #plt.show()
            if success in ["linear", "joint"]:
                goal_id[1] += 1
                robot2_position_record.append([-1]*7)
                distance = np.linalg.norm([ros_pos[1][0]-ue_pos[1][0], ros_pos[1][1]-ue_pos[1][1], ros_pos[1][2]-ue_pos[1][2]])
                print(f"Immediate Distance: {distance}")
                t_end = time.time() + WAIT_TIME
                times = []
                dists = []
                while time.time() < t_end:
                    distance = np.linalg.norm([ros_pos[1][0]-ue_pos[1][0], ros_pos[1][1]-ue_pos[1][1], ros_pos[1][2]-ue_pos[1][2]])
                    #print(f"Time: {t_end-time.time()} Distance: {distance}") 
                    times.append(time.time()-t_end+WAIT_TIME)
                    dists.append(distance)
                    if distance < DISTANCE_THRESHOLD:
                        break
                robot2_position_record.append([-2]*7)
                print(f"{times[-1]} Second Distance: {distance}")
                times_all.append(times)
                dists_all.append(dists)

                if distance < DISTANCE_THRESHOLD:
                    ue_rviz_interface.frame_id = "rob2_cam_link"
                    success_counts += 1
                    capture(pub_data1, ue_rviz_interface, publish=True)
                else:
                    quit()
                    ue_rviz_interface.frame_id = "rob2_cam_link"
                    capture(pub_data1, ue_rviz_interface, publish=False)
            if goal_id[1] == max_goal_id[1]:
                interface.go_to_home_state(interface.robot2_group)
                joint_sub1.unregister()
                joint_sub2 = rospy.Subscriber('/unreal/1/joint_states', JointState, callback_UE1Joint, callback_args=[ue_joints[0],ue_joints_err[0],robot1_joints_record])
                
        elif goal_id[0] < max_goal_id[0]:# and goal_id[0] <= goal_id[1]:
            goal = goals[0][goal_id[0]]
            success = interface.plan_cartesian_paths(interface.robot1_group, xyz=goal[1], rpy=goal[0])
            
            #plt.plot(times, dists)
            #plt.show()
            #success = interface.go_to_pose_goal(interface.robot1_group, xyz=goal[1], rpy=goal[0])
            if success in ["linear", "joint"]:
                goal_id[0] += 1
                robot1_position_record.append([-1]*7)
                distance = np.linalg.norm([ros_pos[0][0]-ue_pos[0][0], ros_pos[0][1]-ue_pos[0][1], ros_pos[0][2]-ue_pos[0][2]])
                print(f"Immediate Distance: {distance}")
                t_end = time.time() + WAIT_TIME
                times = []
                dists = []
                while time.time() < t_end:
                    distance = np.linalg.norm([ros_pos[0][0]-ue_pos[0][0], ros_pos[0][1]-ue_pos[0][1], ros_pos[0][2]-ue_pos[0][2]])
                    times.append(time.time()-t_end+WAIT_TIME)
                    dists.append(distance)
                    if distance < DISTANCE_THRESHOLD:
                        break
                robot1_position_record.append([-2]*7)
                print(f"{times[-1]} Second Distance: {distance}")
                times_all.append(times)
                dists_all.append(dists) 
                #print(f"Time: {t_end-time.time()} Distance: {distance}")
                if distance < DISTANCE_THRESHOLD:
                    ue_rviz_interface.frame_id = "rob1_cam_link"
                    success_counts += 1
                    capture(pub_data0, ue_rviz_interface, publish=True)
                else:
                    quit()
                    ue_rviz_interface.frame_id = "rob1_cam_link"
                    capture(pub_data0, ue_rviz_interface, publish=False)
        else:
            break
        #print(f"Goal (xyz, rpy): {goal}")
        #print(f"Robot Positions (xyz): {ue_pos}")
        rate.sleep()
        #rospy.spin()
        #continue
    joint_sub2.unregister()
    end_time = time.time()
    print(f"Total Time: {end_time-start_time}, Successful (under threshold): {success_counts}")
    print(f"Immediate: {pd.DataFrame([d[0] for d in dists_all]).describe()}")
    print(f"Final: {pd.DataFrame([d[-1] for d in dists_all]).describe()}")
    import more_itertools
    print(f"Flattened: {pd.DataFrame(more_itertools.flatten(dists_all)).describe()}")
    print(f"EEF Settle Time: {pd.DataFrame([d[-1] for d in times_all]).describe()}")
    np.savetxt("/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot1_pos.csv",
               np.array(robot1_position_record),delimiter=',',header="ue_x,ue_y,ue_z,ros_x,ros_y,ros_z,t")#,fmt="%s")
    np.savetxt("/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot2_pos.csv",
               np.array(robot2_position_record),delimiter=',',header="ue_x,ue_y,ue_z,ros_x,ros_y,ros_z,t")#,fmt="%s")
    np.savetxt("/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot2_joints.csv",
               np.array(robot2_joints_record),delimiter=',')#,fmt="%s")
    np.savetxt("/home/lixin/catkin_ws/src/benchbot_moveit/benchbot_multi_eef_moveit_configs/outputs/robot1_joints.csv",
               np.array(robot1_joints_record),delimiter=',')
    for i in range(len(times_all)):
        plt.step(times_all[i], dists_all[i], where='post')
    plt.xlabel("Time (s)")
    plt.ylabel("Distance (m)")
    plt.title("Unreal Engine Distance to Goal Setpoints after RViz Reached Goal")
    plt.show()
    #plt.plot(dists)
    #plt.show()
    
if __name__ == "__main__":
    main()