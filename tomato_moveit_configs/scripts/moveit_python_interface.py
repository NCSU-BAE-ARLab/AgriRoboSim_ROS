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
# roslaunch demo.launch

class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self) -> None:
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)
        rate = rospy.Rate(100)

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
        self.go_to_home_state()
        print("robot 1 planning frame: ",self.robot1_frame)
        print("robot 1 eef link: ", self.robot1_eef_link)
        print("robot 2 planning frame: ",self.robot2_frame)
        print("robot 2 eef link: ", self.robot2_eef_link)
        print("avaliable planning groups: ", self.robot_group_names)
        print(self.robot1_group.get_planner_id())
        #print("Current State: ", self.robot.get_current_state())
        self.add_scene()
    def go_to_home_state(self):
        joint_goal = self.robot1_group.get_current_joint_values()
        joint_goal = [0]*6
        self.robot1_group.go(joint_goal, wait=True)
        self.robot1_group.stop()
        self.robot2_group.go(joint_goal, wait=True)
        self.robot1_group.stop()
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
    def go_to_pose_goal(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0
        pose_goal.position.y = 1.5
        pose_goal.position.z = 2
        self.robot2_group.set_pose_target(pose_goal)
        success = self.robot2_group.go(wait = True)
        print(success)
        if not success:
            self.go_to_pose_goal()
        self.robot2_group.stop()
        self.robot2_group.clear_pose_targets()
        print(self.robot2_group.get_current_pose().pose)
        return
    def plan_cartesian_paths(self, scale = 1):
        waypoints = [] # List of Pose
        wpose = self.robot1_group.get_current_pose().pose
        wpose.position.x += scale * 0.1
        wpose.position.y += 0.0
        wpose.position.z += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z -= scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.robot1_group.compute_cartesian_path(
        waypoints, eef_step = 0.01, jump_threshold = 0.0)
        self.robot1_group.execute(plan, wait=True)
    def add_box(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[0]
        box_pose.pose.orientation.z = q[0]
        box_pose.pose.orientation.w = q[0]
        self.scene.add_box("box_name", box_pose, size=(0.1, 0.1, 1))
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
        self.scene.add_box("plant_stand", box_pose, size=(0.5, 0.5, 1))
        box_pose.pose.position.z = 1.375
        self.scene.add_box("pot", box_pose, size=(0.3, 0.3, .25))
        box_pose.pose.position.y = 1.0
        box_pose.pose.position.z = 1.0
        self.scene.add_box("rob2_stand", box_pose, size=(0.2, 0.2, 1))
        box_pose.pose.position.y = -1.0
        self.scene.add_box("rob1_stand", box_pose, size=(0.2, 0.2, 1))

if __name__ == "__main__":
    
    interface = MoveGroupPythonInterfaceTutorial()
    interface.go_to_joint_state()
    interface.go_to_pose_goal()
    interface.plan_cartesian_paths()
    #interface.add_box()