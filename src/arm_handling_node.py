#!/usr/bin/env python3
# arm_handling_node.py
import rospy
import actionlib
import numpy as np
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from p6.msg import ScanAreaAction, PickAndPlaceAction
import tf.transformations as tf_trans

class ArmHandlingNode:
    def __init__(self):
        # Create the MoveItInterface necessary objects
        self.arm_group = MoveGroupCommander("arm", ns=rospy.get_namespace())
        self.robot = RobotCommander("robot_description")
        self.scene = PlanningSceneInterface(ns=rospy.get_namespace())
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
        
        self.scan_area_server = actionlib.SimpleActionServer('scan_area', ScanAreaAction, self.scan_area, False)
        self.pick_and_place_server = actionlib.SimpleActionServer('pick_and_place', PickAndPlaceAction, self.pick_and_place, False)
        self.scan_area_server.start()
        self.pick_and_place_server.start()

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        # rospy.loginfo("Actual cartesian pose is : ")
        # rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance = 0, constraints = None):
        arm_group = self.arm_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        # rospy.loginfo("Planning and going to the Cartesian Pose:")
        # rospy.loginfo(pose)
        return arm_group.go(wait=True)
    
    def reach_gripper_position(self, relative_position):
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 

    def get_pose_from_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = PoseStamped()
        position = pose.pose.position
        position.x = x
        position.y = y
        position.z = z

        quat = tf_trans.quaternion_from_euler(ro, pi, ya)

        orientation = pose.pose.orientation
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]

        return pose.pose

    def scan_area(self):
        rospy.loginfo("Scanning area")
        rospy.loginfo("Going to home")
        self.arm_group.set_named_target('home')
        (success_flag, trajectory_message, planning_time, error_code) = self.arm_group.plan()
        self.arm_group.execute(trajectory_message, wait=True)
        rospy.loginfo("Reached home")

        home_pose = self.get_cartesian_pose()
        euler_angles = tf_trans.euler_from_quaternion([home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z, home_pose.orientation.w])

        rospy.loginfo("Going left")
        pose_left = self.get_pose_from_xyz_rpy(home_pose.position.x, home_pose.position.y, home_pose.position.z, 
                                                euler_angles[0]+0.1, euler_angles[1], euler_angles[2] + 0.2)
        self.reach_cartesian_pose(pose_left, 0.01)
        rospy.loginfo("Reached left")

        rospy.loginfo("Going right")
        pose_right = self.get_pose_from_xyz_rpy(home_pose.position.x, home_pose.position.y, home_pose.position.z, 
                                                euler_angles[0]+0.1, euler_angles[1], euler_angles[2] - 0.2)
        self.reach_cartesian_pose(pose_right, 0.01)
        rospy.loginfo("Reached right")

        # Return to home pose
        rospy.loginfo("Going home")
        self.reach_cartesian_pose(home_pose, 0.01)
        rospy.loginfo("Reached home")
        self.scan_area_server.set_succeeded()

    def pick_and_place(self, goal):
        object_pose = PoseStamped()
        object_pose.pose.position.x = goal.object.x
        object_pose.pose.position.y = goal.object.y
        object_pose.pose.position.z = goal.object.z

        place_pose = PoseStamped()
        place_pose.pose.position.x = goal.location.x
        place_pose.pose.position.y = goal.location.y
        place_pose.pose.position.z = goal.location.z

        # TODO: Use MoveIt's pick and place utility to pick the object and place it at the location

        self.pick_and_place_server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('arm_handling_node')
    server = ArmHandlingNode()
    server.scan_area()
    rospy.spin()