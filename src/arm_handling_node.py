#!/usr/bin/env python3
# arm_handling_node.py
import rospy
import actionlib
import numpy as np
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
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
        pose = arm_group.get_current_pose()
        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance = 0, constraints = None):
        arm_group = self.arm_group
        arm_group.set_goal_position_tolerance(tolerance)
        if constraints is not None:
            arm_group.set_path_constraints(constraints)
        arm_group.set_pose_target(pose)
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

        return pose

    def scan_area(self, goal = None):
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
        object_pose = self.get_pose_from_xyz_rpy(goal.object.x, goal.object.y, goal.object.z, 0, 0, 0)
        place_pose = self.get_pose_from_xyz_rpy(goal.location.x, goal.location.y, goal.location.z + 0.1, 0, 0, 0)

        grasp = moveit_msgs.msg.Grasp()
        grasp.max_contact_force = 1
        grasp.grasp_pose = object_pose
        grasp.grasp_pose.header.frame_id = "base_link"
        grasp.pre_grasp_approach.direction.header.frame_id = "base_link"
        grasp.pre_grasp_approach.direction.vector.z = -1.0
        grasp.pre_grasp_approach.min_distance = 0.15
        grasp.pre_grasp_approach.desired_distance = 0.2
        grasp.post_grasp_retreat.direction.header.frame_id = "base_link"
        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.15
        grasp.post_grasp_retreat.desired_distance = 0.2

        rospy.loginfo("Picking object...")
        self.arm_group.pick('object', grasp)
        rospy.loginfo("Object picked")

        place = moveit_msgs.msg.PlaceLocation()
        place.place_pose = place_pose
        place.pre_place_approach.direction.header.frame_id = "base_link"
        place.pre_place_approach.direction.vector.z = -1.0
        place.pre_place_approach.min_distance = 0.15
        place.pre_place_approach.desired_distance = 0.2
        place.post_place_retreat.direction.header.frame_id = "base_link"
        place.post_place_retreat.direction.vector.z = 1.0
        place.post_place_retreat.min_distance = 0.15
        place.post_place_retreat.desired_distance = 0.2

        rospy.loginfo("Placing object...")
        self.arm_group.place('object', place)
        rospy.loginfo("Object placed")

        self.pick_and_place_server.set_succeeded()

def set_scene(arm_handler):
    # Create a collision object
    collision_object = CollisionObject()
    collision_object.header.frame_id = arm_handler.arm_group.get_planning_frame()
    collision_object.id = 'object'

    # Define the shape and pose of the object
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.1, 0.1, 0.1]  # size of the box (1x1x1)
    collision_object.primitives = [primitive]

    object_pose = Pose()
    object_pose.orientation.w = 1.0
    object_pose.position.x = 0.3
    object_pose.position.y = 0.3
    object_pose.position.z = 0.3
    collision_object.primitive_poses = [object_pose]

    arm_handler.scene.add_object(collision_object)

if __name__ == '__main__':
    rospy.init_node('arm_handling_node')
    server = ArmHandlingNode()
    set_scene(server)
    # server.scan_area()
    action = PickAndPlaceAction()
    action.action_goal.goal.object.x = 0.3
    action.action_goal.goal.object.y = 0.3
    action.action_goal.goal.object.z = 0.3
    action.action_goal.goal.location.x = 0.4
    action.action_goal.goal.location.y = 0.4
    action.action_goal.goal.location.z = 0.3
    server.pick_and_place(action.action_goal.goal)
    rospy.spin()