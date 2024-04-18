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
from trajectory_msgs.msg import JointTrajectoryPoint
from p6.msg import ScanAreaAction, PickAndPlaceAction
import tf.transformations as tf_trans

class ArmHandlingNode:
    def __init__(self):
        # Create the MoveItInterface necessary objects
        self.arm_group = MoveGroupCommander("arm", ns=rospy.get_namespace())
        self.arm_group.set_end_effector_link("end_effector_link")
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
        
    def get_gripper_absolute_position(self, relative_position):
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        return relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos

    def get_pose_from_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
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

        collision_object = self.create_collision_object('cylinder', object_pose.pose, [1, 0.02])
        # self.scene.add_object(collision_object)
        self.scene.add_cylinder('cylinder', object_pose, 1, 0.03)
        self.arm_group.attach_object('cylinder', link_name='end_effector_link', touch_links=self.robot.get_link_names("arm") + self.robot.get_link_names("gripper"))

        grasp = moveit_msgs.msg.Grasp()
        grasp.max_contact_force = 1

        grasp.grasp_pose = self.get_pose_from_xyz_rpy(goal.object.x, goal.object.y, goal.object.z, 0, np.pi/2, 0)
        grasp.grasp_pose.header.frame_id = "base_link"

        grasp.pre_grasp_approach.direction.header.frame_id = "base_link"
        grasp.pre_grasp_approach.direction.vector.z = -1.0
        grasp.pre_grasp_approach.min_distance = 0.15
        grasp.pre_grasp_approach.desired_distance = 0.2

        grasp.pre_grasp_posture.header.frame_id = "end_effector_link"
        grasp.pre_grasp_posture.joint_names = [self.gripper_joint_name]
        grasp.pre_grasp_posture.points.append(JointTrajectoryPoint())
        grasp.pre_grasp_posture.points[0].positions = [self.get_gripper_absolute_position(1)]
        grasp.pre_grasp_posture.points[0].time_from_start = rospy.Duration(0.5)

        grasp.grasp_posture.header.frame_id = "end_effector_link"
        grasp.grasp_posture.joint_names = [self.gripper_joint_name]
        grasp.grasp_posture.points.append(JointTrajectoryPoint())
        grasp.grasp_posture.points[0].positions = [self.get_gripper_absolute_position(0.2)]
        grasp.grasp_posture.points[0].effort = [1]
        grasp.grasp_posture.points[0].time_from_start = rospy.Duration(0.4)

        grasp.post_grasp_retreat.direction.header.frame_id = "base_link"
        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.15
        grasp.post_grasp_retreat.desired_distance = 0.2

        grasp.allowed_touch_objects = ['cylinder']

        rospy.loginfo("Picking object...")
        # self.arm_group.allow_replanning(True)
        # self.arm_group.attach_object('cylinder')
        self.arm_group.pick('cylinder', grasp)
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

        # rospy.loginfo("Placing object...")
        # self.arm_group.place('ball', place)
        # rospy.loginfo("Object placed")

        self.arm_group.detach_object('cylinder')
        self.scene.remove_world_object('cylinder')

        self.pick_and_place_server.set_succeeded()

    def create_collision_object(self, id, pose, dimensions):
        object = CollisionObject()
        object.id = id
        object.header.frame_id = self.arm_group.get_planning_frame()

        solid = SolidPrimitive()
        solid.type = solid.CYLINDER
        solid.dimensions = dimensions
        object.primitives = [solid]

        object_pose = pose

        object.primitive_poses = [object_pose]
        object.operation = object.ADD
        return object
    
    def create_tennis_ball(self, id, pose):
        object = CollisionObject()
        object.id = id
        object.header.frame_id = self.arm_group.get_planning_frame()

        solid = SolidPrimitive()
        solid.type = solid.SPHERE
        solid.dimensions = [0.035]
        object.primitives = [solid]

        object.primitive_poses = [pose]
        object.operation = object.ADD
        return object
    
    def create_box(self, id, pose):
        object = CollisionObject()
        object.id = id
        object.header.frame_id = self.arm_group.get_planning_frame()

        solid = SolidPrimitive()
        solid.type = solid.BOX
        solid.dimensions = [0.2, 0.2, 0.175]
        object.primitives = [solid]

        object_pose = pose

        object.primitive_poses = [object_pose]
        object.operation = object.ADD
        return object
    
    def set_the_scene(self):
        tennis_ball_1 = self.create_tennis_ball('tennis_ball1', self.get_pose_from_xyz_rpy(0.1, 0.35, 0.035, 0, 0, 0).pose)
        tennis_ball_2 = self.create_tennis_ball('tennis_ball2', self.get_pose_from_xyz_rpy(0.0, 0.35, 0.035, 0, 0, 0).pose)
        tennis_ball_3 = self.create_tennis_ball('tennis_ball3', self.get_pose_from_xyz_rpy(-0.1, 0.35, 0.035, 0, 0, 0).pose)
        box_A = self.create_box('box_with_walls_A', self.get_pose_from_xyz_rpy(0.2, 0.5, 0.0875, 0, 0, 0).pose)
        box_B = self.create_box('box_with_walls_B', self.get_pose_from_xyz_rpy(-0.2, 0.5, 0.0875, 0, 0, 0).pose)
        self.scene.add_object(tennis_ball_1)
        self.scene.add_object(tennis_ball_2)
        self.scene.add_object(tennis_ball_3)
        self.scene.add_object(box_A)
        self.scene.add_object(box_B)


if __name__ == '__main__':
    rospy.init_node('arm_handling_node')
    server = ArmHandlingNode()
    server.set_the_scene()
    # server.scan_area()
    # action = PickAndPlaceAction()
    # action.action_goal.goal.object.x = 0.075319
    # action.action_goal.goal.object.y = 0.290744
    # action.action_goal.goal.object.z = 0.5
    # action.action_goal.goal.location.x = 0.4
    # action.action_goal.goal.location.y = 0.4
    # action.action_goal.goal.location.z = 0.3
    # server.pick_and_place(action.action_goal.goal)

    # server.reach_cartesian_pose(server.get_pose_from_xyz_rpy(0, 0.35, 0.2, np.pi, 0, 0))

    arm_group = server.arm_group
    server.arm_group.attach_object('tennis_ball2', link_name='end_effector_link')
    server.reach_gripper_position(0.7)
    arm_group.set_joint_value_target([-1.7789422557452337, 0.5557483356470527, -1.7313943976828678, -1.5442874257575543, 0.8456311348001027, -0.21382284220865833]) # almost grabbing
    arm_group.go(wait=True)
    arm_group.set_joint_value_target([-1.7766088053169735, 0.6525358609551581, -1.762790101435268, -1.5416014810317904, 0.7175798601998045, -0.21642575960469612]) # grabbing
    arm_group.go(wait=True)
    server.reach_gripper_position(0.56)
    
    arm_group.set_joint_value_target([1.8342356734062477, -0.10681195813735034, 0.8543174765239172, -1.5545036836367894, -2.163119937459122, 1.857624550919922]) # looking down
    arm_group.go(wait=True)


    rospy.spin()