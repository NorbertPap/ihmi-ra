#!/usr/bin/env python3
# arm_handling_node.py
import rospy
import actionlib
import numpy as np
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
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
        self.grasp_subscriber = None
        self.poses = {'pre_grasp': {
            'tennis_ball3': [1.719397670855674, -0.37333796563798316, 1.1480589423163279, -1.5512216011486712, -1.6184193665201825, 1.7093838222433422],
            'tennis_ball1': [1.4621943361296612, -0.39949488261506527, 1.1822399614392927, -1.551553583329822, -1.541284271903372, 1.4683210537520188],
            'tennis_ball2': [2.0359015926088553, -0.7001293993934592, 1.6228026660037917, -1.5757445328693178, -0.8222334495398576, 2.0285886602426797]
        }, 'grasp': {
            'tennis_ball3': [1.752853294926652, -0.7016125141214289, 1.6917573301975652, -1.540532750218083, -0.7462672902188947, 1.7206148048574659],
            'tennis_ball1': [1.4645398269322332, -0.6753798879100028, 1.6185939694013536, -1.5436089253880212, -0.8294487134368262, 1.453250586362636],
            'tennis_ball2': [2.0414828625790875, -0.7000550466485844, 1.621314322252199, -1.584253389913048, -0.8271538827803981, 2.0397546188127893]
        }, 'above_box': {
            'box_A': [0.8617535422236493, -0.2984684098336672, 1.2771823912654794, 0.005754851522628002, -0.7369024330701333, 0.010736561257947752],
            'box_B': [2.151555623414035, -0.1726417936390341, 1.8279398715736015, 2.547322387557106, -0.6864933296066082, -2.405678718595867]
        }}
        rospy.wait_for_service(rospy.get_namespace() + 'compute_ik')
        self.compute_ik_service = rospy.ServiceProxy(rospy.get_namespace() + 'compute_ik', GetPositionIK)

    def get_cartesian_pose(self):
        arm_group = self.arm_group
        pose = arm_group.get_current_pose()
        return pose.pose
    
    def get_ik_solution(self, pose):
        # Specify the desired position and orientation of the end effector
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"  # Replace with your base frame
        pose_stamped.pose = pose

        # Create an empty RobotState and fill it with the current state
        robot_state = arm_group.get_current_state()
        robot_state.joint_state.name = robot_state.joint_state.name[0:6]
        robot_state.joint_state.position = arm_group.get_current_joint_values()[0:6]

        # Create a PositionIKRequest object and fill it with the desired pose
        ik_request = PositionIKRequest()
        ik_request.group_name = "arm"  # Replace with your group name
        ik_request.robot_state = robot_state
        ik_request.pose_stamped = pose_stamped
        ik_request.timeout.secs = 5

        # Call the service and get the response
        response = self.compute_ik_service(ik_request)

        # The response contains the calculated joint values
        return response.solution.joint_state.position[0:6]

    def reach_cartesian_pose(self, pose, tolerance = 0, constraints = None):
        ik_solution = self.get_ik_solution(pose)
        arm_group.set_joint_value_target(ik_solution)
        return arm_group.go(wait=True)
    
    def reach_gripper_position(self, relative_position, wait = True):
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, wait)
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
        # solid.type = solid.SPHERE
        solid.type = solid.BOX
        solid.dimensions = [0.05, 0.05, 0.05]
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
        tennis_ball_2 = self.create_tennis_ball('tennis_ball2', self.get_pose_from_xyz_rpy(-0.1, 0.35, 0.035, 0, 0, 0).pose)
        tennis_ball_3 = self.create_tennis_ball('tennis_ball3', self.get_pose_from_xyz_rpy(0.0, 0.35, 0.035, 0, 0, 0).pose)
        box_A = self.create_box('box_with_walls_A', self.get_pose_from_xyz_rpy(0.2, 0.5, 0.0875, 0, 0, 0).pose)
        box_B = self.create_box('box_with_walls_B', self.get_pose_from_xyz_rpy(-0.2, 0.5, 0.0875, 0, 0, 0).pose)
        self.scene.add_object(tennis_ball_1)
        self.scene.add_object(tennis_ball_2)
        self.scene.add_object(tennis_ball_3)
        self.scene.add_object(box_A)
        self.scene.add_object(box_B)

    def pick(self, obj):
        self.reach_gripper_position(0.7)
        self.arm_group.set_joint_value_target(self.poses['pre_grasp'][obj])
        self.arm_group.go(wait=True)
        self.arm_group.set_joint_value_target(self.poses['grasp'][obj])
        self.arm_group.go(wait=True)
        self.arm_group.attach_object(obj, link_name='right_finger_dist_link', touch_links=['end_effector_link', 'right_finger_prox_link', 'right_finger_dist_link', 'left_finger_prox_link', 'left_finger_dist_link'])
        self.reach_gripper_position(0.3)

    def place(self, obj, location):
        self.arm_group.set_joint_value_target(self.poses['above_box'][location])
        self.arm_group.go(wait=True)
        self.reach_gripper_position(0.8)
        self.arm_group.detach_object(obj)



if __name__ == '__main__':
    rospy.init_node('arm_handling_node')
    server = ArmHandlingNode()
    server.set_the_scene()
    arm_group = server.arm_group
    arm_group.allow_replanning(True)
    server.reach_gripper_position(0.7)
    server.reach_cartesian_pose(server.get_pose_from_xyz_rpy(0.1, 0.35, 0.1, 0, np.pi, 0).pose)
    # arm_group.set_joint_value_target(ik_solution[0:6])
    # arm_group.go(wait=True)
    # server.reach_gripper_position(0.35)

    # server.scan_area()

    # server.reach_gripper_position(0.7)
    # arm_group.set_joint_value_target(server.poses['pre_grasp']['tennis_ball2'])
    # arm_group.go(wait=True)

    # arm_group.set_joint_value_target(server.poses['grasp']['tennis_ball3'])
    # arm_group.go(wait=True)

    # server.pick('tennis_ball3')
    # server.place('tennis_ball3', 'box_A')
    # server.pick('tennis_ball2')
    # server.place('tennis_ball2', 'box_B')

    rospy.spin()