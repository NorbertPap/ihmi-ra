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
from p6.msg import ScanAreaAction, ScanAreaResult, ScanAreaGoal, ScanAreaFeedback, PickAndPlaceAction, PickAndPlaceActionGoal
import tf.transformations as tf_trans
from kortex_driver.srv import SendGripperCommand, SendGripperCommandRequest
from kortex_driver.msg import Finger, GripperMode

class ArmHandlingNode:
    def __init__(self):
        # Create the MoveItInterface necessary objects
        self.arm_group = MoveGroupCommander("arm", ns=rospy.get_namespace())
        self.arm_group.allow_replanning(True)
        self.arm_group.set_end_effector_link("end_effector_link")
        self.robot = RobotCommander("robot_description")
        self.scene = PlanningSceneInterface(ns=rospy.get_namespace())
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
        self.scan_area_server = actionlib.SimpleActionServer('scan_area', ScanAreaAction, execute_cb=self.scan_area_cb, auto_start=False)
        self.pick_and_place_server = actionlib.SimpleActionServer('pick_and_place', PickAndPlaceAction, self.pick_and_place, False)
        self.scan_area_server.start()
        self.pick_and_place_server.start()
        self.grasp_subscriber = None
        
        rospy.wait_for_service(rospy.get_namespace() + 'compute_ik')
        self.compute_ik_service = rospy.ServiceProxy(rospy.get_namespace() + 'compute_ik', GetPositionIK)

        send_gripper_command_full_name = '/my_gen3_lite/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.gripper_command_interface = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        self.locations = {
            'tennis_ball1': [0.1, 0.35, 0.025],
            'tennis_ball2': [-0.1, 0.35, 0.025],
            'tennis_ball3': [0.0, 0.35, 0.025],
            'box_A': [0.2, 0.5, 0.0875],
            'box_B': [-0.2, 0.5, 0.0875]
        }
        self.sizes = {
            'tennis_ball1': [0.05, 0.05, 0.05],
            'tennis_ball2': [0.05, 0.05, 0.05],
            'tennis_ball3': [0.05, 0.05, 0.05],
            'box_A': [0.2, 0.2, 0.175],
            'box_B': [0.2, 0.2, 0.175]
        }

    def scan_area_cb(self, goal):
        scan_coverage_ratio = goal.scan_coverage_ratio
        try:
            self.scan_area(scan_coverage_ratio, self.publish_scan_area_feedback)
        except Exception as e:
            rospy.logerr("Failed to scan the area")
            result.success = False
            self.scan_area_server.set_succeeded(result)
            return
        result = ScanAreaResult()
        result.success = True
        self.scan_area_server.set_succeeded(result)

    def publish_scan_area_feedback(self, status_text):
        feedback = ScanAreaFeedback()
        feedback.status_text = status_text
        self.scan_area_server.publish_feedback(feedback)

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
        robot_state = self.arm_group.get_current_state()
        robot_state.joint_state.name = robot_state.joint_state.name[0:6]
        robot_state.joint_state.position = self.arm_group.get_current_joint_values()[0:6]

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

    def reach_cartesian_pose(self, pose, cartesian_path = False, constraints = None):
        # if pose is PoseStamped, get the Pose
        if isinstance(pose, PoseStamped):
            pose = pose.pose
        if cartesian_path:
            self.arm_group.set_pose_target(pose)
        else:
            ik_solution = self.get_ik_solution(pose)
            self.arm_group.set_joint_value_target(ik_solution)
        self.arm_group.set_path_constraints(constraints)
        return self.arm_group.go(wait=True)
    
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

        quat = tf_trans.quaternion_from_euler(ro, pi, ya, axes='rxyz')

        orientation = pose.pose.orientation
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]

        return pose
    
    def print_current_pose(self):
        current_pose = self.get_cartesian_pose()
        current_position = current_pose.position
        current_quaternion = current_pose.orientation
        current_rpy = tf_trans.euler_from_quaternion([current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w])
        current_rpy = [angle*180/np.pi for angle in current_rpy]
        current_rpy = [f'roll: {current_rpy[0]}', f'pitch: {current_rpy[1]}', f'yaw: {current_rpy[2]}']
        print('Current pose: ' + str(current_position) + ' ' + str(current_rpy))

    def create_scanning_constraint(self):
        # Create a constraint for the end effector to remain within a small volume
        constraint = moveit_msgs.msg.Constraints()
        position_constraint = moveit_msgs.msg.PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "end_effector_link"
        position_constraint.weight = 1.0
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.constraint_region.primitive_poses.append(self.get_cartesian_pose())

        sphere = SolidPrimitive()
        sphere.type = sphere.SPHERE
        sphere.dimensions = [0.1]

        position_constraint.constraint_region.primitives.append(sphere)
        
        constraint.position_constraints.append(position_constraint)
        
        return constraint

    def apply_pitch(self, pose, pitch):
        if isinstance(pose, PoseStamped):
            pose = pose.pose
        orientation_quaternion = pose.orientation
        # Convert the quaternion to a transformation matrix
        current_matrix = tf_trans.quaternion_matrix([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
        # Apply the pitch
        pitch_matrix = tf_trans.rotation_matrix(pitch, [0, 1, 0])
        new_matrix = np.dot(current_matrix, pitch_matrix)
        new_orientation = tf_trans.quaternion_from_matrix(new_matrix)
        pose.orientation.x = new_orientation[0]
        pose.orientation.y = new_orientation[1]
        pose.orientation.z = new_orientation[2]
        pose.orientation.w = new_orientation[3]
        return pose
    
    def get_orientation_from_rpy(self, roll, pitch, yaw):
        orientation = tf_trans.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')
        quaternion = Pose().orientation
        quaternion.x = orientation[0]
        quaternion.y = orientation[1]
        quaternion.z = orientation[2]
        quaternion.w = orientation[3]
        return quaternion

    def scan_area(self, scan_coverage_ratio, feedback_cb = lambda msg: rospy.loginfo(msg)):
        feedback_cb("Starting scanning...")
        feedback_cb("Moving to home position")
        self.arm_group.set_named_target('home')
        self.arm_group.go(wait=True)
        feedback_cb("Reached home")

        home_pose = self.get_cartesian_pose()
        constraints = self.create_scanning_constraint()
        euler_angles = tf_trans.euler_from_quaternion([home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z, home_pose.orientation.w])
        turn_angle = 2 * np.pi * scan_coverage_ratio / 2

        feedback_cb("Going left")
        pose_left = self.get_pose_from_xyz_rpy(
            home_pose.position.x,
            home_pose.position.y,
            home_pose.position.z, 
            euler_angles[0],
            euler_angles[1],
            euler_angles[2] + turn_angle)
        pose_left = self.apply_pitch(pose_left, np.pi/8)
        self.reach_cartesian_pose(pose_left, cartesian_path=True, constraints=constraints)
        feedback_cb("Reached left")

        feedback_cb("Going right")
        pose_right = self.get_pose_from_xyz_rpy(
            home_pose.position.x,
            home_pose.position.y,
            home_pose.position.z, 
            euler_angles[0],
            euler_angles[1],
            euler_angles[2] - turn_angle)
        pose_right = self.apply_pitch(pose_right, np.pi/8)
        self.reach_cartesian_pose(pose_right, cartesian_path=True, constraints=constraints)
        feedback_cb("Reached right")

        feedback_cb("Going center")
        looking_down = self.apply_pitch(home_pose, np.pi/8)
        self.reach_cartesian_pose(looking_down, cartesian_path=True, constraints=constraints)
        feedback_cb("Reached center")

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
        tennis_ball_1 = self.create_tennis_ball('tennis_ball1', self.get_pose_from_xyz_rpy(0.1, 0.35, 0.025, 0, 0, 0).pose)
        tennis_ball_2 = self.create_tennis_ball('tennis_ball2', self.get_pose_from_xyz_rpy(-0.1, 0.35, 0.025, 0, 0, 0).pose)
        tennis_ball_3 = self.create_tennis_ball('tennis_ball3', self.get_pose_from_xyz_rpy(0.0, 0.35, 0.025, 0, 0, 0).pose)
        box_A = self.create_box('box_with_walls_A', self.get_pose_from_xyz_rpy(0.2, 0.5, 0.0875, 0, 0, 0).pose)
        box_B = self.create_box('box_with_walls_B', self.get_pose_from_xyz_rpy(-0.2, 0.5, 0.0875, 0, 0, 0).pose)
        self.scene.add_object(tennis_ball_1)
        self.scene.add_object(tennis_ball_2)
        self.scene.add_object(tennis_ball_3)
        self.scene.add_object(box_A)
        self.scene.add_object(box_B)

    def attach_object(self, obj):
        self.arm_group.attach_object(obj, link_name='end_effector_link',
                                     touch_links=['end_effector_link',
                                                  'right_finger_prox_link',
                                                  'right_finger_dist_link',
                                                  'left_finger_prox_link',
                                                  'left_finger_dist_link'])

    def pick(self, obj):
        orientation_rpy = [0, np.pi, 0]
        self.reach_gripper_position(0.7)

        pre_grasp_pose = self.get_pose_from_xyz_rpy(*self.locations[obj], *orientation_rpy).pose
        pre_grasp_pose.position.z += 0.1 # 10 cm above the object
        self.reach_cartesian_pose(pre_grasp_pose)

        grasp_pose = self.get_pose_from_xyz_rpy(*self.locations[obj], *orientation_rpy).pose
        grasp_pose.position.z  = np.max([grasp_pose.position.z, 0.04]) # grasp object at its center, a minimum of 4cm along the z-axis to avoid collision with the ground
        self.reach_cartesian_pose(grasp_pose)

        self.attach_object(obj)
        self.reach_gripper_position(0.45)

    def place(self, obj, location):
        place_pose = self.get_pose_from_xyz_rpy(*self.locations[location], np.pi/2, np.pi/2, 0).pose # yaw could be anything but there is no way to define that, so we set it to 0
        place_pose.position.z += self.sizes[location][2] + 0.1 # a whole object height above the box plus 10cm to avoid collision
        self.reach_cartesian_pose(place_pose)

        self.reach_gripper_position(0.8)
        self.arm_group.detach_object(obj)

    def send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.gripper_command_interface(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            rospy.sleep(0.5)
            return True



if __name__ == '__main__':
    rospy.init_node('arm_handling_node')
    server = ArmHandlingNode()
    server.set_the_scene()
    
    # server.scan_area(0.25)

    server.pick('tennis_ball3')
    server.place('tennis_ball3', 'box_A')
    # server.pick('tennis_ball1')
    # server.place('tennis_ball1', 'box_B')
    # server.pick('tennis_ball2')
    # server.place('tennis_ball2', 'box_B')

    rospy.spin()