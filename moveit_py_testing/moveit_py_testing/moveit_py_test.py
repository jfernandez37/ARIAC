#!/usr/bin/env python3

from copy import copy
from math import e, pi
from multiprocessing import get_logger
from moveit import PlanningSceneMonitor
import rclpy
import pyassimp
import trimesh
import tf2_ros
from rclpy.node import Node
from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.planning import (
    MoveItPy,
)
from rclpy.logging import get_logger
from time import sleep
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh,MeshTriangle

from ament_index_python.packages import get_package_share_directory
bin_positions = [(x,y) 
                 for x in [-1.9,-2.65]
                 for y in [3.375,2.625,-3.375,-2.625]]
bin_positions_dict = {f"bin{i+1}":bin_positions[i] for i in range(len(bin_positions))}

class Error(Exception):
  def __init__(self, value: str):
      self.value = value

  def __str__(self):
      return repr(self.value)

class MoveItNode(Node):
    def __init__(self):
        super().__init__('moveit_py_node')

        self.get_cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.get_position_fk_client = self.create_client(GetPositionFK, "compute_fk")
    
    def call_get_cartesian_path (self, 
                                 header : Header,
                                 start_state : RobotState, 
                                 group_name : str, 
                                 link_name : str, 
                                 waypoints : list,
                                 max_step : float):

        self.get_logger().info("Getting cartesian path")

        request = GetCartesianPath.Request()

        request.header = header
        request.start_state = start_state
        request.group_name = group_name
        request.link_name = link_name
        request.waypoints = waypoints
        request.max_step = max_step

        future = self.get_cartesian_path_client.call_async(request)


        rclpy.spin_until_future_complete(self, future, timeout_sec=10)

        if not future.done():
            raise Error("Timeout reached when calling move_cartesian service")

        result: GetCartesianPath.Response
        result = future.result()

        return result.solution

    def call_get_position_fk (self, 
                              header : Header,
                              link_joint_names : list, 
                              robot_state : RobotState
                             ):

        self.get_logger().info("Getting cartesian path")

        request = GetPositionFK.Request()

        request.header = header
        request.fk_link_names = link_joint_names
        request.robot_state = robot_state

        future = self.get_position_fk_client.call_async(request)


        rclpy.spin_until_future_complete(self, future, timeout_sec=10)

        if not future.done():
            raise Error("Timeout reached when calling move_cartesian service")

        result: GetPositionFK.Response
        result = future.result()

        return result.pose_stamped

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()
    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        logger.info(str(robot_trajectory.joint_model_group_name))
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    sleep(sleep_time)

def makeMesh(name, pose, filename, logger) -> CollisionObject:
    # mesh = trimesh.load(filename)
    # mesh.vertices
    # mesh.triangles  
    with pyassimp.load(filename) as scene:
        assert len(scene.meshes)
        
        mesh = Mesh()
        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if hasattr(face, 'indices'):
                if len(face.indices) == 3:
                    triangle.vertex_indices = [face.indices[0],
                                                face.indices[1],
                                                face.indices[2]]
            else:
                if len(face) == 3:
                    triangle.vertex_indices = [face[0],
                                                face[1],
                                                face[2]]
            mesh.triangles.append(triangle)
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = float(vertex[0])
            point.y = float(vertex[1])
            point.z = float(vertex[2])
            mesh.vertices.append(point)
        
    o = CollisionObject()
    o.header.frame_id = "world"
    o.id = name
    o.meshes.append(mesh)
    o.mesh_poses.append(pose)
    o.operation = o.ADD
    return o

def add_model_to_planning_scene(name : str,
                                mesh_file : str,
                                logger,
                                planning_scene_monitor: PlanningSceneMonitor,
                                model_pose : Pose
                                ):
    
    package_share_directory = get_package_share_directory("moveit_py_testing")
    model_path = package_share_directory + "/meshes/"+mesh_file
    collision_object = makeMesh(name, model_pose,model_path, logger)
    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(collision_object)
        scene.current_state.update()

def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("moveit_py_test")
    node = MoveItNode()

    robot = MoveItPy(node_name="moveit_py_test")
    
    floor_robot = robot.get_planning_component("floor_robot")
    ceiling_robot = robot.get_planning_component("ceiling_robot")
    sleep(5.00)
    
    # # -------------------------------------------------------
    # #                         Test 1
    # # -------------------------------------------------------
    floor_robot.set_start_state_to_current_state()
    floor_robot.set_goal_state(configuration_name="home")
    plan_and_execute(robot, floor_robot, logger, sleep_time=0.0)
    ceiling_robot.set_start_state_to_current_state()
    ceiling_robot.set_goal_state(configuration_name = "home")
    plan_and_execute(robot, ceiling_robot, logger, sleep_time=0.0)

    robot_model = robot.get_robot_model()
    planning_scene_monitor = robot.get_planning_scene_monitor()
    for i in range(1,len(bin_positions)+1):
        bin_pose = Pose()
        bin_pose.position.x = float(bin_positions_dict[f"bin{i}"][0])
        bin_pose.position.y = float(bin_positions_dict[f"bin{i}"][1])
        bin_pose.position.z = 0.0
        # quaternion
        bin_pose.orientation.x= 0.0
        bin_pose.orientation.y = 0.0
        bin_pose.orientation.z = 0.0
        bin_pose.orientation.w = 1.0
        add_model_to_planning_scene(f"bin{i}","bin.stl",logger, planning_scene_monitor, bin_pose)
    logger.info("Finished adding bins to planning scene")

    # floor_robot.set_start_state_to_current_state()
    # floor_robot.set_goal_state(configuration_name="extended")
    # plan_and_execute(robot, floor_robot, logger, sleep_time=3.0)
    

    # # -------------------------------------------------------
    # #                         Test 2
    # # -------------------------------------------------------
    # # for i in range(10):
    # #     print(i)
    # #     robot_model = robot.get_robot_model()
    # #     robot_state = RobotState(robot_model)

    # #     robot_state.set_to_random_positions()

    # #     # set plan start state to current state
    # #     floor_robot.set_start_state_to_current_state()

    # #     # set goal state to the initialized robot state
    # #     logger.info("Set goal state to the initialized robot state")
    # #     floor_robot.set_goal_state(robot_state=robot_state)

    # #     # plan to goal
    # #     plan_and_execute(robot, floor_robot, logger, sleep_time=3.0)

    # # -------------------------------------------------------
    # #                         Test 3
    # # -------------------------------------------------------
    with planning_scene_monitor.read_write() as scene:
        robot_state = scene.current_state
        robot_state.update()
        # set plan start state to current state
        floor_robot.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "floor_base_link_inertia"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.9
        pose_goal.pose.position.y = -0.3
        pose_goal.pose.position.z = 0.5
        floor_robot.set_goal_state(pose_stamped_msg=pose_goal, pose_link="floor_gripper")

        # plan to goal
    plan_and_execute(robot, floor_robot, logger)

    # with planning_scene_monitor.read_write() as scene:

    #     # instantiate a RobotState instance using the current robot model
    #     robot_state = scene.current_state
    #     robot_state.update()
    #     # header
    #     sleep(1.0)
    #     header = Header()
    #     header.frame_id = "world"
    #     header.stamp = node.get_clock().now().to_msg()

    #     # link model names
    #     link_model_names = robot.get_robot_model().get_joint_model_group("floor_robot").link_model_names
    #     link_model_names = ["floor_gripper"]

    #     # Robot State
    #     sleep(1.0)
        
        
    #     robot_state_msg = robotStateToRobotStateMsg(robot_state)
    #     robot_state.update()

    #     fk_posestamped = node.call_get_position_fk(header, link_model_names, robot_state_msg)

    #     # header
    #     sleep(1.0)
    #     header = Header()
    #     header.frame_id = "world"
    #     header.stamp = node.get_clock().now().to_msg()

    #     # Group name
    #     group_name = "floor_robot"

    #     # Link name
    #     link_name = "floor_gripper"

    #     # Waypoints
    #     logger.info(str(len(fk_posestamped)))
    #     current_pose = fk_posestamped[-1].pose
    #     logger.info("Current position: " + str(current_pose.position))
    #     desired_pose = copy(current_pose)
    #     logger.info("before: "+str(desired_pose))
    #     desired_pose.position.x-=0.1
    #     desired_pose.position.y-=3
    #     desired_pose.position.z-=0.1
    #     logger.info("after: "+str(desired_pose))
    #     desired_pose_2 = copy(desired_pose)
    #     desired_pose.position.x-=0.3
    #     desired_pose.position.y+=6
    #     desired_pose.position.z+=0.25
    #     waypoints = [desired_pose]

        

    #     # Max step
    #     max_step = 0.1
    #     robot_state.update()
    #     robot_state_msg = robotStateToRobotStateMsg(robot_state)
    #     trajectory_msg = node.call_get_cartesian_path(header,
    #                                 robot_state_msg,
    #                                 group_name,
    #                                 link_name,
    #                                 waypoints,
    #                                 max_step)
    #     robot_state.update()
    #     trajectory = RobotTrajectory(robot_model)
    #     trajectory.set_robot_trajectory_msg(robot_state, trajectory_msg)
    #     trajectory.joint_model_group_name = "floor_robot"
    # robot_state.update(True)
    # robot.execute(trajectory, controllers=[])

    # with planning_scene_monitor.read_write() as scene:

    #     # instantiate a RobotState instance using the current robot model
    #     robot_state = scene.current_state
    #     robot_state.update()
    #     # header
    #     sleep(1.0)
    #     header = Header()
    #     header.frame_id = "world"
    #     header.stamp = node.get_clock().now().to_msg()

    #     # link model names
    #     link_model_names = robot.get_robot_model().get_joint_model_group("floor_robot").link_model_names
    #     link_model_names = ["floor_gripper"]

    #     # Robot State
    #     sleep(1.0)
        
        
    #     robot_state_msg = robotStateToRobotStateMsg(robot_state)
    #     robot_state.update()
    #     logger.info("\n".join(str(robotStateToRobotStateMsg(robot_state)).split(",")))

    #     fk_posestamped = node.call_get_position_fk(header, link_model_names, robot_state_msg)

    #     # header
    #     sleep(1.0)
    #     header = Header()
    #     header.frame_id = "world"
    #     header.stamp = node.get_clock().now().to_msg()

    #     # Group name
    #     group_name = "floor_robot"

    #     # Link name
    #     link_name = "floor_gripper"

    #     # Waypoints
    #     logger.info(str(len(fk_posestamped)))
    #     current_pose = fk_posestamped[-1].pose
    #     logger.info("Current position: " + str(current_pose.position))
    #     desired_pose = copy(current_pose)
    #     logger.info("before: "+str(desired_pose))
    #     # desired_pose.position.x-=0.1
    #     desired_pose.position.y-=0.5
    #     # desired_pose.position.z-=0.1
    #     logger.info("after: "+str(desired_pose))
    #     waypoints = [desired_pose]

        

    #     # Max step
    #     max_step = 0.1
    #     robot_state.update()
    #     robot_state_msg = robotStateToRobotStateMsg(robot_state)
    #     trajectory_msg = node.call_get_cartesian_path(header,
    #                                 robot_state_msg,
    #                                 group_name,
    #                                 link_name,
    #                                 waypoints,
    #                                 max_step)
    #     robot_state.update()
    #     trajectory = RobotTrajectory(robot_model)
    #     trajectory.set_robot_trajectory_msg(robot_state, trajectory_msg)
    #     trajectory.joint_model_group_name = "floor_robot"
    # robot_state.update(True)
    # robot.execute(trajectory, controllers=[])
    # try:
    #     while(True):
    #         sleep(1)
    # except KeyboardInterrupt:
        # (int)
    rclpy.shutdown()

if __name__ == "__main__":
	main()