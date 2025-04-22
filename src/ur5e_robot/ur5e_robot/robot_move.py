#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from my_robot_interfaces.msg import TargetCoordinates  # Import the TargetCoordinates message type


TRAJECTORIES = {
    "traj0": [
        {
            "positions": [-0.195016, -1.70094, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
            "time_from_start": Duration(sec=1, nanosec=0),
        },
        {
            "positions": [-1.5090117, -1.4238396,1.4371041,-0.1425934,0.05480334, 0.1377065],
            "velocities": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
            "time_from_start": Duration(sec=2, nanosec=0),
        },
    ],
    "traj1": [
        {
            "positions": [-1.5090117, -1.4238396,1.4371041,-0.1425934,0.05480334, 0.1377065],
            "velocities": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
            "time_from_start": Duration(sec=2, nanosec=0),
        },
        {
            "positions": [-1.2264429,-2.2776547,2.03889363,0.22095868,0.33981561,0.02722714],
            "velocities": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
            "time_from_start": Duration(sec=3, nanosec=0),
        },
    ],
    "traj2": [
        {
            "positions": [-1.2264429,-2.2776547,2.03889363,0.22095868,0.33981561,0.02722714],
            "velocities": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [-1.67552,-2.92709169,1.7259561,1.2074188,-0.11449936,0.00174533],
            "velocities": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
            "time_from_start": Duration(sec=5, nanosec=0),
        },
    ],
    "traj3": [
        {
           "positions": [-1.2264429,-2.2776547,2.03889363,0.22095868,0.33981561,0.02722714],
            "velocities": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
            "time_from_start": Duration(sec=7, nanosec=0),
        },
    ],
}

class JTCClient(Node):
    """Small test client for the jtc"""

    def __init__(self):
        super().__init__("jtc_client")
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.parse_trajectories()
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None
        self.execute_next_trajectory()
        self.subscription = self.create_subscription(TargetCoordinates,'topic',self.array_callback,10)

    def parse_trajectories(self):
        self.goals = {}

        for traj_name in TRAJECTORIES:
            goal = JointTrajectory()
            goal.joint_names = self.joints
            for pt in TRAJECTORIES[traj_name]:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"]
                point.velocities = pt["velocities"]
                point.time_from_start = pt["time_from_start"]
                goal.points.append(point)

            self.goals[traj_name] = goal

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):
            self.get_logger().info("Done with all trajectories")
            return
        traj_name = list(self.goals)[self.i]
        self.i = self.i + 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Done with result: {self.error_code_to_str(result.error_code)}")
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            time.sleep(5)
            self.execute_next_trajectory()

    def delay_between_points(self, traj_name):
        delay_time = 20 
        points = self.goals[traj_name].points
        for point in points:
            time.sleep(delay_time)        

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    def array_callback(self, msg):
        self.get_logger().info("Received array: [%d, %d, %d]" % (msg.var1, msg.var2, msg.var3))
        expired_trajectories = []

    
        for index, status in enumerate([msg.var1, msg.var2, msg.var3]):
            if status == 1:
                expired_trajectories.append(index)
                self.execute_expired_trajectory(index)

        self.get_logger().info(f"Expired trajectories: {expired_trajectories}")

    def execute_expired_trajectory(self, index):
        expired_trajectories = {
            0: "traj_expired1",
            1: "traj_expired2",
            2: "traj_expired3"
        }
    
    
        expired_traj_name = expired_trajectories.get(index)
        if expired_traj_name:
            self.get_logger().info(f"Executing expired trajectory {index}: {expired_traj_name}")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = self.joints

            if expired_traj_name == "traj_expired1":
                positions = [-0.9738937,-0.80040799,0.25115288,0.53843407,0.58712876,0.00698132]
                velocities = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
                time_from_start = Duration(sec=2, nanosec=0)

            elif expired_traj_name == "traj_expired2":
                positions= [-0.35499997,-1.4179055,1.1869984,0.22881266,1.2082914,0.002268928]
                velocities= [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
                time_from_start= Duration(sec=2, nanosec=0)


            elif expired_traj_name == "traj_expired3":
                positions= [0.4758796,-1.092227,0.99326688,0.1052434,2.03889363,0.01937315]
                velocities= [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
                time_from_start= Duration(sec=2, nanosec=0)

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.time_from_start = time_from_start
            goal.trajectory.points.append(point)
            self.get_logger().info(f"Sending goal for expired trajectory {index}: {expired_traj_name}")
            self._send_goal_future = self._action_client.send_goal_async(goal)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().warning(f"Expired trajectory {index} not found in the dictionary.")



def main(args=None):
    rclpy.init(args=args)

    jtc_client = JTCClient()
    rclpy.spin(jtc_client)

    rclpy.shutdown()


if __name__ == "__main__":
    main()