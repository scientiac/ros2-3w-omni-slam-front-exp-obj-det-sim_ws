# Copyright 2025 Zhi Yan @ ENSTA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool

import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

class FrontierExplorationNode(Node):
    """
    A simple frontier-based exploration node.
    """

    def __init__(self):
        super().__init__('frontier_exploration_node')

        # === Parameters ===
        param = self.get_parameter_or('use_sim_time', rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False))
        self.use_sim_time = param.get_parameter_value().bool_value

        # === State Variables ===
        self.map_data = None
        self.map_resolution = 0.0
        self.map_origin = Point()
        self.robot_pose = None
        self.is_navigating = False

        self.invalid_frontiers = set()
        self.completion_publisher = self.create_publisher(Bool, '/exploration/complete', 10)

        # === ROS QoS ===
        # QoS for /map. It must be latching (transient local) to get the map.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # === Subscriptions ===
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # or /odometry/filtered
            self.odom_callback,
            10
        )

        # === Publishers ===
        self.frontier_marker_publisher = self.create_publisher(
            Marker,
            '/exploration_frontiers',
            10
        )

        # === Action Client ===
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # === TF Listener ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # === Main Timer ===
        # Timer to check every 2 seconds if a new goal should be sent
        self.timer = self.create_timer(2.0, self.exploration_timer_callback)

        self.get_logger().info("Frontier Exploration Node started.")
        self.get_logger().info(f"Using simulation time: {self.use_sim_time}")

    def map_callback(self, msg: OccupancyGrid):
        """
        Stores the latest map data.
        """
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        # self.get_logger().info("Map received.")

    def odom_callback(self, msg: Odometry):
        """
        Stores the latest robot pose.
        """
        self.robot_pose = msg.pose.pose
    
    def get_robot_pose_from_tf(self):
        """
        (Fallback method) Tries to get the TF transform from /map -> /base_link.
        /odom is usually sufficient in simulation.
        """
        if self.use_sim_time:
            # In simulation, TF sometimes has a delay
            pass

        try:
            # Get the transform from the 'map' frame to the 'base_link' frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            self.robot_pose = pose.pose
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get TF transform from 'map' to 'base_link': {e}")
            return False


    def exploration_timer_callback(self):
        """
        Main loop: checks state, finds frontiers, and sends a goal.
        """
        # 1. Check prerequisites
        if self.is_navigating:
            self.get_logger().info("Currently navigating to a goal, skipping this cycle.")
            return

        if self.map_data is None:
            self.get_logger().warn("Map has not been received yet, cannot explore.")
            return

        # Try to update TF (more reliable)
        if not self.get_robot_pose_from_tf():
             if self.robot_pose is None:
                self.get_logger().warn("Robot pose (Odom/TF) has not been received yet, cannot explore.")
                return
             else:
                self.get_logger().warn("TF retrieval failed, will use /odom.")
                
        # 2. Find frontiers
        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().info("No more frontiers found, exploration complete!")
            # Can stop the timer or shut down the node here

            msg = Bool()
            msg.data = True
            self.completion_publisher.publish(msg)

            self.timer.cancel()
            return

        # 3. Select the best frontier
        # Simplified version: choose the closest frontier point
        best_frontier = self.find_closest_frontier(frontiers)

        # 4. Convert the frontier point to world coordinates
        goal_pose = self.frontier_to_goal_pose(best_frontier)
        self.get_logger().info(f"Found new frontier, goal: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")

        # 5. Send the goal to Nav2
        self.send_navigation_goal(goal_pose)

    def find_frontiers(self) -> list:
        """
        NumPy-only frontier detection with connected frontier grouping.
        Drop-in replacement with wall-proximity rejection and frontier invalidation.
        Returns a list of frontier centroids: [[y, x], ...]
        """

        if self.map_data is None:
            return []

        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape((height, width))

        unknown = (data == -1)
        free = (data == 0)
        occupied = (data > 50)

        # --- find frontier mask ---
        adj_free = np.zeros_like(free, dtype=bool)
        adj_free[1:, :] |= free[:-1, :]
        adj_free[:-1, :] |= free[1:, :]
        adj_free[:, 1:] |= free[:, :-1]
        adj_free[:, :-1] |= free[:, 1:]
        adj_free[1:, 1:] |= free[:-1, :-1]
        adj_free[1:, :-1] |= free[:-1, 1:]
        adj_free[:-1, 1:] |= free[1:, :-1]
        adj_free[:-1, :-1] |= free[1:, 1:]

        frontier_mask = unknown & adj_free

        if not frontier_mask.any():
            return []

        visited = np.zeros_like(frontier_mask, dtype=bool)
        frontiers = []

        neighbors = [(-1, -1), (-1, 0), (-1, 1),
                     (0, -1),          (0, 1),
                     (1, -1),  (1, 0),  (1, 1)]

        # --- helper: wall proximity check ---
        def near_wall(y, x, r=2):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        if occupied[ny, nx]:
                            return True
            return False

        # --- BFS connected components ---
        for y, x in np.argwhere(frontier_mask):
            if visited[y, x]:
                continue

            queue = [(y, x)]
            visited[y, x] = True
            cluster = []

            while queue:
                cy, cx = queue.pop()
                cluster.append((cy, cx))

                for dy, dx in neighbors:
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        if frontier_mask[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            queue.append((ny, nx))

            # Ignore tiny noisy clusters
            if len(cluster) < 5:
                continue

            # centroid
            cluster_np = np.array(cluster)
            cy, cx = cluster_np.mean(axis=0).astype(int)

            key = (int(cy), int(cx))

            # --- NEW: reject invalid frontiers ---
            if key in self.invalid_frontiers:
                continue

            if near_wall(cy, cx):
                self.invalid_frontiers.add(key)
                continue

            frontiers.append([key[0], key[1]])

        self.publish_frontier_markers(frontiers)
        return frontiers

    def find_closest_frontier(self, frontiers: list) -> list:
        """
        Finds the closest frontier point to the robot from the list.
        """
        min_dist_sq = float('inf')
        closest_frontier = None

        # Convert robot world coordinates to map coordinates (x, y) -> (col, row)
        robot_col = int((self.robot_pose.position.x - self.map_origin.x) / self.map_resolution)
        robot_row = int((self.robot_pose.position.y - self.map_origin.y) / self.map_resolution)

        MIN_DIST_SQUARED = 10**2
        
        for (row, col) in frontiers:
            dist_sq = (row - robot_row)**2 + (col - robot_col)**2

            if dist_sq < MIN_DIST_SQUARED:
                continue

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_frontier = [row, col] # [y, x]
                
        return closest_frontier

    def frontier_to_goal_pose(self, frontier_point: list) -> PoseStamped:
        """
        Converts map coordinates [row, col] to a PoseStamped in world coordinates.
        """
        [row, col] = frontier_point
        
        # Calculate world coordinates
        # +0.5 * res ensures we target the center of the cell
        x = (col * self.map_resolution) + self.map_origin.x + (self.map_resolution / 2)
        y = (row * self.map_resolution) + self.map_origin.y + (self.map_resolution / 2)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        
        # TODO: Ideally, we should set the orientation to point towards the 'unknown' area
        # Simplified version: use default orientation (0 degrees)
        goal_pose.pose.orientation.w = 1.0

        return goal_pose

    def send_navigation_goal(self, goal_pose: PoseStamped):
        """
        Sends a navigation goal to Nav2.
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server is not available.")
            return

        self.is_navigating = True
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info("Sending goal to Nav2...")
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """
        Handles the initial response (accepted/rejected) from Nav2 for the goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the goal!")
            self.is_navigating = False
            return

        # self.get_logger().info("Nav2 accepted the goal.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        # feedback = feedback_msg.feedback
        # self.get_logger().info(f"Navigating... Distance remaining: {feedback.distance_remaining:.2f} m")
        pass

    def nav_result_callback(self, future):
        """
        Handles the final result (success/failure) of the navigation.
        """
        result = future.result().result
        status = future.result().status

        if status == 4: # 4 = SUCCEEDED
            self.get_logger().info("Successfully reached the goal!")
        elif status == 5: # 5 = CANCELED
            self.get_logger().warn("Navigation was canceled.")
        elif status == 6: # 6 = ABORTED
            self.get_logger().error("Navigation failed (Aborted).")
        else:
            self.get_logger().error(f"Navigation returned unknown status: {status}")

        # Regardless of success, this navigation attempt is complete
        self.is_navigating = False

    def publish_frontier_markers(self, frontier_points: list):
        """
        Visualizes the frontier points as small red cubes in RViz.
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontiers'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        marker.scale.x = self.map_resolution
        marker.scale.y = self.map_resolution
        marker.scale.z = self.map_resolution
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        for (row, col) in frontier_points:
            p = Point()
            p.x = (col * self.map_resolution) + self.map_origin.x + (self.map_resolution / 2)
            p.y = (row * self.map_resolution) + self.map_origin.y + (self.map_resolution / 2)
            p.z = 0.0
            marker.points.append(p)

        self.frontier_marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
