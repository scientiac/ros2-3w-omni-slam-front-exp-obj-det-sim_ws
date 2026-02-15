import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

# Messages
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

class CornerExplorationNode(Node):
    def __init__(self):
        super().__init__('object_detection_node') # Name matches launch file

        # === State Variables ===
        self.exploration_finished = False
        self.map_data = None
        self.map_info = None
        self.corners_to_visit = [] # Queue of poses
        self.current_nav_goal = None
        self.dropoff_locations = {} 
        
        # Vision State
        self.latest_shape = "unknown"
        self.latest_color = "unknown" 
        self.bridge = CvBridge()

        # === QoS for Map ===
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # === Subscribers ===
        self.create_subscription(Bool, '/exploration/complete', self.exploration_complete_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Image, '/camera', self.image_callback, 10)

        # === Publishers ===
        self.marker_pub = self.create_publisher(MarkerArray, '/dropoff_zones', 10)

        # === Action Client ===
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # === Main Logic Timer ===
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.processing_stage = "IDLE" 
        self.get_logger().info('Corner Vision System Waiting for Exploration Complete...')

    # --------------------------------------------------
    # 1. Callbacks (System & Map)
    # --------------------------------------------------
    def exploration_complete_callback(self, msg):
        if msg.data and not self.exploration_finished:
            self.get_logger().info("Exploration Complete Signal Received! Starting Corner Logic.")
            self.exploration_finished = True
            self.processing_stage = "CALCULATING"

    def map_callback(self, msg):
        self.map_data = msg
        self.map_info = msg.info

    # --------------------------------------------------
    # 2. Control Loop
    # --------------------------------------------------
    def control_loop(self):
        # STOP CONDITION: Check if we have found all 3 unique platforms
        if self.processing_stage == "DONE":
            return

        if len(self.dropoff_locations) >= 3:
            self.get_logger().info("SUCCESS: All 3 Platforms (Red, Blue, Green) Found!")
            self.processing_stage = "DONE"
            return

        if self.processing_stage == "IDLE":
            pass 

        elif self.processing_stage == "CALCULATING":
            if self.map_data:
                self.calculate_corner_goals()
                if self.corners_to_visit:
                    self.processing_stage = "NAVIGATING"
                    self.send_next_goal()
                else:
                    self.get_logger().error("Could not determine corners from map.")
                    self.processing_stage = "DONE"
            else:
                self.get_logger().warn("Waiting for map data...")

        elif self.processing_stage == "SCANNING":
            # This function now handles the logic of recycling the corner if scanning failed
            self.analyze_wall_markers()
            
            # If we still haven't found all 3, continue navigating
            if len(self.dropoff_locations) < 3:
                if self.corners_to_visit:
                    self.processing_stage = "NAVIGATING"
                    self.send_next_goal()
                else:
                    # This implies we visited everywhere, found <3 items, 
                    # but analyze_wall_markers didn't recycle the corners.
                    # (This shouldn't happen with the new logic below)
                    self.get_logger().warn("Queue empty but missing targets. Waiting...")
            else:
                self.processing_stage = "DONE"

    # --------------------------------------------------
    # 3. Map Analysis
    # --------------------------------------------------
    def calculate_corner_goals(self):
        width = self.map_info.width
        height = self.map_info.height
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        data = np.array(self.map_data.data).reshape((height, width))
        occupied_indices = np.argwhere(data > 50) 

        if occupied_indices.size == 0:
            return

        min_row, min_col = occupied_indices.min(axis=0)
        max_row, max_col = occupied_indices.max(axis=0)

        def grid_to_world(r, c):
            wx = (c * res) + origin_x
            wy = (r * res) + origin_y
            return wx, wy

        OFFSET = 0.8 

        # Define 4 corners
        corners = [
            (min_row, min_col, +OFFSET, +OFFSET), # Bottom-Left
            (min_row, max_col, +OFFSET, -OFFSET), # Bottom-Right
            (max_row, max_col, -OFFSET, -OFFSET), # Top-Right
            (max_row, min_col, -OFFSET, +OFFSET)  # Top-Left
        ]

        self.corners_to_visit = []

        for (r, c, off_y, off_x) in corners:
            wall_x, wall_y = grid_to_world(r, c)
            target_x = wall_x + off_x
            target_y = wall_y + off_y

            # Calculate Orientation: Face the wall
            yaw = math.atan2(wall_y - target_y, wall_x - target_x)
            q = self.euler_to_quaternion(yaw)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = target_x
            goal.pose.position.y = target_y
            goal.pose.orientation = q
            
            self.corners_to_visit.append({'pose': goal, 'wall_pos': (wall_x, wall_y)})

        self.get_logger().info(f"Calculated {len(self.corners_to_visit)} corner inspection points.")

    def euler_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.z = math.sin(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        return q

    # --------------------------------------------------
    # 4. Navigation Handling
    # --------------------------------------------------
    def send_next_goal(self):
        if not self.corners_to_visit:
            return

        self.current_nav_goal = self.corners_to_visit.pop(0)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_nav_goal['pose']

        target_x = goal_msg.pose.pose.position.x
        target_y = goal_msg.pose.pose.position.y
        
        self.get_logger().info(f"Navigating to corner: {target_x:.1f}, {target_y:.1f}")
        
        self.nav_to_pose_client.wait_for_server()
        self.future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected, skipping corner.")
            self.processing_stage = "SCANNING" 
            return
        
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info("Arrived at corner. Scanning...")
            time.sleep(1.0) 
            self.processing_stage = "SCANNING"
        else:
            self.get_logger().warn(f"Navigation failed with status {status}. Skipping.")
            self.processing_stage = "SCANNING"

    # --------------------------------------------------
    # 5. Vision Logic (UPDATED WITH ROBUST DETECTION)
    # --------------------------------------------------
    def get_shape(self, contour):
        """ Robust shape detection from provided implementation """
        peri = cv2.arcLength(contour, True)
        if peri == 0:
            return "unknown"

        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        vertices = len(approx)

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h if h != 0 else 0

        if vertices == 3:
            return "triangle"

        if vertices == 4:
            if 0.8 <= aspect_ratio <= 1.2:
                return "square"
            else:
                return "rectangle"

        return "unknown"

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # Reset detections for this frame
            current_detected_shape = "unknown"
            current_detected_color = "unknown"

            # ---------------- COLOR MASKS ----------------
            mask_r1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
            mask_r2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
            mask_r = cv2.add(mask_r1, mask_r2)

            mask_b = cv2.inRange(hsv, np.array([94, 80, 2]), np.array([126, 255, 255]))
            mask_g = cv2.inRange(hsv, np.array([35, 52, 72]), np.array([85, 255, 255]))

            kernel = np.ones((5, 5), np.uint8)

            # RED OBJECTS (Hough Circles FIRST)
            refined_red = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, kernel, iterations=2)
            gray_red = cv2.GaussianBlur(refined_red, (9, 9), 1.5)

            circles = cv2.HoughCircles(
                gray_red, cv2.HOUGH_GRADIENT, dp=1.2, minDist=40,
                param1=100, param2=25, minRadius=20, maxRadius=200
            )

            circle_mask = np.zeros_like(refined_red)

            if circles is not None:
                current_detected_color = "red"
                current_detected_shape = "circle"
                
                circles = np.uint16(np.around(circles))
                for (x, y, r) in circles[0]:
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                    cv2.putText(frame, "circle", (x - 20, y - r - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    # Remove circle from mask so it won't become a cube
                    cv2.circle(circle_mask, (x, y), r + 5, 255, -1)

            # Remaining RED blobs -> red-cube
            red_no_circles = cv2.bitwise_and(refined_red, cv2.bitwise_not(circle_mask))
            contours_red, _ = cv2.findContours(red_no_circles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours_red:
                if cv2.contourArea(cnt) > 1200:
                    current_detected_color = "red"
                    current_detected_shape = "cube" # or square
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, "red-cube", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # BLUE OBJECTS
            refined_blue = cv2.morphologyEx(mask_b, cv2.MORPH_OPEN, kernel, iterations=2)
            contours_blue, _ = cv2.findContours(refined_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours_blue:
                if cv2.contourArea(cnt) > 1200:
                    shape = self.get_shape(cnt)
                    current_detected_color = "blue"
                    current_detected_shape = shape 
                    
                    label = "triangle" if shape == "triangle" else "blue-cube"
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # GREEN OBJECTS
            refined_green = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, kernel, iterations=2)
            contours_green, _ = cv2.findContours(refined_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours_green:
                if cv2.contourArea(cnt) > 1200:
                    shape = self.get_shape(cnt)
                    current_detected_color = "green"
                    
                    if shape == "rectangle":
                        current_detected_shape = "rectangle"
                    else:
                        current_detected_shape = "cube"

                    label = "rectangle" if current_detected_shape == "rectangle" else "green-cube"
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Update State Variables if something was found
            if current_detected_color != "unknown":
                self.latest_color = current_detected_color
                self.latest_shape = current_detected_shape
            
            # Show the advanced visualization
            cv2.imshow("Gazebo Shape Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'CV Error: {e}')

    def analyze_wall_markers(self):
        """ Checks object. If found, save it. If NOT found, re-queue the corner. """
        wall_x, wall_y = self.current_nav_goal['wall_pos']
        found_type = None
        
        # Check current state variables
        # Note: Blue Rectangle is explicitly NOT a valid type here, so it falls to 'else'
        if self.latest_color == 'blue' and self.latest_shape == 'triangle':
            found_type = "BLUE_PLATFORM"
            color_rgb = (0.0, 0.0, 1.0)
        elif self.latest_color == 'red' and self.latest_shape == 'circle':
            found_type = "RED_PLATFORM"
            color_rgb = (1.0, 0.0, 0.0)
        elif self.latest_color == 'green' and self.latest_shape == 'rectangle':
            found_type = "GREEN_PLATFORM"
            color_rgb = (0.0, 1.0, 0.0)
        
        if found_type:
            # === SUCCESS CASE ===
            if found_type not in self.dropoff_locations:
                self.get_logger().info(f"FOUND NEW: {found_type} at ({wall_x:.2f}, {wall_y:.2f})")
                
                # Save the location
                self.dropoff_locations[found_type] = self.current_nav_goal['pose']
                self.publish_platform_marker(wall_x, wall_y, found_type, color_rgb)
            else:
                self.get_logger().info(f"Re-verified {found_type}. Already in database.")
            
            # We found a valid platform here, so we are done with this specific corner.
            # We do NOT add it back to the queue.

        else:
            # === FAILURE / RETRY CASE ===
            # We saw nothing, or we saw an invalid shape (e.g., Blue Rectangle)
            self.get_logger().info(f"Corner invalid or uncertain (Saw {self.latest_color} {self.latest_shape}). Re-queueing this corner.")
            
            # CRITICAL: Add this specific corner back to the END of the queue.
            # The robot will go to other corners, then come back here later.
            self.corners_to_visit.append(self.current_nav_goal)

    # --------------------------------------------------
    # 6. Visualization Markers
    # --------------------------------------------------
    def publish_platform_marker(self, x, y, label, color_rgb):
        marker_array = MarkerArray()

        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.id = abs(hash(label)) % 1000
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = 1.0
        text_marker.scale.z = 0.3
        text_marker.color.r, text_marker.color.g, text_marker.color.b = color_rgb
        text_marker.color.a = 1.0
        text_marker.text = label
        marker_array.markers.append(text_marker)

        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.type = Marker.SPHERE
        sphere_marker.id = abs(hash(label)) % 1000 + 1
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = x
        sphere_marker.pose.position.y = y
        sphere_marker.pose.position.z = 0.1
        sphere_marker.scale.x = 0.4; sphere_marker.scale.y = 0.4; sphere_marker.scale.z = 0.4
        sphere_marker.color.r, sphere_marker.color.g, sphere_marker.color.b = color_rgb
        sphere_marker.color.a = 0.8
        marker_array.markers.append(sphere_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = CornerExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
