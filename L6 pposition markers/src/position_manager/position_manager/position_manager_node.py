"""
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from position_manager_msgs.srv import SavePosition, GetPositions, DeletePosition, NavigateMulti
from position_manager_msgs.action import NavigatePositions
from position_manager.position_storage import PositionStorage

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time

"""
# ============================================================================
# Position Manager Node
# ============================================================================
"""

class PositionManagerNode(Node):
    def __init__(self):
        super().__init__('position_manager')

        # Frames
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.action_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.general_callback_group = ReentrantCallbackGroup()

        self.storage = PositionStorage(self.get_logger())

        self.current_pose = None
        self._pose_received = False

        # Track currently navigating positions for visualization
        self.current_navigation_targets = set()
        
        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_ekf',
            self.odom_callback,
            10,
            callback_group=self.general_callback_group
        )
        # --- Publishers ---
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/position_markers',
            10
        )

        # Publisher for Web Feedback
        self.web_feedback_pub = self.create_publisher(
            String, 
            '/navigation_web_status', 
            10
        )
        self.marker_timer = self.create_timer(1.0, self.publish_markers, callback_group=self.general_callback_group)
        
         # --- Service Servers ---

        self.save_srv = self.create_service(
            SavePosition, '/save_position', self.save_position_callback, callback_group=self.service_callback_group)

        self.get_srv = self.create_service(
            GetPositions, '/get_positions', self.get_positions_callback, callback_group=self.service_callback_group)

        self.delete_srv = self.create_service(
            DeletePosition, '/delete_position', self.delete_position_callback, callback_group=self.service_callback_group)

        # Wrapper Service to Trigger Multi-Position Navigation from Web
        self.web_multi_nav_srv = self.create_service(
            NavigateMulti,
            '/start_multi_navigation_web',
            self.start_multi_navigation_web_callback,
            callback_group=self.service_callback_group
        )


        # --- Action Servers ---
        self.navigate_action_server = ActionServer(
            self,
            NavigatePositions,
            '/navigate_positions',
            execute_callback=self.navigate_positions_callback,
            goal_callback=self.navigate_goal_callback,
            cancel_callback=self.navigate_cancel_callback,
            callback_group=self.action_callback_group
        )

        # --- Action Client (to Nav2) ---
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.action_callback_group
        )

        self.get_logger().info('Position Manager Node initialized')

    # ========================================================================
    # Callbacks - Subscription
    # ========================================================================

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if not self._pose_received:
            self._pose_received = True

    # ========================================================================
    # Service Callbacks
    # ========================================================================

    def save_position_callback(self, request, response):
        """
        Save position in MAP frame using TF map->base_link.
        Falls back to odom pose ONLY if TF is unavailable.
        """
        try:
            pose = self._get_robot_pose_in_map()

            if pose is not None:
                x, y, theta = pose
            else:
                if self.current_pose is None:
                    response.success = False
                    response.message = "No TF and no odometry data"
                    return response

                x = self.current_pose.position.x
                y = self.current_pose.position.y
                orientation_q = self.current_pose.orientation
                (_, _, theta) = euler_from_quaternion([
                    orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
                ])
                self.get_logger().warn("Saving pose from /odom_ekf as fallback")

            success, message = self.storage.add_position(request.name, x, y, theta)
            response.success = success
            response.message = message
            self.get_logger().info(f"Save position: {message}, x={x:.3f}, y={y:.3f}, yaw={theta:.3f}")
            return response

        except Exception as e:
            response.success = False
            response.message = f"Save failed: {str(e)}"
            return response


    def get_positions_callback(self, request, response):
        positions = self.storage.load_positions()
        response.names = [p['name'] for p in positions]
        response.x_coords = [p['x'] for p in positions]
        response.y_coords = [p['y'] for p in positions]
        response.theta_coords = [p['theta'] for p in positions]
        return response

    def delete_position_callback(self, request, response):
        success, message = self.storage.delete_position(request.name)
        response.success = success
        response.message = message
        return response

    # ========================================================================
    # Action Callbacks - Navigation
    # ========================================================================

    def _publish_web_status(self, message):
        msg = String()
        msg.data = message
        self.web_feedback_pub.publish(msg)

    # --- Multi-Position Web Navigation Service Callback ---
    def start_multi_navigation_web_callback(self, request, response):
        position_names = request.names
        self.get_logger().info(f"Web requested multi-navigation to: {position_names}")

        # Validate all positions exist
        missing = []
        for name in position_names:
            if self.storage.get_position(name) is None:
                missing.append(name)

        if missing:
            response.success = False
            response.message = f"Positions not found: {missing}"
            return response

        if not position_names:
            response.success = False
            response.message = "No positions provided"
            return response

        # Start navigation in background task
        self.executor.create_task(self._execute_web_multi_navigation(list(position_names)))

        response.success = True
        response.message = f"Multi-navigation started for {len(position_names)} positions"
        return response

    # --- Multi-Position Background Task ---
    async def _execute_web_multi_navigation(self, position_names):
        total = len(position_names)
        reached = 0

        self._publish_web_status(f"Starting multi-navigation: {total} positions")

        # Check Nav2 availability once
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self._publish_web_status("Error: Nav2 server not available")
            return

        for idx, name in enumerate(position_names):
            # Mark current position as navigation target
            self.current_navigation_targets.add(name)

            try:
                pos = self.storage.get_position(name)
                if not pos:
                    self._publish_web_status(f"Skipping {name}: not found")
                    continue

                self._publish_web_status(f"Moving to {name} ({idx + 1}/{total})...")

                # Prepare Nav2 goal
                nav_goal = NavigateToPose.Goal()
                nav_goal.pose.header.frame_id = 'map'
                nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
                nav_goal.pose.pose.position.x = pos['x']
                nav_goal.pose.pose.position.y = pos['y']
                q = quaternion_from_euler(0, 0, pos['theta'])
                nav_goal.pose.pose.orientation.x = q[0]
                nav_goal.pose.pose.orientation.y = q[1]
                nav_goal.pose.pose.orientation.z = q[2]
                nav_goal.pose.pose.orientation.w = q[3]

                # Send goal
                send_goal_future = self.nav2_client.send_goal_async(nav_goal)
                goal_handle = await send_goal_future

                if not goal_handle.accepted:
                    self._publish_web_status(f"Nav2 rejected goal for {name}, skipping")
                    continue

                # Wait for result
                result_future = goal_handle.get_result_async()
                result = await result_future

                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    reached += 1
                    self._publish_web_status(f"Arrived at {name} ({reached}/{total} reached)")
                    time.sleep(0.5)
                else:
                    self._publish_web_status(f"Failed to reach {name}")

            finally:
                # Remove position from navigation targets
                self.current_navigation_targets.discard(name)

        self._publish_web_status(f"Multi-navigation complete: {reached}/{total} positions reached")

    # ------------------------------------------------------------------
    #  Multiple Position Navigation (Action Server)
    # ------------------------------------------------------------------ 
    async def navigate_positions_callback(self, goal_handle):
        position_names = goal_handle.request.position_names
        positions_reached = 0

        self.get_logger().info(f'Starting multi-position navigation: {position_names}')

        for name in position_names:
            # Mark current position as navigation target
            self.current_navigation_targets.add(name)

            try:
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Multi-navigation cancelled")
                    goal_handle.canceled()
                    result = NavigatePositions.Result()
                    result.success = False
                    result.message = "Cancelled"
                    result.positions_reached = positions_reached
                    return result

                # Get position
                pos = self.storage.get_position(name)
                if not pos:
                    self.get_logger().warn(f"Position '{name}' not found, skipping")
                    continue

                # Prepare Nav2 goal
                nav_goal = NavigateToPose.Goal()
                nav_goal.pose.header.frame_id = 'map'
                nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
                nav_goal.pose.pose.position.x = pos['x']
                nav_goal.pose.pose.position.y = pos['y']

                q = quaternion_from_euler(0, 0, pos['theta'])
                nav_goal.pose.pose.orientation.x = q[0]
                nav_goal.pose.pose.orientation.y = q[1]
                nav_goal.pose.pose.orientation.z = q[2]
                nav_goal.pose.pose.orientation.w = q[3]

                # Send to Nav2
                send_future = self.nav2_client.send_goal_async(nav_goal)
                nav_goal_handle = await send_future

                if not nav_goal_handle.accepted:
                    self.get_logger().warn(f"Nav2 rejected goal for '{name}', skipping")
                    continue

                # Wait for result
                result_future = nav_goal_handle.get_result_async()
                nav_result = await result_future

                if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
                    positions_reached += 1
                    self.get_logger().info(f"Reached position: {name}")
                    time.sleep(0.5)
                else:
                    self.get_logger().warn(f"Failed to reach: {name}")

                # Publish feedback
                feedback = NavigatePositions.Feedback()
                feedback.current_position_name = name
                goal_handle.publish_feedback(feedback)

            finally:
                # Remove position from navigation targets
                self.current_navigation_targets.discard(name)

        # Complete
        goal_handle.succeed()
        result = NavigatePositions.Result()
        result.success = True
        result.positions_reached = positions_reached
        result.message = f"Reached {positions_reached}/{len(position_names)} positions"
        return result

    # ========================================================================
    # Visualization & Callbacks
    # ========================================================================

    def publish_markers(self):
        try:
            positions = self.storage.load_positions()
            marker_array = MarkerArray()
            for idx, pos in enumerate(positions):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'saved_positions'
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = pos['x']
                marker.pose.position.y = pos['y']
                marker.pose.position.z = 0.1
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15

                # Red color for navigation targets, green for others
                if pos['name'] in self.current_navigation_targets:
                    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
                else:
                    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

                marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)
        except Exception as e:
            self.get_logger().debug(f"Marker publish error: {e}")

    # ========================================================================
    # Action Server Lifecycle Callbacks
    # ========================================================================

    def navigate_goal_callback(self, goal_request):
        self.get_logger().info("Goal request received")
        return GoalResponse.ACCEPT

    def navigate_cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    # ========================================================================
    # Helper Methods
    # ========================================================================

    def _get_robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),  # latest
                timeout=Duration(seconds=1.0),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except TransformException as ex:
            self.get_logger().warn(
                f"TF lookup failed ({self.map_frame} -> {self.base_frame}): {ex}"
            )
            return None


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PositionManagerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()