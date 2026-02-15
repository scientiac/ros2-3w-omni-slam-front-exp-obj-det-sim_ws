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
from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time

class TwistToTwistStamped(Node):
    """
    Subscribes to geometry_msgs/msg/Twist and publishes as geometry_msgs/msg/TwistStamped.
    """

    def __init__(self):
        super().__init__('twist_converter_node')

        # Use get_parameter_or for compatibility with older Galactic/Humble versions
        param = self.get_parameter_or('use_sim_time', rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False))
        self.use_sim_time = param.get_parameter_value().bool_value
        
        # Subscribe to the clean output of Nav2
        self.twist_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        
        # Publish to the topic expected by the robot driver
        self.stamped_twist_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_stamped',
            10
        )
        self.get_logger().info(f"Twist Converter Node started. Using sim time: {self.use_sim_time}")


    def twist_callback(self, msg: Twist):
        """
        Receives a Twist message, packages it as TwistStamped, and publishes it.
        """
        stamped_msg = TwistStamped()
        
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link' 
        stamped_msg.twist = msg
        
        self.stamped_twist_publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    twist_converter = TwistToTwistStamped()
    try:
        rclpy.spin(twist_converter)
    except KeyboardInterrupt:
        pass
    finally:
        twist_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
