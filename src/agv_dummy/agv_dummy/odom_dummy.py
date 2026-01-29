#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_from_yaw(yaw: float):
    # Quaternion (x=0,y=0,z=sin(yaw/2),w=cos(yaw/2))
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class OdomDummy(Node):
    def __init__(self):
        super().__init__('odom_dummy')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('mode', 'circle')  # 'static' or 'circle'
        self.declare_parameter('radius', 0.5)     # meter
        self.declare_parameter('yaw_rate', 0.3)   # rad/s

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.mode = str(self.get_parameter('mode').value).lower()
        self.radius = float(self.get_parameter('radius').value)
        self.yaw_rate = float(self.get_parameter('yaw_rate').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.t0 = self.get_clock().now()
        self.last_t = self.t0

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"odom_dummy running: mode={self.mode}, odom_frame={self.odom_frame}, base_frame={self.base_frame}"
        )

    def on_timer(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9  # seconds since start
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now

        # Default: static at origin
        x = 0.0
        y = 0.0
        yaw = 0.0
        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.mode == 'circle':
            yaw = self.yaw_rate * t
            x = self.radius * math.cos(yaw)
            y = self.radius * math.sin(yaw)
            vx = -self.radius * self.yaw_rate * math.sin(yaw)
            vy =  self.radius * self.yaw_rate * math.cos(yaw)
            wz = self.yaw_rate

        qx, qy, qz, qw = quat_from_yaw(yaw)

        # TF: odom -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(wz)

        self.odom_pub.publish(odom)


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = OdomDummy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
