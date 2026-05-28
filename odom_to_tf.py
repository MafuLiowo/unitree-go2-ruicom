#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_bridge')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, 
            '/utlidar/robot_odom', 
            self.odom_callback, 
            10
        )
        self.get_logger().info('TF Bridge started - converting /utlidar/robot_odom to TF')
    
    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug('Sent TF: %s -> %s' % (msg.header.frame_id, msg.child_frame_id))

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()