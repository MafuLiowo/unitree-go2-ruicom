#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.client = self.create_client(LoadMap, '/map_server/load_map')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wait map_server service...')
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_map = None
        
    def timer_callback(self):
        if self.current_map is not None:
            self.current_map.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.current_map)
    
    def load_and_publish(self):
        request = LoadMap.Request()
        request.map_url = '/home/unitree/ai-unitree-go2-ruicom/map/map4.yaml'
        
        future = self.client.call_async(request)
        future.add_done_callback(self.map_loaded_callback)
    
    def map_loaded_callback(self, future):
        try:
            response = future.result()
            if response.result == 0:
                self.current_map = response.map
                self.get_logger().info('successs')
            else:
                self.get_logger().error('fail')
        except Exception as e:
            self.get_logger().error(f'services fail: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    node.load_and_publish()
    rclpy.spin(node)

if __name__ == '__main__':
    main()