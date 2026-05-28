"""Test: publish a simple ROS 2 topic from Isaac Sim's Python environment."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

rclpy.init()
node = Node('isaac_bridge_test')
pub = node.create_publisher(String, '/cargo_bot/test', 10)

msg = String()
msg.data = 'hello from isaac sim'

print(f'Publishing on /cargo_bot/test ...')
for i in range(30):
    msg.data = f'hello from isaac sim #{i}'
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(1)
    print(f'  published #{i}')

node.destroy_node()
rclpy.shutdown()
print('Done.')
