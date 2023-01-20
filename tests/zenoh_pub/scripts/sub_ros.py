#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class test(Node):

    def __init__(self):
        super().__init__('test_node')

        self.TwistSub = self.create_subscription(Twist, 'demo/data', self.TwistCallback, 1)

    def TwistCallback(self, msgTwist):
        print(msgTwist.linear.x)
        print(msgTwist.angular.z)
        return


if __name__ == '__main__':
    rclpy.init()
    ts = test()
    rclpy.spin(ts)
    ts.destroy_node()
    rclpy.shutdown()