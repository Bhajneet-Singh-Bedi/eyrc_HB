#!/usr/bin/env python3


import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose

class MinimalPublisher(Node):

    def __init__(self):
        # super().__init__('py_topic_publisher_spiral')
        super().__init__('Task_1a')
        self.get_logger().info('Hello World')
        self.get_logger().info('Hello World4')
        # self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        # self.publisher=self.create_publisher(Pose, 'turtle1/pose', 1)
        # timer_period = 0.5  # seconds
        # self.timer_ = self.create_timer(timer_period, self.publish_message)


    # def publish_message(self):
    #     message = Twist()
    #     mssg=Pose()
    #     self.get_logger().info('Position : %f' % (mssg))
    #     message.linear.x = 2.0
    #     message.angular.z = 0.5
    #     self.get_logger().info('Linear Velocity : %f, Angular Velocity : %f, Position: ' % (message.linear.x, message.angular.z))
        # self.publisher_.publish(message)
        # self.i += float(sys.argv[3])

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# # from turtlesim.srv import SetPen
# from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose
# robot_x = 0
# distance = 0

# class MyNode(Node):


#     def __init__(self):
#         super().__init__('Task_1a')
#         self.get_logger().info('Hello World')
#         self.get_logger().info('Hello World4')
#     def subscribe():
#         node=rclpy.create_node('turtle_subs')
#         subcription=node.create_subscription(Pose, '/turtlesim/Pose', 10)
#         print(subcription)

#         try:
#             rclpy.spin(node)
#         except KeyboardInterrupt:
#             print('lala')
#             pass

#         node.destroy_node()
#         # rclpy.shutdown()
#     def callback(msg):
#         # This function will be called whenever a new message is received
#         print(f"Received turtlesim pose: x={msg.x}, y={msg.y}, theta={msg.theta}")


# # def call_set_pen_service(self, r, g, b, width, off):
# #         client = self.create_client(SetPen, '/turtle2/set_pen')
# #         while not client.wait_for_service(1.0):
# #             self.get_logger().warn("Waiting for service...")

# #         request = SetPen.Request()
# #         request.r = r
# #         request.g = g
# #         request.b = b
# #         request.width = width
# #         request.off = off

# #         future = client.call_async(request)


# def main(args=None):
#     rclpy.init(args=args)

#     node=MyNode()
#     MyNode.subscribe()
#     # call_set_pen_service(1,2,3, 2,1,1)
#     rclpy.shutdown()

# if __name__=="__main__":
#     main()