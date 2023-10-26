########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID: 2070
# Team Leader Name: Sanket Sharma
# Team Members Name: Ankit Chaudhary, Ankit Das, Bhajneet Singh Bedi, Sanket Sharma
# College: Chandigarh University
########################################################################################################################

#!/usr/bin/env python3


import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


angular=1.0
linear=1.0
angular_two =-1.0
linear_two=1.5
start=0
t=0
total_time=0
x=0
class MinimalPublisher(Node):

    def __init__(self, name):
        # super().__init__('py_topic_publisher_spiral')
        super().__init__('Task_1a')
        # self.get_logger().info(txt)
        # self.get_logger().info('Hello World4')
        self.publisher_ = self.create_publisher(Twist, '{}/cmd_vel'.format(name), 1)
        self.subscriber=self.create_subscription(Pose, '{}/pose'.format(name), self.pose_callback, 1)
        # self.publish_message()
        # self.get_logger().info('I will try to do something')
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        # while x==0:
        #     self.publish_message()
        #     time.sleep(0.5)
        

    def pose_callback(self, msg:Pose):
        self.get_logger().info('X:%f, Y: %f' % (msg.x, msg.y))
        
 
    def publish_message(self):
        global t, start
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        tmm = time.time()
        self.get_logger().info('Linear Velocity : %f, Angular Velocity : %f, Time: %f' % (message.linear.x, message.angular.z, tmm-start))
        if tmm-start >= t:
            # self.get_logger().info('demo text')
            global x
            if x==1:
                self.get_logger().info('Destination reached by bot 2')
                raise SystemExit

            if x==0:
                self.get_logger().info('Destination reached by bot 1')
                self.call_bot_two()
                start = time.time()
                x=1
            # self.future = self.cli.call_async(self.x)
            # self.timer_.destroy()
            # Used to shut down the node
            # raise SystemExit
        # time.sleep(1)
        self.publisher_.publish(message)
        # self.get_logger().info('Destination Reached for bot 1')
        # self.i += float(sys.argv[3])

    def call_bot_two(self):
        client=self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for service...')
        req = Spawn.Request()
        req.x=5.197917
        req.y=5.197917
        req.theta=0.0
        req.name='turtle2'

        future = client.call_async(req)

        self.publisher_ = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)
        self.subscriber=self.create_subscription(Pose, 'turtle/pose', self.pose_callback, 1)
        global t, tm, linear, angular
        tm = calculate_time(abs(angular_two))
        # t=tm+t
        t = tm
        linear = linear_two 
        angular = angular_two
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        # tmm = time.time()
        # if tmm-start >= total_time:
        #     raise SystemExit




# Don't know why I used 4, but had to... may be to complete a circle
def calculate_time(angular):
    # tmmm=angular*4/2*(math.pi)
    tmmm = 2*(math.pi)/angular
    print(tmmm)
    return tmmm 



def main(args=None):
    rclpy.init(args=args)
    global start, t, total_time
    t = calculate_time(angular)
    # tm = calculate_time(angular_two)
    # total_time=t+tm
    start=time.time()

    minimal_publisher = MinimalPublisher('turtle1')
    # minimal_publisher.publish_message()
    # while rclpy.ok():
    #     pass
    #     rclpy.spin_until_future_complete(minimal_publisher)
    #     if minimal_publisher.future.done():
    #         try:
    #             reponse=minimal_publisher.future.result()
    #         except Exception as e:
    #             minimal_publisher.get_logger().info(
    #                 'Service call failed %r' % (e,))
    #         break
    rclpy.spin(minimal_publisher)
    print('yeah')

    minimal_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()