########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1B ###########################################
# Team ID: 2070
# Team Leader Name: Sanket Sharma
# Team Members Name: Ankit Chaudhary, Ankit Das, Bhajneet Singh Bedi, Sanket Sharma
# College: Chandigarh University
########################################################################################################################

#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal


class HBTask1BController(Node):
    def __init__(self):
        super().__init__("hb_task1b_controller")

        # Initialze Publisher and Subscriber
        # initialising publisher and subscriber of cmd_vel and odom respectively
        self.velPub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odomSub = self.create_subscription(Odometry, "/odom", self.odomCb, 10)
        self.odomSub

        # Declare a Twist message
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        # Initialise the required variables to 0

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0
        self.hb_orient_quat = [0, 0, 0, 0]
        self.hb_orient_euler = [0, 0, 0]

        self.des_pos = [0, 0, 0]  # x y theta(deg)

        self.kP = [2, 2, 2]
        # Initialise variables that may be needed for the control loop

        self.error = [0, 0, 0]
        self.errorB = [0, 0, 0]

        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, "next_goal")
        self.req = NextGoal.Request()
        self.index = 0
        self.future = None
        self.flag = 0

    def send_request(self, index):
        self.req.request_goal = index
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

    def odomCb(self, msg):
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y
        self.hb_orient_quat[0] = msg.pose.pose.orientation.x
        self.hb_orient_quat[1] = msg.pose.pose.orientation.y
        self.hb_orient_quat[2] = msg.pose.pose.orientation.z
        self.hb_orient_quat[3] = msg.pose.pose.orientation.w
        self.hb_orient_euler = euler_from_quaternion(self.hb_orient_quat)
        self.hb_theta = self.hb_orient_euler[2]


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the EbotController class
    ebot_controller = HBTask1BController()

    # Send an initial request with the index from ebot_controller.index
    ebot_controller.send_request(ebot_controller.index)

    # Main loop
    while rclpy.ok():
        # Check if the service call is done
        if ebot_controller.future.done():
            try:
                # response from the service call
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().infselfo("Service call failed %r" % (e,))
            else:
                #########           GOAL POSE             #########
                ebot_controller.des_pos[0] = response.x_goal
                ebot_controller.des_pos[1] = response.y_goal
                ebot_controller.des_pos[2] = response.theta_goal
                ebot_controller.flag = response.end_of_list

                ####################################################

                ebot_controller.error[0] = -(
                    ebot_controller.hb_x - ebot_controller.des_pos[0]
                )

                ebot_controller.error[1] = -(
                    ebot_controller.hb_y - ebot_controller.des_pos[1]
                )

                ebot_controller.error[2] = -(
                    ebot_controller.hb_theta - ebot_controller.des_pos[2]
                )

                ebot_controller.errorB[0] = (
                    math.cos(-ebot_controller.hb_theta) * ebot_controller.error[0]
                ) - (math.sin(-ebot_controller.hb_theta) * ebot_controller.error[1])
                ebot_controller.errorB[1] = (
                    math.sin(-ebot_controller.hb_theta) * ebot_controller.error[0]
                ) + (math.cos(-ebot_controller.hb_theta) * ebot_controller.error[1])

                ebot_controller.vel.linear.x = (
                    ebot_controller.errorB[0] * ebot_controller.kP[0]
                )
                ebot_controller.vel.linear.y = (
                    ebot_controller.errorB[1] * ebot_controller.kP[1]
                )
                ebot_controller.vel.angular.z = (
                    ebot_controller.error[2] * ebot_controller.kP[2]
                )

                if (
                    abs(ebot_controller.error[0]) < 0.2
                    and abs(ebot_controller.error[1]) < 0.2
                    and abs(ebot_controller.error[2]) < 0.1
                ):
                  
                    ebot_controller.index += 1
                    if ebot_controller.flag == 1:
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.index)

                ebot_controller.velPub.publish(ebot_controller.vel)

        rclpy.spin_once(ebot_controller)

    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
