#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from geometry_msgs.msg import Pose2D
import cv2 as cv

from cv_bridge import CvBridge, CvBridgeError

# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        self.get_logger().info('Kyaaa Haalllll hai....')
        # Subscribe the topic /camera/image_raw

        self.subscriber=self.create_subscription(Image, '/camera/image_raw',self.image_callback,10)
        self.hb_x=0.0
        self.hb_y=0.0
        self.hb_theta=0.0
        # self.publisher_ = self.create_publisher(Twist, '{}/cmd_vel'.format(name), 1)
        # self.subscriber=self.create_subscription(Pose, '{}/pose'.format(name), self.pose_callback, 1)


    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e, throttle_duration_sec=1)

        image = cv_image

        arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
        arucoParams = cv.aruco.DetectorParameters()
        detectors = cv.aruco.ArucoDetector(arucoDict,arucoParams)
        corners, ids, rejected = detectors.detectMarkers(cv_image)
        # cv.imshow("The image", cv_image)
        # print(corners)
        # aruco.drawDetectedMarkers(cv_image,corners)
        # print("This is something: ", markerCorners , markerIds , rejectedCandidates)
        # marker_points = np.array([
        #     [-marker_size/2, -marker_size/2, 0],
        #     [-marker_size/2, marker_size/2, 0],
        #     [marker_size/2, marker_size/2, 0],
        #     [marker_size/2, -marker_size/2, 0]
        # ], dtype=np.float32)
        # print(ids)


        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                cv.putText(cv_image,str(Id),(10,50),0,.7,(255,0,0),thickness=2)
                # print(marker_position)
                cv.imshow("The Image Dude", image)
                cv.waitKey(1)
               

        # cv.
        #convert ROS image to opencv image
        #Detect Aruco marker
        # Publish the bot coordinates to the topic  /detected_aruco

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
