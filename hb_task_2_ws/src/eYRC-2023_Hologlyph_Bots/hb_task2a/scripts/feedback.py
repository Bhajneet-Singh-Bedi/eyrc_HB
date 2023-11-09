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
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import math
from std_msgs.msg import Float64MultiArray, String
import yaml
# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        self.get_logger().info('Kyaaa Haalllll hai....')
        # Subscribe the topic /camera/image_raw
        self.publisher = self.create_publisher(String, '/detected_aruco', 10)
        self.subscriber=self.create_subscription(Image, '/camera/image_raw',self.image_callback,10)
        self.hb_x=0.0
        self.hb_y=0.0
        self.hb_theta=0.0
        self.dict_8={}
        self.coordinate_dict={}
        self.i=0
        self.x_const=0.0
        self.y_const=0.0
        self.del_x=0.0
        self.del_y=0.0
        self.camera_resolution=0.036
        # self.publisher_ = self.create_publisher(Twist, '{}/cmd_vel'.format(name), 1)
        # self.subscriber=self.create_subscription(Pose, '{}/pose'.format(name), self.pose_callback, 1)


    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e, throttle_duration_sec=1)

        image = cv_image
        
        Detected_ArUco_markers = {}

        arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        arucoParams = cv.aruco.DetectorParameters()
        detectors = cv.aruco.ArucoDetector(arucoDict,arucoParams)
        corners, ids, rejected = detectors.detectMarkers(cv_image)
        crn=corners

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

        if corners:
            for ids, corners in zip(ids, corners):
                if ids == 1:
                    Detected_ArUco_markers[str(ids)] = corners 

                corners = corners.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # topRight = (int(topRight[0]), int(topRight[1]))
                # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # corners = (topLeft, topRight, bottomRight, bottomLeft)

                self.coordinate_dict[int(ids)]=corners
                cv.putText(image,f"{ids[0]}",topLeft,cv.FONT_HERSHEY_PLAIN,1.3,(200, 100, 0),2,cv.LINE_AA,)
                # print(self.coordinate_dict)
                # print(ids, corners)
                #print('YEAHHHHHHHHHHHHHHH')
            self.coord(self.coordinate_dict, image)

            
        ArUco_marker_angles = {}
        # print(crn)
        ## enter your code here ##
        for ids in Detected_ArUco_markers:
            for corn in Detected_ArUco_markers[ids]:
                topListx = []
                topListy = []
                for i in range(4):

                    xcenter = int((corn[0][0]+corn[2][0])/2)							#determining x coordinate of center by midpoint theorem
                    ycenter = int((corn[0][1]+corn[2][1])/2)							#determining y coordinate of center by midpoint theorem

                    topListx.append(corn[i][0])					#making a list of corners x axis
                    topListy.append(corn[i][1])
                    #print topList
                    #print "works"

                xtop1 = max(topListx)								#top right
                ytop2 = min(topListy)								#top left

                for i in range(4):
                    if corn[i][0] == xtop1: 	
                        ytop1 = corn[i][1]
                        
                    if corn[i][1] == ytop2:
                        xtop2 = corn[i][0]

                lcenterx = int((xtop2+xtop1)/2)											#determining y coordinate of an edge to draw line
                lcentery = int((ytop2+ytop1)/2)											#determining x coordinate of an edge to draw line
                if ycenter - lcentery==0:
                    angle = 0
                else:
                    slope = -1*( xcenter - lcenterx )/float(( ycenter - lcentery ))
                    angle = 90 - int(57.3*math.atan(slope))
                ArUco_marker_angles[ids] = angle
                print(ArUco_marker_angles)
        """
        topLeft, topRight, bottomRight, bottomLeft

        {12: [array([458., 460.], dtype=float32), array([480., 459.], dtype=float32), 
        array([481., 481.], dtype=float32), array([459., 482.], dtype=float32)], 
        1: [array([262., 265.], dtype=float32), array([276., 265.], dtype=float32), array([275., 279.], 
        dtype=float32), array([262., 279.], dtype=float32)], 
        10: [array([457.,  20.], dtype=float32), array([480.,  19.], dtype=float32), 
        array([481.,  41.], dtype=float32), array([458.,  42.], dtype=float32)], 
        8: [array([18., 20.], dtype=float32), array([40., 19.], dtype=float32), 
        array([41., 42.], dtype=float32), array([19., 43.], dtype=float32)]}
        
        8, 10 
          1
        4, 12
        """
               

        # cv.
        #convert ROS image to opencv image
        #Detect Aruco marker
        # Publish the bot coordinates to the topic  /detected_aruco
    def coord(self, dict, image):

        # Centre of aruco markers.
        # print(type(dict[8]))
        center_8 = np.mean(dict[8], axis=0)
        center_10 = np.mean(dict[10], axis=0)
        center_12 = np.mean(dict[12], axis=0)
        center_1 = np.mean(dict[1], axis=0)

        center_8=(int(center_8[0]), int(center_8[1]))
        center_10=(int(center_10[0]), int(center_10[1]))
        center_12=(int(center_12[0]), int(center_12[1]))
        center_1=(int(center_1[0]), int(center_1[1]))

        cv.circle(image, center_8, 4, (255, 255, 255), 2)
        cv.circle(image, center_10, 4, (255, 255, 255), 2)
        cv.circle(image, center_12, 4, (255, 255, 255), 2)
        cv.circle(image, center_1, 4, (255, 255, 255), 2)
        # print(center_1, center_8, center_10, center_12)

        #Considering a constant starting distance between corner and bot.
        if self.i==0:
            self.x_const = center_1[0]-center_8[0]
            self.y_const = center_1[1]-center_8[1]
            self.i+=1
        else:
            pass
        # Mark for changes of distances.
        self.del_x = center_1[0]-center_8[0]
        self.del_y = center_1[1]-center_8[1]


        # Finalizing x,y coordinates.
        # For x
        if self.x_const > self.del_x:
            self.hb_x=self.del_x-self.x_const
        elif self.x_const < self.del_x:
            self.hb_x=self.del_x-self.x_const
        elif self.x_const==self.del_x:
            self.hb_x=0

        # For y
        if self.y_const > self.del_y:
            self.hb_y=self.y_const-self.del_y
        elif self.y_const < self.del_y:
            self.hb_y=self.y_const-self.del_y
        elif self.y_const==self.del_y:
            self.hb_y=0
        
        
        print("Coordinates are: ",self.hb_x*self.camera_resolution, self.hb_y*self.camera_resolution)
        data = {
            'pose': {
                'position': {
                    'x': self.hb_x*self.camera_resolution,
                    'y': self.hb_y*self.camera_resolution
                }
            }
        }

        # yaml_file_path = 'output.yaml'

        # Write the data to the YAML file
        # with open(yaml_file_path, 'w') as yaml_file:
            # yaml.dump(data, yaml_file, default_flow_style=False)
        
        message = String()

        yaml_data = yaml.dump(data)
        # with open(yaml_file_path, 'r') as yaml_file:
            # message.data = yaml_file.read()
        message.data = yaml_data
        self.publisher.publish(message)

        cv.imshow("The Image Dude", image)
        key = cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()