import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_msg.msg import ArInfo
from cv_bridge import CvBridge
import cv2 as cv
from cv2 import aruco
import numpy as np

# Nodes in this program
NODE_NAME = "ar_detection_node"

# Topics subscribed/published to in this program
CAMERA_TOPIC_NAME = "/camera/color/image_raw"
AR_INFO_TOPIC_NAME = "/ar/info"

AR_WINDOW = "ar"


class ArDetection(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        #TODO: define msg type to aggregate ids and corners
        # setup for ar info publisher
        self.ar_info_publisher = self.create_publisher(
            ArInfo, AR_INFO_TOPIC_NAME, 10
        )
        self.ar_info_publisher
        self.ar_info = ArInfo()
        # setup camera subscriber
        self.camera_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC_NAME, self.detect_ar_tags, 10
        )
        self.camera_subscriber
        self.bridge = CvBridge()

        # set parameter from command line. could also add a config file
        self.declare_parameters(namespace="", parameters=[("debug", False)])

    def detect_ar_tags(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters
        )
        # corners come as [x, y] aka [col, row]
        
        if ids is None:
            self.ar_info.ids = []
            self.ar_info.corners = []
        else:
            self.ar_info.ids = np.array(ids).flatten().tolist()
            self.ar_info.corners = np.array(corners).flatten().tolist()

        self.ar_info_publisher.publish(self.ar_info)

        if self.get_parameter("debug").value == True:
            frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            cv.imshow(AR_WINDOW, frame_markers)
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    ar_publisher = ArDetection()
    try:
        rclpy.spin(ar_publisher)
        ar_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        ar_publisher.get_logger().info(f"Shutting down {NODE_NAME}...")

        # Kill cv2 windows and node
        cv.destroyAllWindows()
        ar_publisher.destroy_node()
        rclpy.shutdown()
        ar_publisher.get_logger().info(f"{NODE_NAME} shut down successfully.")


if __name__ == "__main__":
    main()
