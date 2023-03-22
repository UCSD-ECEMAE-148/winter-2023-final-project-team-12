import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import ArInfo
from std_msgs.msg import Int32MultiArray
import numpy as np
import time

# Nodes in this program
NODE_NAME = "controller_node"

# Topics subscribed to in this program
AR_INFO_TOPIC_NAME = "/ar/info"
LANE_FOLLOW_ACTUATOR_NAME = '/lane/cmd_vel'

# Topics published
ACTUATOR_TOPIC_NAME = '/cmd_vel'
HSV_TOPIC_NAME = '/lane/hsv'

# setup states
STATE_IDENTIFY = "identify"
STATE_DROPOFF = "dropoff"
STATE_RETURN = "return"
STATE_STALL = "stall"

# map id to hsv values
hsv_map = {
#   id: [h_l,s_l,v_l,h_h,s_h,v_h]
    1: [100, 60, 75, 115, 255, 255],    # blue
    2: [25, 50, 75, 35, 255, 255]     # yellow
}

# map id to return track instructions
return_instructions = {
    #id: [(throttle, steer, sleep time), ...]
    # reverse to release 
    1: [(-0.5, 0, 0.5), (0.5, 0.7, 9.0)],
    2: [(-0.5, 0, 0.5), (0.5, 0.55, 11.5)]
}

class Controller(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        # create subscriber for ar tag information
        self.ar_info_subscriber = self.create_subscription(ArInfo, AR_INFO_TOPIC_NAME, self.handle_ar, 10)
        self.ar_info_subscriber

        # create subscriber for directions from line follow algorithm
        self.line_follow_subscriber = self.create_subscription(Twist, LANE_FOLLOW_ACTUATOR_NAME, self.handle_line_follow, 10)
        self.line_follow_subscriber

        # create publisher for vesc control
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_publisher
        self.twist_cmd = Twist()

        # create publisher for color filter
        self.hsv_publisher = self.create_publisher(Int32MultiArray, HSV_TOPIC_NAME, 10)
        self.hsv_publisher
        self.hsv_value = Int32MultiArray()

        self.declare_parameters(
            namespace='',
            parameters=[
            ('stop_threshold', 150),
            ('zero_steer', 0.0),
            ('zero_throttle', 0.0)
            ]
        )

        self.state = STATE_IDENTIFY
        self.stop = False
        self.package_id = -1
    
    def stop_detected(self, corners):
        size = np.linalg.norm(corners[0] - corners[2])
        threshold = self.get_parameter('stop_threshold').value
        self.get_logger().debug(f'Stop size: {size}, Threshold: {threshold}')
        return size >= threshold

    def handle_ar(self, data):
        ids = data.ids
        # un-flatten list. 4 corners, each with [x,y]. Unknown number of tags
        corners = np.reshape(data.corners, (-1, 4, 2))

        if self.state == STATE_IDENTIFY:
            # check if any ar tag corners fit within bounds of pusher
            # set self.package_id to correlating id.
            self.package_id = -1
            for i in range(len(ids)):
                # if id found and it isn't stop_id
                if ids[i] != 0:
                    self.package_id = ids[i]
                    break
        elif self.state == STATE_DROPOFF:
            for i in range(len(corners)):
                if ids[i] == 0 and self.stop_detected(corners[i]):
                    self.stop = True
                    break
        self.control_logic()

    def handle_line_follow(self, data):
        self.twist_cmd = data 

    def control_logic(self):
        self.get_logger().debug(f'State: {self.state}, Package: {self.package_id}')
        if self.state == STATE_IDENTIFY:
            if self.package_id in hsv_map.keys():
                self.hsv_value.data = hsv_map[self.package_id]
                self.hsv_publisher.publish(self.hsv_value)
                self.get_logger().info('Moving to DROPOFF mode')
                self.state = STATE_DROPOFF
        elif self.state == STATE_DROPOFF:
            if self.stop:
                self.stop = False
                self.twist_cmd.angular.z = self.get_parameter('zero_steer').value
                self.twist_cmd.linear.x =self.get_parameter('zero_throttle').value
                self.state = STATE_RETURN
                self.get_logger().info('Moving to RETURN mode')
        elif self.state == STATE_RETURN:
            # self.state = STATE_STALL
            sleep_count = 50
            instructions = return_instructions.get(self.package_id, [])
            for throttle, steer, sleep_time in instructions:
                self.get_logger().info(f'Throttle: {throttle} ({type(throttle)}), Steer: {steer}({type(steer)}), Sleeper: {sleep_time}')
                self.twist_cmd.linear.x = float(throttle)
                self.twist_cmd.angular.z = float(steer)
                adjusted_sleep_time = sleep_time/sleep_count
                sleep_start = time.time()
                # vesc shutsoff after a second or so. This a
                for _ in range(sleep_count):
                    self.twist_publisher.publish(self.twist_cmd)
                    time.sleep(adjusted_sleep_time)
                sleep_end = time.time()
                self.get_logger().info(f'Actual sleep time: {sleep_end-sleep_start}')

            self.get_logger().info('Moving to IDENTIFY mode')
            self.twist_cmd.angular.z = self.get_parameter('zero_steer').value
            self.twist_cmd.linear.x =self.get_parameter('zero_throttle').value
            self.state = STATE_IDENTIFY

        self.twist_publisher.publish(self.twist_cmd)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        rclpy.spin(controller)
        controller.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        controller.get_logger().info(f"Shutting down {NODE_NAME}...")

        controller.destroy_node()
        rclpy.shutdown()
        controller.get_logger().info(f"{NODE_NAME} shut down successfully.")


if __name__ == "__main__":
    main()
