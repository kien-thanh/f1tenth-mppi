#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
import csv
import ast
import yaml
import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
    get_package_share_directory("path_finder"),
    "config",
    "params.yaml"
)
# TODO - FOR SIMULATION WORK, I WILL HAVE THIS FILE HERE
# TODO - IN ACTUAL USAGE, I SHOULD COMMUNICATE WITH THE OTHER NODE
# TODO - BY RECEIVING A PUBLISH OF THE COORDS 

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    
    finds current waypoint! (does it go to it tho?)
    """
    
    def __init__(self):
        super().__init__('pure_pursuit_node')
        with open(config, "r") as f:
            self.param = yaml.safe_load(f)
        self.pose_topic = self.param["pose_topic"]
        pose_topic = "/ego_racecar/odom"
        map_topic = "/map"
        scan_topic = "/scan"
        clicked_topic = "/clicked_point"
        waypoints = "/custom_waypoints"
        
        # self.pose_sub = self.create_subscription(PoseStamped, '/current_pose',
        #                                          self.pose_callback,
        #                                          10)
        
        # TODO - real code to use is above
        self.pose_sub = self.create_subscription(

            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.param["map_topic"],
            self.map_callback,
            10
        )

        self.waypoint_sub = self.create_subscription(
            String,
            self.param["pure_pursuit_topic"],
            self.waypoint_callback,
            10
        )
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.param["drive_topic"],
            10
        )
        
        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        # self.subscription_joy = self.create_subscription(
        #     Joy,
        #     "/joy",
        #     self.joy_callback,
        #     qos
        # )

        # TODO - probably don't need this ^^

        # TODO - need to change this part for the actual car
        self.declare_parameter('waypoint_file', '/sim_ws/waypoints_straight.csv')
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        

        # for real world testing, would need to add a little safety system
        # also could do a bit more integration between systems 
        # safety system can be done by properly putting this as "autonomous" code - look at documentation

        self.lookahead_distance = .50  # lookahead distance for pure pursuit
        self.current_pose = None
        self.last_position = None
        self.current_position = None
        self.steering_angle = None
        self.transformed_goal = None
        self.current_goal = None

        # active waypoint visualization
        self.marker_pub = self.create_publisher(Marker, 'waypoints_marker', 10)
        self.waypoints = None
        # self.waypoints  = self.load_waypoints(waypoint_file)
        # change the csv file if doing in real life :)
        self.logging_timer = self.create_timer(.5, self.log_info)
        # self.publish_waypoints()
        self.set_speed = 1.0

    def waypoint_callback(self, msg):
        """ should get it back as a group of tuples"""
        received_str = msg.data
        self.waypoints, self.set_speed = ast.literal_eval(received_str)
        if self.set_speed is None:
            self.set_speed=1.0
        # self.publish_waypoints()

    def map_callback(self,msg):
        """ checks the map data """
        print("got something?")
        occupancy_data = msg.data
        print(type(occupancy_data))
        print(f"watch this: {len(occupancy_data)//141} == 124")
        
        self.get_logger().info(f"Received map: {msg.info.width} x {msg.info.height}")
        self.get_logger().info(f"Received map: {msg.info.height}")
        self.get_logger().info(f"Received map: {msg.info.resolution}")
        self.get_logger().info(f"Received map: {msg.info.origin}")

        # print(f"{occupancy_data=}")
    def subscription_joy(self):
        """function likely used to stop in unsafe scenarios for the real world :)"""
        pass

    # def load_waypoints(self, filename):
    #     """ loads waypoints from the csv """
    #     waypoints = []
    #     with open(filename, 'r') as csvfile:
    #         csvreader = csv.reader(csvfile)
    #         next(csvreader)
    #         for row in csvreader:
    #             x, y, theta = map(float, row)
    #             waypoints.append((x,y))
    #     return waypoints

    def make_point(self, point_):
        point = Point()
        point.x = point_[0]
        point.y = point_[1]
        point.z = 0.0
        return point
    
    def publish_waypoints(self):
        
        start_marker = self.init_marker(g=1.0)
        start_marker.points.append(self.make_point(self.waypoints[0]))
        

        end_marker = self.init_marker(r=1.0)
        end_marker.points.append(self.make_point(self.waypoints[-1]))
        

        path_markers = self.init_marker(r=1.0, b=1.0)

        for point in self.waypoints[1:-1]:
            # point = Point()
            # point.x = x
            # point.y = y
            # point.z = 0.0
            path_markers.points.append(self.make_point(point))
        # print("MADE IT EVEN FURTHER")

        self.marker_pub.publish(start_marker)
        # print("PASSED FIRST PUBLISH")
        self.marker_pub.publish(end_marker)
        # print("PASSED SECOND PUBLISH")
        self.marker_pub.publish(path_markers)
        self.get_logger().info('Published waypoints marker.')

    def init_marker(self, r=0.0, g=0.0, b=0.0):
        """ gets the standard settings"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        return marker


    def pose_callback(self, pose_msg):
        # TODO - need to change how positions are read for the actual car

        if not self.waypoints:
            return
        
        # self.current_pose = pose_msg.pose

        if self.pose_topic == self.param["pose_topic"]:
            self.current_pose = pose_msg.pose
        else:
            self.current_pose = pose_msg.pose.pose
        # for simulation only ^^

        # find the nearest waypoint in front of the car
        current_position = np.array([self.current_pose.position.x, self.current_pose.position.y])
        indexed_dists = sorted([(index, np.linalg.norm(current_position - np.array(wp))) for index, wp in enumerate(self.waypoints)], key = lambda tup: tup[1])
        # sort by distances
        yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        heading_vector = np.array([np.cos(yaw), np.sin(yaw)])

        # find the first waypoint that is ahead of our lookahead distance
        goal_waypoint = None
        for i, distance in indexed_dists:
            if distance >= self.lookahead_distance:
                direction_to_goal = np.array(self.waypoints[i]) - current_position
                direction_to_goal_mag = np.linalg.norm(direction_to_goal)
                heading_mag = np.linalg.norm(heading_vector)

                # Calculate cosine of angle between vectors
                cos_angle = np.dot(heading_vector, direction_to_goal)
                actual_angle = np.arccos(cos_angle/(heading_mag*direction_to_goal_mag))
                # Ensure the angle is not pointing directly behind
                if np.degrees(actual_angle) < 120: # obtuse angle... no good?
                    goal_waypoint = self.waypoints[i]
                    break 
                    
        if goal_waypoint is None: 
            self.get_logger().info("No valid waypoint found within lookahead distance")
            return
        # transform goal point to the car's reference point
        goal_x, goal_y = self.transform_goal_point(goal_waypoint, current_position, yaw)


        # calculate curvature und steering angle 
        curvature = 2 * goal_y / (goal_x**2 + goal_y**2)
        steering_angle = np.arctan(curvature * self.lookahead_distance/self.set_speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = np.clip(steering_angle, -0.4189, 0.4189)  # limit steering anlge
        self.current_position = current_position
        self.steering_angle = steering_angle
        self.transformed_goal = goal_x, goal_y
        self.current_goal = goal_waypoint
        
        drive_msg.drive.speed = self.set_speed  # set constant speed
        self.last_position = np.array(goal_x, goal_y)
        self.drive_pub.publish(drive_msg)
        

    def log_info(self):
        """ 
        logs info twice a second,
        good for debugging
        """
        # if self.current_position is not None:
            # self.get_logger().info(f'Current position: x={self.current_position[0]}, y={self.current_position[1]}')
            # if self.current_goal is not None:
            #     self.get_logger().info(f'Goal waypoint: x={self.current_goal[0]}, y={self.current_goal[1]}')
            # self.get_logger().info(f'Transformed goal: x={self.transformed_goal[0]}, y={self.transformed_goal[1]}')
            # self.get_logger().info(f'Steering angle: {self.steering_angle}')
            # print('\n')


    def transform_goal_point(self, goal_point, current_position, yaw):
        """ 
        Transform goal point to vehicle reference frame
        """
        # current pose according to global frame
        # transform goal point to the vehicles "POV"
        translation_matrix = np.array([[np.cos(yaw), np.sin(yaw)],
                                      [-np.sin(yaw), np.cos(yaw)]])
        goal_vector = np.array(goal_point) - current_position
        transformed_goal = np.dot(translation_matrix, goal_vector)

        return transformed_goal[0], transformed_goal[1]
    
    def quaternion_to_yaw(self, orientation):
        """ convert quaternion to yaw angle. (converts to vehicle frame)"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z*orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        print('ended')

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
