#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA
import os
from ament_index_python.packages import get_package_share_directory
from nav_msgs.srv import GetMap
import math
import concurrent.futures
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
import csv  # may need this?
import ast
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time
from scipy.spatial import KDTree
from util import Occupancy, TreeNode
import yaml

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this


# TODO - so, I think with the global node, I should be able to pick a point, and draw a line between 
# TODO - that "checkpoint" and the borders of the track; 

config = os.path.join(
    get_package_share_directory("path_finder"),
    "config",
    "params.yaml"
)
# TODO - !HHH - SUPER IMPORTANT, make sure that rrt_node, and 
#       lbc_node start before the particle filter does


class DualSearch(Node):
    """ class to handle global and local search """
    def __init__(self):
        super().__init__('rrt_node')
        with open(config, "r") as f:
            self.param = yaml.safe_load(f)
        
        self.pose_topic = self.param["pose_topic"]
        scan_topic = "/scan"
        clicked_topic = "/clicked_point"
        waypoints = "/custom_waypoints"
        global_path_topic = "/global_path"
        map_topic = '/map'
        
        self.map_client = self.create_client(GetMap, "/map_server/map")
        self.get_map()
        # print("goes past")
        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # self.map_sub_ = self.create_subscription(
        #     OccupancyGrid,
        #     self.param["map_topic"],
        #     self.map_callback,
        #     10,
        # )

        # TODO- PAUSE MAP SUB FOR TESTING

        self.global_cb_group = ReentrantCallbackGroup()
        self.global_planner_timer = self.create_timer(1.0, self.update_publish_global, callback_group=self.global_cb_group)


        self.pose_sub_ = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            1)

        self.point_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.param["initial_pose_topic"],
            self.goal_callback,
            10
        )

        # publishers
        # self.drive_pub = self.create_publisher(
        #     AckermannDriveStamped,
        #     '/drive',
        #     10
        # )

        self.waypoint_pub = self.create_publisher(
            String,
            waypoints,
            10
        )
        self.global_path_pub = self.create_publisher(
            String,
            self.param["global_path_topic"],
            10
        )

        self.marker_pub = self.create_publisher(Marker, 'waypoints_marker', 10)

        # TODO - include the other listeners I have here
        # with open("data_file_room.json", 'r') as dt:
        #     info = dt.read()
        #     map_info = ast.literal_eval(info)
        # dt.close()
        # self.Grid = Occupancy(.06, (136, 142), (-1.56, -1.54), map_info)


        self.global_path = None
        self.local_path = None
        self.already_sampled = set()
        self.last_global_update = None
        self.global_update_interval = 5.0
        self.global_path_length = None

        # TODO - want to create the grids here so they 
        # TODO -- can both access the occupancy grid
        # self.Grid=None

        # for ROOM
        # self.Grid = Occupancy(.05, (141, 124), (-1.31, -4.25), map_info)

        # for TRACK
        # self.Grid = Occupancy(.057959, (2000,2000), (-84.85359914210505, -36.30299725862132), map_info)
        # self.global_planner = RRT(False, parent=self)
        # self.Grid = Occupancy()
        self.current_position = None, None
        self.goal = None
        self.goal_tolerance=None
        self.yaw = None
        self.coord_x,self.coord_y = None, None
        # self.goal represents the "global" goal
        self.kd_tree = None
        self.dist_tolerance = 0.50 # 50 centimeters for critical point selection
        # TODO - IN REAL CASE, WAIT FOR GRID TO INITIALIZE PLANNERS
        # self.global_planner = None
        self.mapped = False
        self.created_bfs = False
        self.checked_space = set()
        self.available_space = set()
        self.path = None
        self.ready = False
        self.start_yaw = None
        # self.local_planner = RRT(True, parent=self)


    def goal_callback(self, msg):
        """ converts clicked point to goal """
        # self.goal = self.Grid.pos_to_coord(msg.point.x, msg.point.y)
        # print(self.goal)
        print("should be ready")
        self.ready = True # self.ready is really only needed for testing purposes
        self.global_planner = RRT(False, parent=self)

    def get_map(self):
        """ collect the map from the particle filter """
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Get map service not available, waiting :(")
        req = GetMap.Request()
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        map_msg=future.result().map
        self.Grid = Occupancy(map_msg.info.resolution,
                              (map_msg.info.width, map_msg.info.height),
                              (map_msg.info.origin.position.x,
                               map_msg.info.origin.position.y),
                               map_msg.data)
        self.goal_tolerance = int(0.5/self.Grid.scale)
        


    # def map_callback(self, msg):
    #     """ only want to create one instance"""
    #     print("still receiving map!!")
    #     if not self.mapped: # only want to create 1 class instance
            
    #         # I believe width corresponds to x and height to y
    #         # TODO - assumes "static" map doesn't update
    #         self.Grid = Occupancy(msg.info.resolution, 
    #                         (msg.info.width, msg.info.height), 
    #                         (msg.info.origin.position.x,
    #                         msg.info.origin.position.y),
    #                         msg.data)
            
    #         # print((msg.info.resolution, 
    #         #                 (msg.info.width, msg.info.height), 
    #         #                 (msg.info.origin.position.x,
    #         #                 msg.info.origin.position.y)))
    #         print(self.Grid)
    #         self.goal_tolerance = int(0.5/self.Grid.scale)
    #         self.global_planner = RRT(False, parent=self)
    #         self.mapped=True


    def pose_callback(self, pose_msg):
        # print("pos called")
        if self.Grid is not None:
            if self.pose_topic == self.param["pose_topic"]:
                # == is real :)
                self.yaw = self.quaternion_to_yaw(pose_msg.pose.orientation)
                
                x = pose_msg.pose.position.x
                y = pose_msg.pose.position.y
            else:
                self.yaw = self.quaternion_to_yaw(pose_msg.pose.pose.orientation)
                x = pose_msg.pose.pose.position.x
                y = pose_msg.pose.pose.position.y
            if self.start_yaw is None:
                self.start_yaw = self.yaw
            self.coord_x, self.coord_y = self.Grid.pos_to_coord(x,y)
            # updates pos for both trees ^^
            if not self.created_bfs:
                self.Grid.develop_area()
                # self.waypoint_publish(False, self.available_space)
                # print("created!")
                self.created_bfs=True


            current_time = time.time()
            if self.goal is None:
                # make the goal - is circular now so should be able 
                # to use the yaw?? :( perhaps
                goal_distance = .4 # 40 cm behind car?
                yaw_vec = np.array([np.cos(self.yaw), np.sin(self.yaw)])
                yaw_vec *= goal_distance/self.Grid.scale 
                self.goal = tuple(np.array([self.coord_x, self.coord_y])-yaw_vec)
            if self.goal is not None and self.global_path is None and self.ready:
                print("updating global path")
                self.update_global_path()
                # if (current_time - self.last_global_update > self.global_update_interval) \
                #         or self.global_path is None:
                #     print("am here")
                #     self.last_global_update = current_time
                #     self.update_global_path()
                #     print("updated global path")

    def update_publish_global(self):
        if self.global_path is not None:
            self.update_global_path()

    
    def quaternion_to_yaw(self, orientation):
        """ convert quaternion to yaw angle. (converts to vehicle frame)"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z*orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    
    def update_global_path(self):
        """ retrieves global path - 
        - ensured path first run

        attains path in form of a list of tuples:
        
        [(coord node, {set of bar}, heading in radians 
        -- perhaps not correct heading)]
        """
        # print("UPDATING GLOBAL PATH")
        if self.global_path is None:
            while self.global_path is None:
                # print('working')
                # want to keep searching until we have at least a basic path
                self.global_path = self.global_planner.plan()
                # print(self.global_path)
        else:
            temp_global = self.global_planner.plan()
            self.global_path = temp_global if temp_global is not None else self.global_path
        
        self.publish_global_path()

    def publish_global_path(self):
        """ 
        published the global path as a string
        __repr__ function of Nodes automatically converts
        them to a tuple (self.x, self.y) form
        """
        msg = String()
        msg.data = str(self.global_path)
        self.global_path_pub.publish(msg)
        if self.last_global_update is None:
            self.last_global_update = time.time()
        self.get_logger().info("Sent the Global Path for Local Path Tracing")
        # self.global_path_length = len(self.global_path)

        
        # self.kd_tree = KDTree(np.array([item[0].get_coord() for item in self.global_path]))
 

    # def waypoint_publish(self, main_path=True, group=None):
    #     if self.path is None:
    #         self.path = self.init_marker(r=1.0)
        
    #     if not main_path:
    #         self.path = self.init_marker(b=1.0)
    #         for value in group:
    #             x,y = value
    #             point=Point()
    #             point.x=x
    #             point.y=y
    #             point.z=0.0
    #             self.path.points.append(point)
    #         self.marker_pub.publish(self.path)
            
    #     # print(f"{self.waypoints}")
    #     # x,y = self.waypoints[-1]
    #     # point = Point()
    #     # point.x = x
    #     # point.y = y
    #     # point.z = 0.0
    #     # self.path.points.append(point)

    #     self.marker_pub.publish(self.path)

    # def init_marker(self, r=0.0, g=0.0, b=0.0):
    #     """ gets the standard settings"""
    #     marker = Marker()
    #     marker.header.frame_id = "map"
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = "waypoints"
    #     marker.id = 0
    #     marker.type = Marker.POINTS
    #     marker.action = Marker.ADD
    #     marker.pose.orientation.w = 1.0
    #     marker.scale.x = 0.1
    #     marker.scale.y = 0.1
    #     marker.color.a = 1.0
    #     marker.color.r = r
    #     marker.color.g = g
    #     marker.color.b = b
    #     return marker

# class def for RRT
class RRT(DualSearch):  # TODO - could make it a child class of dual search node
    """
    have to modify this code skeleton to fit the RRT* algorithm
    current skeleton is better suited for the RRT algorithm

    """
    def __init__(self, am_i_local, parent):
        # super().__init__()
        # super().__init__('rrt_node')
        self.am_i_local = am_i_local
        # TODO - want specific behavior out of the local planner ^^
        self.waypoints = []

        # GRID STUFF -------------------
        # self.Grid = None  
        self.mapped=False
        self.tree = []
        self.new_goal=False
        # self.goal_tolerance = None
        self.car_width = .4 # meters
        self.path=None
        # self.yaw = 0  # initialize yaw for not_allowed_circle
        self.am_local = am_i_local
        self.parent = parent
        self.record = set()
        self.update=0
        
        
        # ------------------------------

        # MAP TESTING -----_@__!_!_!_#)@#@(_@#(_#@()#))
        # with open("data_file.json", 'r') as dt:
        #     info = dt.read()
        #     map_info = ast.literal_eval(info)
        # dt.close()
        # self.Grid = Occupancy(.05, (141, 124), (-1.31, -4.25), map_info)
        self.goal_tolerance = int(0.8/self.Grid.scale)

        # -------------------------------

    def __getattr__(self, name):
        return getattr(self.parent, name)


    def get_iteration_count(self): ## GLOBAL ##
        """
        calculates reasonable number of iterations for system 
        based on speed and computational resources
        """
        # TODO - incorporate this later - may also depend on speed of vehicle
        return 100

    def plan(self, point=None):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # pretty sure pose_msg.pose.pose.position.x is required (remove verify later?)

        if self.Grid is not None:
            if not self.tree:
                start = TreeNode(self.coord_x, self.coord_y)
                self.tree.append(start)
            else:
                # print("GETTING CAUGHT HERE")
                if self.no_longer_valid(self.coord_x, self.coord_y) or self.new_goal:
                    self.new_goal=False
                    self.replan(self.coord_x, self.coord_y)
                else:
                    # print("CHOOSING TO GROW TREE")
                    return self.grow_tree()

    def no_longer_valid(self, x, y): ## GLOBAL ##
        """ checks if current tree no longer valid, possible reasons:
        - not where I thought I would be 
        - unexpected obstacle found 
        - perhaps a timing based restart mechanism?
        """
        # TODO - this requires having an estimate of where we should be...
        # could calculate based on speed and the waypoints
        # need more development in other locations before this works
        return False


    def replan(self, x, y): ## GLOBAL ##
        """ 
        starts the tree over 
        because of unforseen condition
        """
        del self.tree[:]
        start = TreeNode(x, y)
        self.tree.append(start)
        self.grow_tree()
    
    def grow_tree(self):  ## GLOBAL ##
        """ expands the current tree"""
        # print(f"{self.goal=}")
        if self.goal is not None:

            # print("doing something?")
            for _ in range(self.get_iteration_count()): # number of iterations
                sampled_point = self.sample()
                # check_node = TreeNode(sampled_point[0], sampled_point[1])
                # self.waypoints.append(self.Grid.coord_to_pos(check_node))
                # self.waypoint_publish(group=self.waypoints)

                # print(f"{sampled_point=}")
                # print(f"{self.tree=}")
                nearest_index = self.nearest(self.tree, sampled_point)
                nearest_node = self.tree[nearest_index]
                new_node = self.steer(nearest_index, nearest_node, sampled_point)
                if new_node is None:
                    continue

                if not self.Grid.check_collision(nearest_node, new_node):
                    neighbors = self.near(self.tree, new_node)
                    min_cost_node = nearest_node
                    min_cost = nearest_node.cost + self.line_cost(nearest_node, new_node)

                    for neighbor in neighbors:
                        cost = neighbor.cost + self.line_cost(neighbor, new_node)
                        if cost < min_cost and not self.Grid.check_collision(neighbor, new_node):
                            min_cost = cost
                            min_cost_node = neighbor
                    new_node.parent = min_cost_node
                    new_node.cost = min_cost
                    self.tree.append(new_node)
                    self.already_sampled.add((new_node.x, new_node.y))

                    # only want to change it based on points we selected
                    if nearest_node.is_stem:
                        new_node.is_stem=True
                    elif nearest_index <=3:
                        nearest_node.is_stem=True

                    
                    # self.waypoints.append(self.Grid.coord_to_pos(new_node))
                    # self.waypoint_publish(group=self.waypoints)
                    
                    # self.waypoints.append(self.Grid.coord_to_pos((sampled_point[0], sampled_point[1])))
                    
                    for neighbor in neighbors:
                        cost = new_node.cost + self.line_cost(new_node, neighbor)
                        if cost < neighbor.cost and not self.Grid.check_collision(new_node, neighbor):
                            # print("DID SOMETHING")
                            neighbor.parent = new_node
                            neighbor.cost = cost
                    
                    if self.is_goal(new_node, *self.goal):
                        # -----
                        print("found goal?")
                        path = self.find_path(self.tree, new_node)
                        path[-1] = TreeNode(self.goal[0], self.goal[1])
                        # TODO - !L : make sure the sample rate not too high, 
                        #       meaning angle is less than 2 degrees for turns
                        return self.identify_critical_points(ang_tolerance=2, 
                                                             dist_tolerance=self.param["critical_point_tolerance"],
                                                              coords=path)

        else:
            print("HUH")

    def waypoint_publish(self, main_path=True, group=None):
        if self.path is None:
            self.path = self.init_marker(r=1.0)
        
        if not main_path:
            self.path = self.init_marker(b=1.0)
            for value in group:
                x,y = value
                point=Point()
                point.x=x
                point.y=y
                point.z=0.0
                self.path.points.append(point)
            self.marker_pub.publish(self.path)
            
        # print(f"{self.waypoints}")
        x,y = self.waypoints[-1]
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        self.path.points.append(point)

        self.marker_pub.publish(self.path)

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
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        return marker

    def sample(self): ## GLOBAL ##
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point
        """
        x = self.Grid.random_point(self.Grid.width) 
        y = self.Grid.random_point(self.Grid.height)
        

        # trying system where it returns a tuple of smaller area
        # x,y = self.Grid.random_point(None)
        point = (x,y)

        # TODO - !H - 
        if not self.in_no_sample_zone(x,y) and point not in self.already_sampled:
            # self.already_sampled.add(point)
            return point
        return self.sample()
    
    def in_no_sample_zone(self, x, y): ## GLOBAL ##
        """ circles around car that represent turn radius, won't search here"""
        # TODO - CAN INSTEAD GIVE PSEUDO-POSITION OF CAR... MEANING EACH FIRST POINT HAS "POSITION" OF CAR
        # print(f"{self.coord_x=} {self.coord_y=}")
        turning_radius = 0.3 / self.Grid.scale  # meters to pixels
        # yaw is zero when in tune with the x-axis, not y axis 
        # print(f"{self.yaw=}")
        
        yaw = self.yaw +np.pi/2
        left_circle_center = (
            self.coord_x - turning_radius * np.cos(yaw),
            self.coord_y - turning_radius * np.sin(yaw)
        )

        right_circle_center = (
            self.coord_x + turning_radius * np.cos(yaw),
            self.coord_y + turning_radius * np.sin(yaw)
        )
        # print(f"{left_circle_center=}{right_circle_center=}")
        dist_to_left = np.sqrt((x-left_circle_center[0])**2 + (y-left_circle_center[1])**2)
        dist_to_right = np.sqrt((x-right_circle_center[0])**2 + (y-right_circle_center[1])**2)

        return dist_to_left <= turning_radius or dist_to_right<=turning_radius

    def nearest(self, tree, sampled_point): ## GLOBAL ##
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_index = 0
        min_dist = float('inf')
        for i,node in enumerate(tree):
            dist = LA.norm(node - sampled_point)
            # ([sampled_point[0] - node.x, sampled_point[1] - node.y])
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        return nearest_index

    def steer(self, nearest_index, nearest_node, sampled_point): ## GLOBAL ##
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        # TODO - !H - need to adjust to create a closed loop - either by picking points
        #        relatively in front of the node (may limit shortest path opportunities )
        #       or by adjusting the goal and path conditions
        changed = False # use for stem checking
        direction = (nearest_node*(-1)) + sampled_point # equivalent to 
        # np.array([sampled_point[0] - nearest_node.x, sampled_point[1]-nearest_node.y])
        # print(f"{direction=}")
        length = LA.norm(direction)
        # print(f"{length=}")
        if length==0.0:
            direction=np.array([0,0])
        else: direction = direction/length
        if nearest_index <=3 or nearest_node.is_stem:
            # first_nodes_v = self.tree[1] - self.tree[0]
            # node_heading = np.arctan2(first_nodes_v[1], first_nodes_v[0])
            node_heading = self.start_yaw
            # self.yaw # just give it yaw haha - close enough
            pseudo_heading = np.arctan2(direction[1], direction[0])
            heading_tolerance = np.radians(40)
            
            if abs(pseudo_heading-node_heading) > heading_tolerance:
                return None
        # TODO - !H 
        # CHOOSING CANDIDATE VERY DEPENDENT ON THIS STEP SIZE
        # TODO - !HH - step size should be large at first, then get smaller when improving path

        step_size = int(self.param["steer_step_size"] / self.Grid.scale)
        new_point = (nearest_node + (direction*step_size)).astype(int)
        return TreeNode(new_point[0], new_point[1], parent=nearest_node)
    
    

    def identify_critical_points(self, ang_tolerance, dist_tolerance, coords): ## GLOBAL ##
        """ 
        simplifies the path found from RRT to what is needed 

        ang_tolerance: accepted in degrees and converted into radians
        dist_tolerance: accepted in meters used in meters
        
        returns the critical points :)
        """
        # TODO - CAN PROBABLY ALSO EXTRACT HEADING OUT OF THIS (COULD BE USEFUL FOR PATH FINDING)
        # critical_points = [coords[0]]
        # TODO - CHECK THIS LOGIC
        # TODO - IF I WANT TO CLASSIFY BY STRAIGHT OR CURVES... THEN NEED TO PREPARE LATER THINGS
        # TODO - TO ACCEPT THINGS OF THAT LENGTH :)))
        # TODO - !H
        critical_points = []


        length = len(coords)
        dist=0
        for i in range(length): # was range(1, length-1)
            v1 = coords[i] - coords[i-1]
            v2 = coords[(i+1)%length-1] - coords[i]
            dist += LA.norm(coords[i]-coords[i-1])
            
            # angle = self.calculate_angle(v1, v2)
            # angle > np.radians(ang_tolerance) or 
            if (dist/self.Grid.scale >= dist_tolerance):
                dist=0
                v21 = v2 + v1
                segment_len = LA.norm(v21)
                if segment_len==0:
                    continue
                # print(f"{v21=}")
                # print(f"{segment_len=}")
                perp = (np.roll(v21, shift=1)*(-1,1))/segment_len
                # TODO - !H - STILL NEED TO DEAL WITH WHEN LA.norm is zero6
                bar = set()
                new_pos = TreeNode(coords[i].x, coords[i].y)

                # need to check this heading logic
                heading = np.arctan2(v21[1],v21[0]) # y/x 
                mult=0
                while not self.Grid.check_collision(new_pos, new_pos, kd_mode=True) and abs(mult)<10:
                    mult +=1
                    bar.add((new_pos.x, new_pos.y))
                    new_value = (new_pos + (perp*mult)).astype(int)
                    new_pos.x = new_value[0]
                    new_pos.y = new_value[1]
                mult=0
                new_pos = TreeNode(coords[i].x, coords[i].y)

                while not self.Grid.check_collision(new_pos, new_pos, kd_mode=True) and abs(mult)<10:
                    mult -=1
                    bar.add((new_pos.x, new_pos.y))
                    new_value = (new_pos + (perp*mult)).astype(int)
                    new_pos.x = new_value[0]
                    new_pos.y = new_value[1]
                bar.add((coords[i].x, coords[i].y))
                critical_points.append((coords[i], bar, heading))
        bar_collection = []
        print(f"should be {len(critical_points)} points ----------------------------")
        for coord, bar, heading in critical_points:
            value = self.Grid.coord_to_pos(coord)
            self.waypoints.append(value)
            self.waypoint_publish()
            

            for value in bar:
                bar_collection.append(self.Grid.coord_to_pos(value))

        self.waypoint_publish(False, bar_collection)

        self.waypoint_publish()



        return critical_points

            # else:
            #     dist = np.linalg.norm(np.array[coords[i].x, coords[i].y] - np.array(critical_points[-1].x, critical_points[-1].y))
            #     if dist > dist_tolerance:
            #         critical_points.append(coords[i])
            # critical_points.append(coords[-1])

    def calculate_angle(self, v1, v2): ## GLOBAL ##
            """ calculates the angle between consecutive points """
            unit_v1 = v1 / LA.norm(v1)
            unit_v2 = v2 / LA.norm(v2)
            dot_product = np.dot(unit_v1, unit_v2)
            return np.arccos(dot_product)

    
        
    def is_goal(self, latest_added_node, goal_x, goal_y): ## GLOBAL ##
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        # TODO - !M - for it to be classified as a goal, heading of final point
        #       needs to approximately match the original heading of the car 
        # TODO - !H - not here, but also consider it a goal condition 
        #       when one half of tree meets other half, perhaps this is done by giving 
        #       an ID to the first points depending on their heading related to the car

        # haha, seems all I had to do was not permit certain angles next to the start :0
        # TODO - !H - kind of slow... since it only searches from one direction, in the future
        #       can make it two search trees that join - harder to mitigate that from one tree (in fast manner)
        # -----
        # TODO - !HHHH - ADD GOAL LOGIC TO CHECK IF IT IS GOING IN DIRECTION OF INITIAL YAW
        counter=0
        node = latest_added_node
        subbed_values = np.array([goal_x - latest_added_node.x, goal_y-latest_added_node.y])
        sub_norm = subbed_values / LA.norm(subbed_values)
        dotted = np.dot(sub_norm, np.array([np.cos(self.start_yaw), np.sin(self.start_yaw)])) 
        if (LA.norm(subbed_values) <= self.goal_tolerance) and dotted > 0 and self.time_valid():
            while node is not None:
                counter+=1
                node = node.parent
                truthy = counter > 12 
                if truthy:
                    self.update +=1
                    print(f"dotted value is {dotted=}")
                    return truthy


        return False
        # return (LA.norm([goal_x - latest_added_node.x, goal_y - latest_added_node.y]) <= self.goal_tolerance) and counter>10

    def time_valid(self):
        """ creates a grace period for the updates - 10 seconds?"""
        if self.last_global_update is not None:
            return time.time() - self.last_global_update < 10
        return True
    

    def find_path(self, tree, latest_added_node): ## GLOBAL ##
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        node = latest_added_node
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse() # last added should be the first step :)
        return path


    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node): ## GLOBAL ##
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return node.cost
        assert NotImplementedError

    def line_cost(self, n1, n2): ## GLOBAL ##
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straight line
        Returns:
            cost (float): the cost value of the line
        """
        # TODO - remember that distances are in terms of pixels
        cost = LA.norm(n1-n2)  
        return cost
        # for now cost is just determined by distance

    def near(self, tree, node): ## GLOBAL ##
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighbors ([]): neighborhood of nodes as a list of Nodes
        """
        # make radius 1
        radius = int(1 / self.Grid.scale)
        neighbors = []
        for n in tree:
            if LA.norm(node-n) <= radius:
                # means that the node is "near"
                neighbors.append(n)
        return neighbors
    

    def publish_path(self, path):
        msg = String()
        data = tuple(map(self.Grid.coord_to_pos, path))
        msg.data = str(data)
        print(ast.literal_eval(msg.data))
        # print(msg.data)
        length = len(data)
        self.waypoint_pub.publish(msg)
        # TODO- MAY NEED TO INTERPOLATE POINTS BEFORE SENDING 
        self.get_logger().info(f"Sent waypoints containing {length} points")
        
def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    dual_rrt = DualSearch()
    executor = MultiThreadedExecutor()
    executor.add_node(dual_rrt)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        dual_rrt.destroy_node()
        rclpy.shutdown()
    # rclpy.spin(dual_rrt)

    # dual_rrt.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
