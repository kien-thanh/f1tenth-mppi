#!/usr/bin/env python3

""" 
Local Bezier Curve Node
- local operations 

TODO - SHARED PARAMETERS SHOULD BE IN A YAML FILE
TODO - !H - JUST MAKE A __repr__ func for the TreeNode class to show tuple of coords
        -- gonna want to make the KDTree on this side I believe... 
"""
import os
import rclpy
from rclpy.node import Node
import numpy as np 
from scipy.spatial import KDTree
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from util import TreeNode, Occupancy
import ast
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import time
import yaml
from ament_index_python.packages import get_package_share_directory


config = os.path.join(
    get_package_share_directory("path_finder"),
    "config",
    "params.yaml"
)

class LBC(Node):
    """ class to process local """
    def __init__(self):
        super().__init__('lbc_node')
        with open(config, "r") as f:
            self.param = yaml.safe_load(f)

        self.pose_topic = self.param["pose_topic"]
        scan_topic = "/scan"
        waypoints = "/custom_waypoints"
        global_path_topic = "/global_path"
        map_topic = '/map'
        self.map_client = self.create_client(GetMap, "/map_server/map")
        self.get_map()

        # usually PoseStamped
        self.pose_sub_ = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            1
        )

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            self.param["scan_topic_sim"],
            self.scan_callback,
            1
        )
        
        self.global_path_sub_ = self.create_subscription(
            String,
            self.param["global_path_topic"],
            self.global_path_callback,
            1,
        )


        self.waypoint_pub = self.create_publisher(
            String,
            self.param["pure_pursuit_topic"],
            10,
        )
        self.marker_pub = self.create_publisher(Marker, 'waypoints_marker', 10)

        # with open("data_file_circular.json", 'r') as dt:
        #     info = dt.read()
        #     map_info = ast.literal_eval(info)
        # dt.close()

        self.global_path = None
        self.global_path_length = None
        self.kd_tree = None
        self.local_path = None
        self.coord_x, self.coord_y = None, None
        self.dist_tolerance = 0.50 # meters ## depends on the search distance (for nearby in rrty)
        self.yaw = None
        # FOR TESTING ------------------
        # ROOM
        # self.Grid = Occupancy(.05, (141, 124), (-1.31, -4.25), map_info)

        # TRACK
        # self.Grid = Occupancy(.057959, (2000,2000), (-84.85359914210505, -36.30299725862132), map_info)

        # ------------------------------
        self.mapped = False
        self.checked = set() # checked bezier points
        self.known_failures = set() # checked bezier points that fail
        self.start_time = time.time()
        self.counter=0
        self.speed = 1.0
        self.min_length,self.max_length = 20, 50 #empirically gathered
        self.min_turning_radius = .80 # I believe it's 70 cm
        self.lookahead_dist=None
        self.recursion_depth=0

    def pose_callback(self,pose_msg):
        """ updates position and runs localizer (if can) """
        
        if self.pose_topic == self.param["pose_topic"]: # this one is prepped for real world
            x,y = pose_msg.pose.position.x, pose_msg.pose.position.y
            self.yaw = self.quaternion_to_yaw(pose_msg.pose.orientation)
        else: # this should be prepped for simulation
            x,y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
            self.yaw = self.quaternion_to_yaw(pose_msg.pose.pose.orientation)

         
        self.coord_x, self.coord_y = self.Grid.pos_to_coord(x,y)
        if self.kd_tree is not None:
            # print('should be updating local path')
            self.update_local_path()
            self.publish_local_path()
    
    def get_map(self):
        """ should fetch the map that the particle filter is using """
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Get map service not available, waiting")
        req = GetMap.Request()
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        map_msg=future.result().map
        self.Grid = Occupancy(map_msg.info.resolution, 
                            (map_msg.info.width, map_msg.info.height), 
                            (map_msg.info.origin.position.x,
                            map_msg.info.origin.position.y),
                            map_msg.data)
        print("got the map message!!!")

    def quaternion_to_yaw(self, orientation):
        """ convert quaternion to yaw angle. (converts to vehicle frame)"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z*orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        pass

    # def map_callback(self, msg):
    #     print("Calling map callback")
    #     """ only want to create one instance"""
    #     if not self.mapped: # only want to create 1 class instance
    #         # I believe width corresponds to x and height to y
    #         # TODO - assumes "static" map doesn't update
    #         self.Grid = Occupancy(msg.info.resolution, 
    #                         (msg.info.width, msg.info.height), 
    #                         (msg.info.origin.position.x,
    #                         msg.info.origin.position.y),
    #                         msg.data)
    #         print(self.Grid)
    #         self.goal_tolerance = int(0.5/self.Grid.scale)
    #         self.mapped=True

    def global_path_callback(self, msg):
        """ use the path given and construct trees etc """
        print("calling global map callback")
        # TODO - !L -- will be better in the future to create custom ROS2 message type -> lower overhead
        # print(msg.data)
        # print(type(msg.data))
        # print(ast.literal_eval(msg.data))
        self.global_path = tuple(map(lambda x: (np.array(x[0]), x[1], x[2]), ast.literal_eval(msg.data)))
        self.global_path_length = len(self.global_path)
        # print(self.global_path)
        self.kd_tree = KDTree(tuple(item[0] for item in self.global_path))

    def waypoint_publish(self, main_path=True, group=None):
        if not main_path:
            self.path = self.init_marker(g=1.0)
            for value in group:
                x,y = value
                point=Point()
                point.x=x
                point.y=y
                point.z=0.0
                self.path.points.append(point)
            self.marker_pub.publish(self.path)
            
        # print(f"{self.waypoints}")
        # x,y = self.waypoints[-1]
        # point = Point()
        # point.x = x
        # point.y = y
        # point.z = 0.0
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


    def plan(self, point=None):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # pretty sure pose_msg.pose.pose.position.x is required (remove verify later?)
        # print("CALLING PLAN")

        if self.Grid is not None:
            priority = point[0]
            # ^^ don't do anything with priority pick for now 
            candidates = point[1]
            entry_angle = point[2]
            control_point_dist = 0.35 / self.Grid.scale # 1 meter # was .40 before (.40 works very well except turn 1) (.3 works great turn one, not turn 2)
            control_point_dist_exit = 0.85 / self.Grid.scale # was .7 before (.85 and .9 and (sometimes .8) work decently for turn radius)
            return self.candidate_selection(priority,candidates, entry_angle, control_point_dist, control_point_dist_exit)

            # TODO - should do parallel processing with local and global so no interference
            # TODO - try out once having tested this current alg

    

    def candidate_selection(self, main_point, candidates, angle, control_point_dist, exit_control, multiplier=1, num_angles=5):
        """ uses possible points and entry_angle buffer to decide best paths 
        
        candidates: must be iterable
        angle: must be radians

        """
        # TODO - NEED TO INTEGRATE THIS WITH OTHER LOCAL BEHAVIOR!! AND TEST THIS HAHAHA
        # TODO - MAKE SURE TO ALSO TEST THE BAND CREATION, DON'T KNOW IF THAT WORKS YET
        p0 = self.coord_x, self.coord_y
        # p0 = p0 - (0.05/self.Grid.scale)*np.array([np.cos(self.yaw), np.sin(self.yaw)])
        best_path = None
        best_cost = float('inf')
        best_p1 = best_p2 = None
        angle_buffer = np.radians(10) # current degree buffer in radians
        angle_range = np.linspace(-angle_buffer, angle_buffer, num_angles)
        threshold_distance = 1.1 # meters
        candidate_threshold = 4
        num_candidates = len(candidates)
        candid_distance = np.linalg.norm(np.array(main_point) - p0) * self.Grid.scale
        if candid_distance < threshold_distance:
                ratio = candid_distance / threshold_distance
                exit_control *= ratio
                control_point_dist *=ratio
        elif num_candidates < candidate_threshold:
            ratio = num_candidates / candidate_threshold
            exit_control *= ratio
            control_point_dist *=ratio
        

        for candidate in candidates:
            
            for angle_buff in angle_range:
                # create the control points 
                # TODO - P0 and candidate are both Node objects, so need to do a __sub__ method
                direction_vector = np.array([np.cos(angle+angle_buff), np.sin(angle+angle_buff)])
                

                ex_shift = exit_control * direction_vector
                entrance_vector = np.array([np.cos(self.yaw), np.sin(self.yaw)])
                ent_shift = control_point_dist * entrance_vector * multiplier
                # TODO - !M -  currently have simplified bezier quintic
                #       may want to institute more complex way to define p2,p3
                p1 = p0 + ent_shift
                p2 = p1 + ent_shift*.50
                p4 = candidate - ex_shift
                p3 = p4 - ex_shift*.60 #adding a little more weight here to see if
                # it helps with sharp turns

                path = self.bezier_quintic(p0,p1,p2,p3,p4,candidate)

                # path = self.bezier_cubic(p0, p1, p2, candidate)
                # print(self.compute_cost(path))
                cost, length, curvature = self.compute_cost(path)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path
                    best_length, best_curvature = length, curvature
                    best_p1, best_p2 = p1, p2
        # if best_cost == float('inf'):
        #     # try generating a simpler curve
        #     for candidate in candidates:
        #         for angle in angle_range:
        #             direction_vector = np.array([np.cos(angle), np.sin(angle)])
        #             ex_shift = exit_control * direction_vector
        #             entrance_vector = np.array([np.cos(self.yaw), np.sin(self.yaw)])
        #             ent_shift = control_point_dist * entrance_vector
        #             p1 = p0 + ent_shift
        #             p2 = p3 - ex_shift
        #             path = self.bezier_cubic(p0,p1,p2, candidate)
        #             if cost < best_cost:
        #                 best_cost = cost
        #                 best_path = path
        #                 best_length, best_curvature = length, curvature

        if best_cost==float("inf"):
            if self.recursion_depth<6:
                self.recursion_depth+=1
                if self.candidate_selection(main_point,candidates, angle, control_point_dist, exit_control, multiplier=1/.75) is None:
                    return self.candidate_selection(main_point,candidates, angle, control_point_dist, exit_control, multiplier=.75)

            elif self.recursion_depth<12:
                self.recursion_depth+=1
                return self.candidate_selection(main_point,candidates, angle, control_point_dist, exit_control, multiplier=.75)
            print("NOOOO")
            print(best_cost)
            
            return None, None
        # print(best_cost)
        speed = self.compute_speed(best_length, best_curvature)
        # best_cost, best_p1, best_p2
        return best_path, speed
    

    def bezier_quadratic(self, p0, p1, p2, num_points=15):
        # print(f"{p0=},{p1=},{p2=}")
        A = 0.5 * np.linalg.norm(np.cross(p1-p0, p2-p1))
        m = 0.5 * (p0+p2)
        k0 = A / (np.linalg.norm(p1-p0)**3)
        k1 = A / (np.linalg.norm(p2-p1)**3)
        # print(f"{np.linalg.norm(p1-m)=}")
        k_max_curvature = (np.linalg.norm(p1-m)**3) / (A**2)
        midpoint_dist = np.linalg.norm(p2-m)
        if midpoint_dist < np.linalg.norm(p1- (.5*(p0+m))) and midpoint_dist < np.linalg.norm(p1-(.5*(m+p2))):
            max_curvature = k_max_curvature
        else:
            max_curvature = max(k0,k1)
        if 1/max_curvature < self.min_turning_radius:
            print("CURVATURE FAILURE")
            # return
        t_values = np.linspace(0, 1, num_points)
        curve = []
        for t in t_values:
            # print("trying a value")
            point = ((1 - t) ** 2 * np.array(p0) +
                        2 * ((1 - t)) * t * np.array(p1) +
                         t ** 2 * np.array(p2)).astype(int)
            # int_point = point.astype(int)
            check_point = tuple(point)
            point_node = TreeNode(point[0], point[1])
            if check_point in self.known_failures:
                # print("CAUGHT A FAILURE")
                return 
            # elif check_point not in self.checked:
            #     self.checked.add(check_point)
            #     if self.Grid.check_collision(point_node, point_node, kd_mode=True):
            #         # print("CAN FIND COLLISIONS")
            #         self.known_failures.add(check_point)
            #         return  # don't want to get anything from it.. just give up
            curve.append(point)
        return np.array(curve)


    def bezier_quintic(self, p0, p1, p2, p3, p4, p5, num_points = 30):
        """ creates a quintic bezier curve for more flexibility in angle """
        # TODO - H -- likely much much quicker to interpolate the other points,
        #        and only generate about 10 - interpolate the rest because you 
        #        don't really need to check them all again :(  
        #        interpolation would require one of the values to constantly increase..
        #           may not be possible unless simply go by gradient

        t_values = np.linspace(0, 1, num_points)
        curve = []
        for num, t in enumerate(t_values):
            # print("trying a value")
            point = ((1 - t) ** 5 * np.array(p0) +
                        5 * ((1 - t) ** 4) * t * np.array(p1) +
                        10 * ((1 - t) ** 3) * (t**2) * np.array(p2) +
                        10 * ((1 - t) ** 2) * (t**3) * np.array(p3) +
                        5 * (1 - t) * (t ** 4) * np.array(p4)
                        + (t ** 5) * np.array(p5)).astype(int)
            # int_point = point.astype(int)
            check_point = tuple(point)
            point_node = TreeNode(point[0], point[1])
            if check_point in self.known_failures:
                # print("CAUGHT A FAILURE")
                return 
            elif check_point not in self.checked:
                self.checked.add(check_point)
                if self.Grid.check_collision(point_node, point_node, kd_mode=True):
                    # print("CAN FIND COLLISIONS")
                    self.known_failures.add(check_point)
                    return  # don't want to get anything from it.. just give up
            curve.append(point)
        return np.array(curve)
            

    def bezier_cubic(self, p0, p1, p2, p3, num_points=30): ## LOCAL ##
        """ creates bezier cubic points based on 4 contact points"""
        # TODO - !H -- this probably needs some testing to see how to position X and Y
        #       -- also need to figure out how to convert to real coords
        # TODO !H -- different ways to see if a point is invalid...
        #        could make a square and see if any of those points in the OccupancyGrid...
        #        could find closest point and see if it is further than the car's width
        #        implement the square method rn and benchmark later 
        t_values = np.linspace(0, 1, num_points)
        curve = []
        for t in t_values:
            # print("trying a value")
            point = ((1-t)**3 * np.array(p0) + 3*(1-t)**2 * t * np.array(p1) + 3*(1-t)*t**2 * np.array(p2) + t**3*np.array(p3)).astype(int)
            point_node = TreeNode(point[0], point[1])
            check_point = tuple(point)
            # must be hashable ^^
            if check_point in self.known_failures:
                # print("CAUGHT A FAILURE")
                return 
            elif check_point not in self.checked:
                self.checked.add(check_point)
                if self.Grid.check_collision(point_node, point_node):
                    # print("CAN FIND COLLISIONS")
                    self.known_failures.add(check_point)
                    return  # don't want to get anything from it.. just give up
            curve.append(point)
        # print(f"{curve=}")
        return np.array(curve)

    def compute_cost(self, path): ## LOCAL ##
        """ simple cost estimation based on length and curvature """
        if path is None:
            return float("inf"), None, None
        else:
            length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
            if length < self.min_length:
                self.min_length=length
            elif length > self.max_length:
                self.max_length=length
            curvature = np.sum(np.diff(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0]))))  # taking out the abs for now np.abs() after np.sum()
            # TODO -H --more accurate curvature given by dividing by length --
            return (length**2 + 10*(curvature), length, curvature)
    
    def compute_speed(self, length, curvature):
        """ takes minimum speed, maximum speed and curvature / length - outputs target speed"""
        min_speed=1.0
        max_speed=1.5
        normalized_curve = abs(curvature) / (abs(curvature) + 1) # (length +1)
        normalized_length = self.normalize_length(length)
        # normalized_length = length / (length + 1)
        speed = (max_speed * (normalized_length *(abs(1-normalized_curve)))) + min_speed
        # print(f"{normalized_curve=}, {normalized_length=}, {speed=}, {self.min_length=}, {self.max_length=}")
        speed = max(min_speed, min(speed, max_speed))
        
        print(f"{speed=}")
        return speed

    def normalize_length(self, length):
        return (length - self.min_length)/ (self.max_length - self.min_length)

    def angle_within_band(self, desired_angle, band_min, band_max): ## LOCAL ##
        return band_min <= desired_angle <= band_max
    
    def update_local_path(self):
        local_goal = self.extract_local_goal()
        # local path comes in the form of coords (in an array?)
        self.local_path, self.speed = self.plan(local_goal)

    def extract_local_goal(self):
        """
        uses the consecutive nature of the path to find next position
        """
        # TODO - !L - later change the lookahead distance to be based
        #       on the speed of the vehicle :)

        # TODO - !H - once the testing works normally, 
        # -- use s (straight) and c (curve) 
        # checks if the closest point then looks for the next curve point 
        # and grabs the last point (looks for the next curve and goes back one index)
        self.lookahead_dist = 1.5 # meters
        lookahead_shift = int((self.lookahead_dist / self.param["critical_point_tolerance"])+0.60) # .5 rounds

        current_pose = np.array([self.coord_x, self.coord_y])
        distance, index = self.kd_tree.query(current_pose)
        lookahead_index = (index + lookahead_shift) % (self.global_path_length-1)
        # while np.linalg.norm(np.array([self.coord_x, self.coord_y]) - self.global_path[lookahead_index][0])*self.Grid.scale < self.lookahead_dist:
        #     lookahead_index = (lookahead_index +1) % (self.global_path_length-1)
        # # print(f"{lookahead_shift=}")
        # returns the local goal
        return self.global_path[lookahead_index]
    
    def publish_local_path(self):
        """ sends the path to waypoint follower """
        self.counter+=1
        if time.time() - self.start_time > 1:
            self.start_time = time.time()
            print(f"{self.counter} publishes in a second! -----------------------------------")
            self.counter=0
        # TODO - !M - For now, just ignore if it doesn't return a path
        # TODO - !HHH - will need to send more information to the path follower
        #       so that it knows better how to follow the points and make sure 
        #       it doesn't turn too early
        if self.local_path is not None:
            msg = String()
            data = (tuple(map(self.Grid.coord_to_pos, self.local_path)), self.speed)
            self.waypoint_publish(False, data[0])
            msg.data = str(data)
            # print(msg.data)
            length = len(data)
            self.waypoint_pub.publish(msg)
            # TODO- MAY NEED TO INTERPOLATE POINTS BEFORE SENDING 
            # self.get_logger().info(f"Sent waypoints containing {length} points")

def main(args=None):
    rclpy.init(args=args)
    print("LOCALIZER INITIALIZED")
    localizer = LBC()
    rclpy.spin(localizer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()