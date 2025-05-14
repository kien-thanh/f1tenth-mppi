# # file to help me practice grabbing information from map :)
# import numpy as np

# # static_struct = np.array([[]])

# # def static(...):
# #     """ 
# #     function grabs information from map to create 'static' layer
# #     want a quick lookup idea
# #     """

# # def dynamic(...):
# #     """ 
# #     could be created by processing the LiDar Data and 
# #     adding something that was not present in the static environment 
# #     (if the object is far enough away from other points so that it 
# #     cannot be the same point read incorrectly)
# #     """

# # TODO - write a code that computes the path of travel ÖÖÖ 

# def find_grid_path(start, stop, width):
#     """ finds the affected coordinates """
#     direct_vector = stop-start
#     length = int(np.linalg.norm(direct_vector))
#     print(f"{length=}")
#     direct_vector = direct_vector / length
#     offset = int(width / 2)
#     print(f"{offset=}")
#     path = []
#     square = [np.array([x,y]) for x in range(start[0]-offset, start[0]+(offset+1)) for y in range(start[1]-offset, start[1]+(offset+1))]
#     path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
#     # might be easiest to populate a square and run the for loop on each point in the square
#     print(path)
#     # for 


# start = np.array([31, 27])
# stop = np.array([30, 27])
# width = .3/.05

# find_grid_path(start, stop, width)


# def random_point(limiter):
#         """ 
#         finds a random point in the 'open space' 
#         """
#         value = np.random.uniform(0, limiter)
#         return int(value) 

# def sample():
#         """
#         This method should randomly sample the free space, and returns a viable point

#         Args:
#         Returns:
#             (x, y) (float float): a tuple representing the sampled point

#         """
#         x = random_point(142) 
#         y = random_point(125)

#         return (x, y)

# print(sample())


# import numpy as np

# length = np.linalg.norm([-2-3, -5-18])
# print(f"LENGTH IS {length}")

# import numpy as np


# def identify_critical_points(self, ang_tolerance, dist_tolerance):
#     """ 
#     simplifies the path found from RRT to what is needed 
#     returns the critical points :)
#     """
#     critical_points = [coords[0]]
#     length = len(coords)
#     for i in range(1, length-1):
#         v1 = np.array(coords[i]) - np.array(coords[i-1])
#         v2 = np.array(coords[i+1]) - np.array(coords[i])
        
#         angle = calculate_angle(v1, v2)
#         if angle > np.radians(ang_tolerance):
#             v21 = v2 - v1
#             perp = np.linalg.norm(v21)/v21
#             bar = set()
#             new_pos = TreeNode(coords[i].x, coords[i].y)
#             while not check_collision(coords[i], new_pos):
#                 mult +=1
#                 bar.add((new_pos.x, new_pos.y))
#                 new_pos.x = int(new_pos.x *perp[0]*mult)
#                 new_pos.y = int(new_pos.y *perp[1]*mult)

#             while not check_collision(coords[i], new_pos):
#                 mult -=1
#                 bar.add((new_pos.x, new_pos.y))
#                 new_pos.x = int(new_pos.x *perp[0]*mult)
#                 new_pos.y = int(new_pos.y *perp[1]*mult)
#             critical_points.append((coords[i], bar))
#         else:
#             dist = np.linalg.norm(np.array[coords[i]] - np.array(critical_points[-1]))
#             if dist > dist_tolerance:
#                   critical_points.append(coords[i])
#         critical_points.append(coords[-1])

# def calculate_angle(v1, v2):
#         """ calculates the angle between consecutive points """
#         unit_v1 = v1 / np.linalg.norm(v1)
#         unit_v2 = v2 / np.linalg.norm(v2)
#         dot_product = np.dot(unit_v1, unit_v2)
#         return np.arccos(dot_product)


# def create_bars():
#       """ given points, creates the bars """


# coords = [
#         (0.0, 0.0),
#         (1.0, 1.0),
#         (2.0, 2.0),
#         (3.0, 1.0),
#         (4.0, 0.0),
#         (5.0, -1.0),
#         (6.0, -2.0),
#         (7.0, -1.0),
#         (8.0, 0.0),
#         (9.0, 1.0),
#         (10.0, 2.0)
#     ]



## THESE ARE CUTS FROM RRT ## 

class TreeNode:
    # TODO - ADD FUNCTIONS TO EASILY SUBTRACT TREE NODES 
    # TODO - ALSO SEE IF YOU CAN ADD SOME SORT OF LEN() 
    # TO FIND THE DISTANCE BETWEEN THEM
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0 if parent is None else float('inf') # only used in RRT*
        # self.is_root = False
    
    def __sub__(self, other):
        """ subtracts the difference between a node and other item"""
        # need to make sure that other has len 2
        if hasattr(other, 'x'):
            # get the attributes
            return np.array([self.x - other.x, self.y-other.y])

        else:
            # probably a numpy array
            return np.array([self.x - other[0], self.y - other[1]])
        
    def __mul__(self, other):
        """ runs multiplication """

        if hasattr(other, 'x'):
            # maybe works somewhat like dot product?
            return np.array([self.x * other.x, self.y*other.y])
        else:
            # integer or float multiplication
            return np.array([self.x * other, self.y * other])
        
    def __add__(self, other):
        """ adds two together, +1 func call compare to sub"""
        return self - (-1*other)
    
    def __div__(self, other):
        raise NotImplementedError
    
    def get_coord(self):
        """ returns the current coords in tuple form """
        return np.array([self.x, self.y])
    
    

    
    # def calc_distance(self, other):
    #     """ grabs the distance between two points """
    #     return LA.norm(self - other)

class Occupancy:
    """ 
    Class for managing the occupancy layers 
    """

    def __init__(self, scale, dimensions, origin, static_grid):
        """
        Args: 
            dimensions - tuple width and height
            static_grid - tuple of all map locations
        """
        
        # with open("data_file.json", 'w') as dt:
        #     dt.write(str(list(static_grid)))
        #     print("wrote the file")
        # dt.close()
        
        self.dynamic_grid = set()
        self.scale = scale
        self.width = dimensions[0]
        self.height = dimensions[1]
        self.origin = origin
        self.static_grid = self.initialize_static(static_grid)
        print(self.scale, self.width,self.height, self.origin)

    # possible functions I will need

    def __add__(self, pos):
        """ dunder add for dynamic graph """
        if pos[0]*pos[1] < self.width*self.height:
            self.dynamic_grid.add(pos)
        else:
            raise IndexError("Out of the map's range") 
    
    def __contains__(self, pos):
        """ checks if a position is open or not """
        return pos in self.static_grid | self.dynamic_grid

    
    # def __del__(self, pos):
    #     """ if something was present in the dynamic before,
    #     but is no longer, remove it 
    #     """
    #     if pos in self.dynamic_grid:
    #         self.dynamic_grid.remove(pos)


    def pos_to_coord(self, x, y):
        """ finds pixel x and y given real world position"""
        min_x = self.origin[0]
        pos_x = int(max(min((x - min_x)//self.scale, self.width), 0))
        min_y = self.origin[1]
        pos_y = int(max(min((y-min_y)//self.scale, self.height), 0))
        # ensure it stays within bounds ^^ 
        return pos_x, pos_y

    def initialize_static(self, static_grid):
        """  data is in row-major order
         returns pixel coordinates self.width x self.height 
        - unknown points will be left as unoccupied"""

        return set((index - (self.width * (y:=index//self.width)), y) for 
                   index, value in enumerate(static_grid) if value == 100)
    
        # modulo operator is slow, mitigating by seeing where the y value 
        # is... 
        
    def random_point(self, limiter):
        """ 
        finds a random point in the 'open space' 
        """

        value = np.random.uniform(0, limiter)
        if value in self:
            # not free - search again 
            value = self.random_point(limiter)
        return int(value) 

    
    def coord_to_pos(self, node):
        """ transforms into the map's perspective 
        TODO - this may be better suited outside the class
        """
        min_x = self.origin[0]
        min_y = self.origin[1]
        if hasattr(node, 'x'):
            x = (node.x * self.scale)+min_x
            y = (node.y* self.scale)+min_y
        else:
            # assume can separate by index
            x = (node[0] * self.scale)+min_x
            y = (node[1]* self.scale)+min_y

        return (x,y)




def update_local_path(self):
    local_goal = self.extract_local_goal()
    # local path comes in the form of coords (in an array?)
    self.local_path = self.local_planner.plan(local_goal)

def extract_local_goal(self): ## LOCAL ##
    """
    uses the consecutive nature of the path to find next position
    """
    # TODO - !L - later change the lookahead distance to be based
    #       on the speed of the vehicle :)

    # TODO - !H - once the testing works normally, 
    # -- use s (straight) and c (curve) 
    # checks if the closest point then looks for the next curve point 
    # and grabs the last point (looks for the next curve and goes back one index)
    lookahead_dist = 1.0 # meters
    lookahead_shift = int(lookahead_dist / self.dist_tolerance)

    current_pose = np.array([self.coord_x, self.coord_y])
    distance, index = self.kd_tree.query(current_pose)
    lookahead_index = (index + lookahead_shift) % self.global_path_length

    # returns the local goal
    return self.global_path[lookahead_index]

    # TODO - VERY IMPORTANT 
    # easiest way to do this would be to have a constant distance between points
    # so indexing finds the distance I want from the position I have...
    # although this could introduce jittering for straighter sections
    # meaning I could also implement like "straight" and "curve" flags, 
    # ^^ where instead of looking ahead a certain distance for straight,
    #        I just aim for the furthest point ! :)
    # blah = KDTree(some_data, metric=self.dist_finder)



    # TODO - WILL USE CUSTOM FUNCTION FOR NOW, BUT SHOULD TEST SPEED
    # LATER AND THEN PROBABLY CHANGE IT - LIKELY NEED A BIG RESTRUCTURE 
    # IN GENERAL FOR OPTIMIZATION?? NOT ALL CALCULATIONS FEEL EFFICIENT ATM


def update_publish_local(self): ## LOCAL ##
    if self.kd_tree is not None:
        self.update_local_path()
        self.publish_local_path()


def scan_callback(self, scan_msg):
    """
    LaserScan callback, you should update your occupancy grid here

    Args: 
        scan_msg (LaserScan): incoming message from subscribed topic
    Returns:
        2nd layer to the ocupancy grid

    """
    # logic will be implemented later perhaps... is a little complicated 
    # because it depends on the accuracy of positioning system... 
    # might be better for shorter term obstacles , like cars
    # like dynamic should be used for moving objects / quick reactions.. focus on latr
    # assert NotImplementedError

    pass


def publish_local_path(self): ## LOCAL ##
    """ sends the path to waypoint follower """
        # while True:
    #     print("AHAHAHAHA")
    msg = String()
    data = tuple(map(self.Grid.coord_to_pos, self.local_path))
    msg.data = str(data)
    # print(msg.data)
    length = len(data)
    self.waypoint_pub.publish(msg)
    # TODO- MAY NEED TO INTERPOLATE POINTS BEFORE SENDING 
    self.get_logger().info(f"Sent waypoints containing {length} points")

# if self.am_local and point is not None:
#     priority = point[0]
#     # ^^ don't do anything with priority pick for now 
#     candidates = point[1]
#     entry_angle = point[2]
#     return self.candidate_selection(candidates, entry_angle)

# else:


# if self.am_local:
#     # first want to get the bars

#     return self.identify_critical_points(ang_tolerance=2, dist_tolerance=self.dist_tolerance, coords=path)
# else:
#     return path
# should correctly send the path back
# self.publish_path(path)
# break

def candidate_selection(self, candidates, angle, control_point_dist = 1, num_angles=5): ## LOCAL ##
        """ uses possible points and entry_angle buffer to decide best paths 
        
        candidates: must be iterable
        angle: must be radians

        """
        # TODO - NEED TO INTEGRATE THIS WITH OTHER LOCAL BEHAVIOR!! AND TEST THIS HAHAHA
        # TODO - MAKE SURE TO ALSO TEST THE BAND CREATION, DON'T KNOW IF THAT WORKS YET
        p0 = self.coord_x, self.coord_y
        best_path = None
        best_cost = float('inf')
        best_p1 = best_p2 = None
        angle_buffer = np.radians(5) # current degree buffer in radians
        angle_range = np.linspace(angle-angle_buffer, angle+angle_buffer, num_angles)

        for candidate in candidates:
            for angle in angle_range:
                # create the control points 
                # TODO - P0 and candidate are both Node objects, so need to do a __sub__ method
                direction_vector = np.array([np.cos(angle), np.sin(angle)])
                shift = control_point_dist * direction_vector
                p1 = p0 + shift
                p2 = candidate - shift

                path = self.bezier_cubic(p0, p1, p2, candidate)

                cost = self.compute_cost(path)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path
                    best_p1, best_p2 = p1, p2

        print(best_cost)
        # best_cost, best_p1, best_p2
        return best_path

                # now

def bezier_cubic(self, p0, p1, p2, p3, num_points=50): ## LOCAL ##
    """ creates bezier cubic points based on 4 contact points"""
    # TODO - !H -- this probably needs some testing to see how to position X and Y
    #       -- also need to figure out how to convert to real coords
    # TODO !H -- different ways to see if a point is invalid...
    #        could make a square and see if any of those points in the OccupancyGrid...
    #        could find closest point and see if it is further than the car's width
    #        implement the square method rn and benchmark later 
    t_values = np.linspace(0, 1, num_points)
    curve = []
    checked = set()
    for t in t_values:
        point = ((1-t)**3 * np.array(p0) + 3*(1-t)**2 * t * np.array(p1) + 3*(1-t)*t**2 * np.array(p2) + t**3*np.array(p3)).astype(int)
        # print("THE POINT HERE IS THAT IT IS ")
        # print(f"{point=}")
        # print("AND THAT IS ALL")
        point_node = TreeNode(point[0], point[1])
        check_point = tuple(point)
        # must be hashable ^^
        if check_point not in checked:
            checked.add(check_point)
            if self.check_collision(point_node, point_node):
                return # don't want to get anything from it.. just give up
            curve.append(point)
    return np.array(curve)

def compute_cost(self, path): ## LOCAL ##
    """ simple cost estimation based on length and curvature """
    length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
    curvature = np.sum(np.abs(np.diff(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0])))))
    return length + 10*curvature


def angle_within_band(self, desired_angle, band_min, band_max): ## LOCAL ##
    return band_min <= desired_angle <= band_max










def check_collision(self, nearest_node, new_node): ###########
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        width = self.car_width / self.Grid.scale  # gives us the car width in terms of coordinate squares 
        # assuming new_node is the target (stop) and near_pos is start
        direct_vector = new_node - nearest_node
        length = int(np.linalg.norm(direct_vector))
        direct_vector = (direct_vector / length) if length>0 else np.array([0,0])
        offset = int(width/2)
        square = [np.array([x,y]) for x in range(nearest_node.x-offset, nearest_node.x+(offset+1)) 
                  for y in range(nearest_node.y-offset, nearest_node.y+(offset+1))]
        # TODO - PROBABLY A QUICKER WAY TO DO THE SQUARE
        path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
        # TODO - could potentially be quicker to check while creating the path, depending on how large the path is
        return any(location in self.Grid for location in path) 