"""
File for holding custom data structures and common functions 

"""
import numpy as np
from scipy.spatial import KDTree
import random
import time

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
        
        # with open("data_file_room.json", 'w') as dt:
        #     dt.write(str(list(static_grid)))
        #     print("wrote the file")
        # dt.close()
        
        self.dynamic_grid = set()
        self.car_width = 0.3 # 30 centimeter
        self.scale = scale
        self.width = dimensions[0]
        self.height = dimensions[1]
        self.origin = origin
        self.static_grid = self.initialize_static(static_grid)
        print(self.scale, self.width,self.height, self.origin)
        self.KD_tree = KDTree(tuple(item for item in self.static_grid))
        self.available_space = set()
        self.checked_space = set()

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
        # value = random.sample(self.available_space,1)[0] # returns an item in list form  #np.random.uniform(0, limiter)
        # value = self.available_space.pop()
        if value in self: # or value not in self.available_space
            # not free - search again 
            value = self.random_point(limiter)
        # return int(value) 
        return value

    
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
    
    def develop_area(self):
        offset = int(2 / self.scale)
        this_space = set()
        # print(f"{self.static_grid=}")
        for x_,y_ in self.static_grid:
            if (x_, y_) not in self.available_space:
                for value in [(x,y) for x in range(x_-offset, x_+(offset+1)) 
                        for y in range(y_-offset, y_+(offset+1))]:
                    this_space.add(value)
                    self.available_space.add(value)

    
    
    def check_collision(self, nearest_node, new_node, kd_mode=False):
        # TODO - check collision mechanism NOT GOOD ENOUGH! 
        # too slow, and for large maps just not even possible really
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
        if kd_mode is True:
            # see if the kd_search is quicker
            current_pose = np.array([nearest_node.x, nearest_node.y])
            distance, index = self.KD_tree.query(current_pose)
            return distance * self.scale  <= (self.car_width*.67) 
            # .75 adds a little buffer since really we only need half width
                 
        else:
            # assuming new_node is the target (stop) and near_pos is start
            direct_vector = new_node - nearest_node
            length = int(np.linalg.norm(direct_vector))
            direct_vector = (direct_vector / length) if length>0 else np.array([0,0])
            square = [np.array([nearest_node.x, nearest_node.y])]
            path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
            for position in path:
                distance, index = self.KD_tree.query(position)
                if distance*self.scale <= (self.car_width*.75):
                    return True
            
            return False

class TreeNode:
    # TODO - ADD FUNCTIONS TO EASILY SUBTRACT TREE NODES 
    # TODO - ALSO SEE IF YOU CAN ADD SOME SORT OF LEN() 
    # TO FIND THE DISTANCE BETWEEN THEM
    def __init__(self, x, y, parent=None, is_stem=False):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0 if parent is None else float('inf') # only used in RRT*
        self.is_stem = is_stem # for the beginning nodes, to prevent backward movement :)

    
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
    
    def __repr__(self):
        return f"({self.x}, {self.y})"

