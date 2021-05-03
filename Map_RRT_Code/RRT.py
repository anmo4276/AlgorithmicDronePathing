import numpy as np
import matplotlib.pyplot as plt
import math
import random
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d

mapp = np.load("map.npy")

class Node:
    """
    Node for RRT Algorithm. This is what you'll make your graph with!
    """
    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)

def get_nd_obstacle(state_bounds, x, y):
    '''
    Function to return a circular obstacle in an n-dimensional world
    :param state_bounds: Array of min/max for each dimension
    :return: A single circular obstacle in form of a list with 1st entry as the circle center and the 2nd as the radius
    '''
    center_vector = []
    center_vector.append(x)
    center_vector.append(y)
    radius = 10 # Downscaling the radius
    return [np.array(center_vector), radius]

def setup_random_2d_world():
    '''
    Function that sets a 2D world with fixed bounds and # of obstacles
    :return: The bounds, the obstacles, the state_is_valid() function
    '''
    state_bounds = np.array([[0,1080],[0,1080]]) # matrix of min/max values for each dimension
    obstacles = [] # [pt, radius] circular obstacles
    for i in range(len(mapp)):
        for j in range(len(mapp[i])):
            if mapp[i][j] == 1:
                obstacles.append(get_nd_obstacle(state_bounds, j, i))

    def state_is_valid(state):
        '''
        Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
        :param state: n-Dimensional point
        :return: Boolean whose value depends on whether the state/point is valid or not
        '''
        for dim in range(state_bounds.shape[0]):
            if state[dim] < state_bounds[dim][0]: return False
            if state[dim] >= state_bounds[dim][1]: return False
        for obs in obstacles:
            if np.linalg.norm(state - obs[0]) <= obs[1]: return False
        return True

    return state_bounds, obstacles, state_is_valid

def setup_fixed_test_2d_world():
    '''
    Function that sets a test 2D world with fixed bounds and # of obstacles
    :return: The bounds, the obstacles, the state_is_valid() function
    '''
    state_bounds = np.array([[0,1080],[0,1080]]) # matrix of min/max values for each dimension
    obstacles = [] # [pt, radius] circular obstacles
    obstacles.append([[0.5,0.5],0.2])
    obstacles.append([[0.1,0.7],0.1])
    obstacles.append([[0.7,0.2],0.1])

    # Pretty wild but you can have nested functions in python and in this case it will retain
    # its local variables state_bounds and obstacles. You won't need to pass them later.
    def state_is_valid(state):
        '''
        Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
        :param state: n-Dimensional point
        :return: Boolean whose value depends on whether the state/point is valid or not
        '''
        for dim in range(state_bounds.shape[0]):
            if state[dim] < state_bounds[dim][0]: return False
            if state[dim] >= state_bounds[dim][1]: return False
        for obs in obstacles:
            if np.linalg.norm(state - obs[0]) <= obs[1]: return False
        return True

    return state_bounds, obstacles, state_is_valid

def _plot_circle(x, y, radius, color="-k"):
    '''
    Internal function to plot a 2D circle on the current pyplot object
    :param x: The x coordinate of the circle
    :param y: The y coordinate of the circle
    :param radius: The radius of the circle
    :param color: Matplotlib color code
    :return: None
    '''
    deg = np.linspace(0,360,50)

    xl = [x + radius * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + radius * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)

def visualize_2D_graph(state_bounds, obstacles, nodes, goal_point=None, filename=None):
    '''
    Function to visualise the 2D world, the RRT graph, path to goal if goal exists
    :param state_bounds: Array of min/max for each dimension
    :param obstacles: Locations and radii of spheroid obstacles
    :param nodes: List of vertex locations
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param filename: Complete path to the file on which this plot will be saved
    :return: None
    '''
    fig = plt.figure()
    plt.xlim(state_bounds[0,1], state_bounds[0,0])
    plt.ylim(state_bounds[1,1], state_bounds[1,0])

    for obs in obstacles:
        _plot_circle(obs[0][0], obs[0][1], obs[1])

    goal_node = None
    for node in nodes:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
            plt.plot(node_path[:,0], node_path[:,1], '-b') # Blue lines that reach every point
        # The goal may not be on the RRT so we are finding the point that is a 'proxy' for the goal
        if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5:
            goal_node = node
            plt.plot(node.point[0], node.point[1], 'k^') # End point triangle
        else:
            plt.plot(node.point[0], node.point[1], 'ro') # All red dots

    plt.plot(nodes[0].point[0], nodes[0].point[1], 'ko') # starting point black dot

# No idea what this is for
    pathing_nodes = []
    if goal_node is not None:
        cur_node = goal_node
        while cur_node is not None: 
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                pathing_nodes.append(cur_node)
                plt.plot(node_path[:,0], node_path[:,1], '-y')
                cur_node = cur_node.parent
            else:
                break
    
    point_list = []
    for k in range(len(pathing_nodes)):
        (x, y) = pathing_nodes[k].point[0], pathing_nodes[k].point[1]
        point_list.append((x,y))
    point_list.append((nodes[0].point[0], nodes[0].point[1]))
    
    new_path_list = []
    for k in point_list:
        new_path_list.append((k[0]/30, (1080-k[1])/30))
    print(new_path_list)
    
    if goal_point is not None:
        plt.plot(goal_point[0], goal_point[1], 'gx') # The end point green X

    if filename is not None:
        fig.savefig(filename)
    else:
        plt.show()

def get_random_valid_vertex(state_is_valid, bounds):
    '''
    Function that samples a random n-dimensional point which is valid (i.e. collision free and within the bounds)
    :param state_valid: The state validity function that returns a boolean
    :param bounds: The world bounds to sample points from
    :return: n-Dimensional point/state
    '''
    vertex = None
    while vertex is None: # Get starting vertex
        pt = np.random.rand(bounds.shape[0]) * (bounds[:,1]-bounds[:,0]) + bounds[:,0]
        if state_is_valid(pt):
            vertex = pt
    return vertex

###############################################################################
## END BASE CODE
###############################################################################

def get_nearest_vertex(node_list, q_point):
    '''
    Function that finds a node in node_list with closest node.point to query q_point
    :param node_list: List of Node objects
    :param q_point: n-dimensional array representing a point
    :return Node in node_list with closest node.point to query q_point
    '''

    # TODO: Your Code Here
    num = float('inf')
    closest_node = node_list[0]
    
    for i in node_list:
        closest_point = np.linalg.norm(i.point - q_point)
        if closest_point < num:
            num = closest_point
            closest_node = i
    return closest_node

def steer(from_point, to_point, delta_q):
    '''
    :param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
    :param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
    :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
    :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
    '''
    # TODO: Figure out if you can use "to_point" as-is, or if you need to move it so that it's only delta_q distance away
    if (np.linalg.norm(to_point - from_point) < delta_q):
        new_to_point = to_point
    else:
        smaller_vector = ((to_point - from_point) / np.linalg.norm(to_point - from_point)) * delta_q
        new_to_point = smaller_vector + from_point
    path = np.linspace(from_point, new_to_point, num=10)
    
    return path

def check_path_valid(path, state_is_valid):
    '''
    Function that checks if a path (or edge that is made up of waypoints) is collision free or not
    :param path: A 1D array containing a few (10 in our case) n-dimensional points along an edge
    :param state_is_valid: Function that takes an n-dimensional point and checks if it is valid
    :return: Boolean based on whether the path is collision free or not
    '''

    # TODO: Your Code Here
    test = True
    for i in path:
        if state_is_valid(i) == False:
            test = False
    return test
#     raise NotImplementedError

def rrt(state_bounds, state_is_valid, starting_point, goal_point, k, delta_q):
    '''
    TODO: Implement the RRT algorithm here, making use of the provided state_is_valid function.
    RRT algorithm.
    If goal_point is set, your implementation should return once a path to the goal has been found 
    (e.g., if q_new.point is within 1e-5 distance of goal_point), using k as an upper-bound for iterations. 
    If goal_point is None, it should build a graph without a goal and terminate after k iterations.

    :param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
    :param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
    :param starting_point: Point within state_bounds to grow the RRT from
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param k: Number of points to sample
    :param delta_q: Maximum distance allowed between vertices
    :returns List of RRT graph nodes
    '''
    node_list = []
    node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent
    
    while len(node_list) <= k:
        new_to_point = get_random_valid_vertex(state_is_valid, state_bounds)
        if (goal_point is not None):
            if random.random() < 0.05:
                new_to_point = goal_point
        nearest_point = get_nearest_vertex(node_list, new_to_point)
        path = steer(nearest_point.point, new_to_point, delta_q)
        new_to_point = path[len(path)-1]
        if check_path_valid(path, state_is_valid) == False:
            continue
        new_node = Node(new_to_point, nearest_point)
        new_node.path_from_parent = path
        node_list.append(new_node)
        if (goal_point is not None):
            if np.linalg.norm(new_to_point - goal_point) < 1e-5:
                break
                
    #RRT*
    
#     for itr in range(k):
#         new_to_point = get_random_valid_vertex(state_is_valid, state_bounds)
#         if (goal_point is not None):
#             if random.random() < 0.05:
#                 new_to_point = goal_point
#         nearest_point = get_nearest_vertex(node_list, new_to_point)
#         path = steer(nearest_point.point, new_to_point, delta_q)
#         new_to_point = path[len(path)-1]
#         if check_path_valid(path, state_is_valid) == False:
#             continue
#         new_node = Node(new_to_point, nearest_point)
#         new_node.path_from_parent = path
#         node_list.append(new_node)
#         if (goal_point is not None):
#             if np.linalg.norm(new_to_point - goal_point) < 1e-5:
#                 break

    return node_list


if __name__ == "__main__":
    K = 250 # Feel free to adjust as desired

    bounds, obstacles, validity_check = setup_random_2d_world()
    starting_point = np.array([200,100]) # Start Point 
    goal_point = np.array([950,700]) # End Point
    print("----------------START POINT----------------")
    print(starting_point)
    print("----------------END POINT----------------")
    print(goal_point)
    nodes = rrt(bounds, validity_check, starting_point, goal_point, K, np.linalg.norm(bounds/10.))
    
    point_list = []
    for k in range(len(nodes)):
        (x, y) = nodes[k].point[0], nodes[k].point[1]
        point_list.append((x/30,(1080-y)/30))
    print("----------------LIST OF ALL POINTS----------------")
    print(point_list)
    print("----------------LIST OF TRAVEL POINTS----------------")
    visualize_2D_graph(bounds, obstacles, nodes, goal_point, 'rrt_goal_run2.png')