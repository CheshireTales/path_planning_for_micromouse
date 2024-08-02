import cv2
import numpy as np
import random
import math
from matplotlib import pyplot as plt

random.seed(42)

# DO NOT ADD OTHER VARIABLE HERE

# General settings
seed = 42
image_file = "sample_map.png"

# Task 1 - Occupancy map
unsafe_kernel_size = 7
unsafe_iterations = 3

# Task 3 - BFS
size_of_node_grid = 10
bfs_start_node = 0
bfs_end_node = 99

# Task 4 - PRM and Dijkstra
iterations = 500
k_connections = 3
max_connection_range = 100
start_location = (100,100)
goal_location = (450,450)

# Task 5 - RRT
goal_radius = 20
step = 40

# DO NOT ADD OTHER VARIABLE HERE

# Task 1.1 (1 mark): Display the occupancy map
img_original = cv2.imread(image_file)

# Display resulting image
plt.imshow(img_original)
plt.show()

# Task 1.2 (2 Marks): Process occupancy map add and display configuration space
kernel = np.ones((unsafe_kernel_size, unsafe_kernel_size), np.uint8) # TODO: Use this kernenl to generate un_safe area
img_unsafe = cv2.erode(img_original,kernel, iterations = unsafe_iterations)

img_dif = cv2.subtract(img_original,img_unsafe)
buffer_mask = np.all(img_dif == [255,255,255], axis=-1)
img_original[buffer_mask]=[255,0,0]
img_dangerzone = img_original.copy()
plt.imshow(img_dangerzone)

# Display resulting image
plt.show()

# Task 2.1 (3 marks):  Complete the graph class

class Node:
    def __init__(self, node_id, x, y):
        self.id = node_id
        self.x = x
        self.y = y
    
    def get_point(self):
        return (self.x,self.y)
    
    def get_ID(self):
        return self.id

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}

    def add_node(self, node_id, x, y):
        # Only make node if node does not exist
        node_list = self.nodes
        if node_id not in node_list:
            new_node = Node(node_id,x,y)
            node_list[node_id] = new_node
            # create empty dictionary to store node for other side of the edge 
            self.edges[node_id] = {}
        pass
    
    def check_edge_exists(self,node_id1,node_id2):
        edge_list = self.edges
        if node_id1 in edge_list and node_id2 in edge_list:
            if node_id2 in edge_list[node_id1]:
                return True
            else:
                return False
        else:
            return False

    def add_edge(self, node_id1, node_id2, weight):
        node_list = self.nodes
        edge_list = self.edges
        # Check if nodes exist
        nodes_exist = node_id1 in node_list and node_id2 in node_list
        edge_exists = self.check_edge_exists(node_id1,node_id2)
        if nodes_exist and not edge_exists:
            edge_list[node_id1][node_id2] = weight
            edge_list[node_id2][node_id1] = weight
        pass

    def remove_edge(self, node_id1, node_id2):
        edge_list = self.edges
        edge_exists = self.check_edge_exists(node_id1,node_id2)
        # Check if edge exists
        if edge_exists:
            edge_list [node_id1][node_id2] = None
            edge_list [node_id2][node_id1] = None
        pass
    
    def get_nodes(self):
        return list(self.nodes.keys())
    
    def get_edge_weight(self, node_id1, node_id2):
        edge_list = self.edges
        edge_exists = self.check_edge_exists(node_id1,node_id2)
        if edge_exists:
            edge_weight = edge_list[node_id1][node_id2]
            return edge_weight
        return None
# This is a useful function which you may choose to impliment and use 
# It looks through the image to see if the path is clear between one cooridnate to the next
# Returns True or False 
def path_clear(image, x1, y1, x2, y2):
    x1 = int(x1)
    x2 = int(x2)
    y1 = int(y1)
    y2 = int(y2)

    x_lower = min(x1,x2)
    x_higher = max(x1,x2)
    
    y_lower = min(y1,y2)
    y_higher = max(y1,y2)
    
    white_pixel = [255, 255, 255]
    
    # SPECIAL: Vertical case
    if (x1==x2):
        for y in range(y_lower,y_higher):
            if not (image[y, x1] == white_pixel).all():
                return False
    # SPECIAL: Horizontal case
    elif (y1==y2):
        for x in range(x_lower,x_higher):
            if not (image[y1, x] == white_pixel).all():
                return False  
        
    # All other cases
    else:
        m = (y2-y1)/(x2-x1)
        b = -m*x1+y1
        for x in range(x_lower,x_higher):
            y = math.ceil(m*x+b)
            if not (image[y, x] == white_pixel).all():
                return False
        
    return True

# Task 3.1 (4 marks): Generate a grid of nodes and connect the edges
#bfs_image = cv2.imread(image_file)
bfs_image = img_dangerzone.copy()
bfs_graph = Graph()
height, width,_ = img_dangerzone.shape
n = size_of_node_grid
x_space = width/(n+1)
y_space = height/(n+1)

# Generate nodes
x = width/(n+1)
y = height/(n+1)
node_list = bfs_graph.nodes
for id in range(bfs_start_node,bfs_end_node+1):
    bfs_graph.add_node(id,x,y)
    if ((x+x_space)>= (width-1)):
        x = width/(n+1)
        y += y_space
    else:
        x += x_space
# Generate edges
# Neighboring nodes are: node+1, node-1, node-n, node+n
# Check node exists in node list
    # If node exists, check path is clear between given nodes
row = 1
edge_list = bfs_graph.edges
for node in range(bfs_start_node,bfs_end_node+1):
    node_this = node_list[node]
    if ((node+1) in node_list) and ((node+1) % n != 0):
        if path_clear(img_dangerzone,node_this.x,node_this.y,node_list[node+1].x,node_list[node+1].y):
            bfs_graph.add_edge(node,node+1,1)
    if ((node-1) in node_list) and ((node) % n != 0):
        if path_clear(img_dangerzone,node_this.x,node_this.y,node_list[node-1].x,node_list[node-1].y):
            bfs_graph.add_edge(node,node-1,1)
    if ((node+n) in node_list):
        if path_clear(img_dangerzone,node_this.x,node_this.y,node_list[node+n].x,node_list[node+n].y):
            bfs_graph.add_edge(node,node+n,1)
    if ((node-n) in node_list):
        if path_clear(img_dangerzone,node_this.x,node_this.y,node_list[node-n].x,node_list[node-n].y):
            bfs_graph.add_edge(node,node-n,1)
# Draw Graph
# Draw Nodes
for node in range(0,100):
    x = math.ceil(node_list[node].x)
    y = math.ceil(node_list[node].y)
    cv2.circle(bfs_image,(x,y), 3, (0,255,0), -1)
# Draw Edges
num_edges = 2*n*(n-1)
edge_list = bfs_graph.edges
for node1 in range (bfs_start_node,bfs_end_node):
    x1 = int(node_list[node1].x)
    y1 = int(node_list[node1].y)
    #print("node1:",x1,y1)
    for node2 in edge_list[node1]:
        x2 = int(node_list[node2].x)
        y2 = int(node_list[node2].y)
        #print("node2:",x2,y2)
        distance = math.sqrt((x2-x1)**2+(y2-y1)**2)
        if (distance < (width/n)):
            cv2.line(bfs_image,(x1,y1),(x2,y2),(0,125,0),1)
            
# Draw numbers
font = cv2.FONT_HERSHEY_SIMPLEX
node_startx = int(node_list[bfs_start_node].x)
node_starty = int(node_list[bfs_start_node].y)
node_endx = int(node_list[bfs_end_node].x)
node_endy = int(node_list[bfs_end_node].y)
cv2.putText(bfs_image,str(bfs_start_node),(node_startx,node_starty),font,0.75,(0,255,0),1,cv2.LINE_AA)
cv2.putText(bfs_image,str(bfs_end_node),(node_endx,node_endy),font,0.75,(0,255,0),1,cv2.LINE_AA)

# Display Resulting Image
plt.imshow(bfs_image)
plt.show()

# Task 3.2 (4 marks): Impliment BFS

# Returns an array of nodes in order of which nodes is visited next.
# ie. [0, 10, 20, 30, 40, 50, 60, 70, 71, 72, 73, 74, 75, 76, 77, 78, 88, 98, 99]
def bfs(graph, start_node_id, end_node_id): 
    # Initialize queue
    visited = []
    queue = [[start_node_id]]
    while queue:
        # Start path
        path = queue.pop(0)
        current_node = path[-1]
        # Found the end!
        if current_node == end_node_id:
            return path
        # If haven't been explored, explore branch
        if current_node not in visited:
            visited.append(current_node)
            edge_list = graph.edges
            adjacent_nodes = list(edge_list[current_node].keys())
            for adjacent in adjacent_nodes:
                # add new adjacent node to section of branch
                new_path = list(path)
                new_path.append(adjacent)
                queue.append(new_path)
                
    return []

# Task 3.2: Impliment BFS continued

# Run bfs and display the output
path = bfs(bfs_graph,bfs_start_node,bfs_end_node)

# Draw path
for current in range(len(path) - 1):
    node_id1 = path[current]
    node_id2 = path[current+1]                  
    x1 = int(node_list[node_id1].x)
    y1 = int(node_list[node_id1].y)
    x2 = int(node_list[node_id2].x)
    y2 = int(node_list[node_id2].y)          
    cv2.line(bfs_image,(x1,y1),(x2,y2),(0,0,255),3)
                      
                          
# Display resulting image
print(f"Path: {path}")
plt.imshow(bfs_image)
plt.show()

# This is a useful function which you may choose to impliment and use 
# It finds and returns the n closest nodes which are within the range
def find_closest_nodes(image,graph, target_x, target_y, n,range):

    close_nodes = []
    # Iterate through nodes to find all within range
    node_list = graph.nodes
    for node in node_list:
        test_nodex = node_list[node].x
        test_nodey = node_list[node].y
        distance = math.sqrt((target_x-test_nodex)**2+(target_y-test_nodey)**2)
        if (distance < range) and (path_clear(image,target_x,target_y,test_nodex,test_nodey) and not ((test_nodex,test_nodey)==(target_x,target_y))):
            close_nodes.append((node,distance))
    close_nodes.sort(key = lambda x: x[1]) 
    return close_nodes[:n]

find_closest_nodes(bfs_image,bfs_graph,300,350,3,100)

# Task 4.1 (3 marks): PRM
# NOTE: The iteration only increases when a valid node is placed on the map. If the node is invalid, the program should continue without incrementing the iteration count. 

prm_image = img_dangerzone.copy()
prm_graph = Graph()
random.seed(seed)
node_list = prm_graph.nodes
# Generate random nodes where there no obstacles and draw
i = 0
while i < iterations:
    y = random.randint(0, prm_image.shape[0]-1)
    x = random.randint(0, prm_image.shape[1]-1)
    # add node if not on obstacle
    white_pixel = (255,255,255)
    if (prm_image[y, x] == white_pixel).all():
        i += 1
        prm_graph.add_node(i,int(x),int(y))
        cv2.circle(prm_image,(x,y), 3, (0,255,0), -1)
        neighbors = find_closest_nodes(img_dangerzone,prm_graph,int(x),int(y),k_connections,max_connection_range)
        for a in range (0,len(neighbors)):
            node_idx = neighbors[a][0]
            node_dist = neighbors[a][1]
            test_nodex = int(node_list[node_idx].x)
            test_nodey = int(node_list[node_idx].y)
            if (path_clear(img_dangerzone,int(x),int(y),test_nodex,test_nodey)):
                prm_graph.add_edge(i,node_idx,node_dist)
                cv2.line(prm_image,(int(x),int(y)),(test_nodex,test_nodey),(0,0,255),1)

stragglers = []
for id in edge_list:
    if len(edge_list[id]) == {}:
        stragglers.append(id)
        straggler_friends = find_closest_nodes(img_dangerzone,prm_graph,node_list[id].x,node_list[id].y,k_connections,max_connection_range)
        for s in range (0,len(neighbors)):
            node_idx = straggler_friends[s][0]
            node_dist = straggler_friends[s][1]
            test_nodex = int(node_list[node_idx].x)
            test_nodey = int(node_list[node_idx].y)
            if (path_clear(img_dangerzone,node_list[id].x,node_list[id].y,test_nodex,test_nodey)):
                prm_graph.add_edge(id,node_idx,node_dist)
                cv2.line(prm_image,(node_list[id].x,node_list[id].y),(test_nodex,test_nodey),(0,0,255),1)
                
# Display resulting image
plt.imshow(prm_image)
plt.show()

## ======================================================= WORKING ON =========================================================================

# Task 4.2 (3 marks): Djistraks
# Example output - Path: [-1, 143, 43, 1, 14, 44, 67, 7, 9, 4, 12, 364, -2], Cost: 561.9189671797234
def dijkstra(graph, start_id, end_id):

    path = []
    total_cost = 0
    node_list = graph.nodes
    edge_list = graph.edges
    # shortest_path[next_node]=(current_node,weight)
    shortest_path = {start_id:(None,0)}
    current = start_id
    visited = []
    print("INITIALIZING || ",start_id)
    while not (current == end_id):
        print("Current node:",current)
        visited.append(current)
        potential_edges = edge_list[current]
        current_weight = shortest_path[current][1]
        #print("Current weight:",current_weight)
        #print("Potential_edges:",potential_edges)
        for next_node in potential_edges:
            #print("New weight to be added:",current_weight)
            new_weight = edge_list[current][next_node]+current_weight
            #print("NEW WEIGHT || ",new_weight)
            if next_node not in shortest_path:
                shortest_path[next_node] = (current,new_weight)
                #print("SHORTEST PATH ||",shortest_path)
            else:
                new_shortest_weight = shortest_path[next_node][1]
                if new_shortest_weight > new_weight:
                    shortest_path[next_node] = (current, new_weight)
        # set next node based on smallest weight
        sorted_edges = dict(sorted(edge_list[current].items(), key=lambda item: item[1]))
        next_to_explore = {node:sorted_edges[node] for node in sorted_edges if node not in visited}
        if not next_to_explore:
            print("IMPOSSIBLE")
        else:
            print("Next to explore:",next_to_explore)
            print("New current",current)
            current = list(next_to_explore)[0]

        
    
    # Go back to the start from end
    while current is not None:
        path.append(current)
        next_node = shortest_path[current][0]
        current = next_node
    path = path[::-1]
    return (path, total_cost)

dijkstra(prm_graph,0,99)

## ====================== CHATGPT DIJKSTRAS ===================================
import heapq

def dijkstra(graph, start_id, end_id):
    path = []
    node_list = graph.nodes
    edge_list = graph.edges

    # shortest_path[node] = (previous_node, cost)
    shortest_path = {start_id: (None, 0)}
    priority_queue = [(0, start_id)]
    visited = set()

    while priority_queue:
        current_weight, current_node = heapq.heappop(priority_queue)

        if current_node in visited:
            continue

        visited.add(current_node)

        if current_node == end_id:
            break

        for next_node, weight in edge_list[current_node].items():
            if next_node in visited:
                continue

            new_weight = current_weight + weight

            if next_node not in shortest_path or new_weight < shortest_path[next_node][1]:
                shortest_path[next_node] = (current_node, new_weight)
                heapq.heappush(priority_queue, (new_weight, next_node))

    # Reconstruct the path from end_id to start_id
    current_node = end_id
    total_cost = shortest_path[end_id][1]
    while current_node is not None:
        path.append(current_node)
        current_node = shortest_path[current_node][0]

    path.reverse()

    return path, total_cost

# Example usage:
# Assuming prm_graph is defined and has nodes and edges attributes.
dijkstra(prm_graph, 0, 99)
