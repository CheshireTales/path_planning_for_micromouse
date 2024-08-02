# Task 4.1 (3 marks): PRM
# NOTE: The iteration only increases when a valid node is placed on the map. If the node is invalid, the program should continue without incrementing the iteration count. 
print("START PRM -------------------")
prm_graph = Graph()
prm_image = img_dangerzone.copy()
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
        i += 1
        
stragglers = []
print("DOING STRAGGLERS NOW")
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
print("SUCCESSFULLY FINISHED PRM")
    
# Display resulting image

plt.imshow(prm_image)
plt.show()
