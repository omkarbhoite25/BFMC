import networkx as nx
import numpy as np
from scipy.spatial import distance
import math
from time import sleep
class PathPlanning:

    map_path = "/home/alberto/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src/map.graphml"
    intersection_nodes = ['346', '270', '32', '36', '34', '6', '25', '27', '23', '370', '81', '2', '15', '16', '77', '72', '70', '79', '311', '314', '61', '467', '4', '59', '54', '52', '63', '68', '423', '303', '305', '45', '43', '41']
    central_nodes = ['347', '271', '37', '39', '38', '11', '29', '30', '28', '371', '84', '9', '20', '20', '82', '75', '74', '83', '312', '315', '65', '468', '10', '64', '57', '56', '66', '73', '424', '304', '306', '48', '47', '46']
    turning_left = False
    turn_angle = 0
    G = nx.read_graphml(map_path)

    @staticmethod
    def shortest_path(source, target):
        return list(nx.all_shortest_paths(PathPlanning.G, source=source, target=target))[0]

    @staticmethod
    def find_closest_node(x, y):
        distances_dict = {"Nodes": [], "Distances":[]}
        for node in PathPlanning.G.nodes():
            node_x = PathPlanning.G.nodes[node]['x']
            node_y = PathPlanning.G.nodes[node]['y']

            distance_from_node = distance.euclidean((x,y), (node_x,node_y))
            distances_dict["Nodes"].append(node)
            distances_dict["Distances"].append(distance_from_node)

        
        min_distance = min(distances_dict["Distances"])
        min_index  = distances_dict["Distances"].index(min_distance)
        min_node = distances_dict["Nodes"][min_index]
  
        return min_node

    @staticmethod
    def remove_central_nodes(path):
        new_path = path[:]
        for node in PathPlanning.central_nodes:
            if node in new_path:
                new_path.remove(node)
        return new_path

    @staticmethod
    def find_inter_nodes():
        inter_nodes = []
        for node in PathPlanning.G.nodes:
            neighbors_list = list(PathPlanning.G.neighbors(node))
            if len(neighbors_list) > 0 and len(list(PathPlanning.G.neighbors(neighbors_list[0]))) > 1:
                inter_nodes.append(node)
        print(inter_nodes)

    
    @staticmethod
    def find_central_nodes():
        inter_nodes = []
        for node in PathPlanning.G.nodes:
            neighbors_list = list(PathPlanning.G.neighbors(node))
            if len(neighbors_list) > 0 and len(list(PathPlanning.G.neighbors(neighbors_list[0]))) > 1:
                inter_nodes.append(neighbors_list[0])
        print(inter_nodes)

    '''
    @staticmethod
    def check_intersection(x, y, path):
        intersection = False

        closest_node = PathPlanning.find_closest_node(x,y)
        target_node = 0
        if(closest_node in PathPlanning.intersection_nodes):
            intersection = True
            in_path = True
            try:
                index = path.index(closest_node) + 1
            except:
                in_path = False

            if(in_path and path > 1):
                target_node = path[index]
            else:
                target_node = None
        print(intersection)
        return intersection, target_node
    '''

    @staticmethod
    def check_intersection(path):
        for i in range(2):
            if path[i] in PathPlanning.intersection_nodes:
                target_node = path[i+1] #if path[i+1] not in PathPlanning.central_nodes else path[i+2]
                #print("Target NODE: ", target_node)
                #sleep(3)
                return True, target_node
        #if path[0] or path[1] in PathPlanning.intersection_nodes:
        #    return True, path[1]
        return False, None

    @staticmethod
    def find_target(path):
        for i in range(2):
            if path[i] in PathPlanning.intersection_nodes:
                target_node = path[i+1] #if path[i+1] not in PathPlanning.central_nodes else path[i+2]
                #print("Target NODE: ", target_node)
                #sleep(3)
                return target_node
        #if path[0] or path[1] in PathPlanning.intersection_nodes:
        #    return True, path[1]
        return False, None
    @staticmethod
    def update_path(path, x, y, finish):
        closest_node = PathPlanning.find_closest_node(x,y)
        print("CLOSEST NODE: " + str(closest_node))

        if closest_node == finish:
            return path, True
        if(closest_node in path):
            path = path[path.index(closest_node)+1:]
            print
            #path.remove(closest_node)

        return path, False


    @staticmethod
    def intersection_navigation(path, x, y, target_node, start_yaw, yaw, complete_path):
        #Ignore next node as it is an central intersection node
        
        reached_target = False
        steering_angle = 0
        if(len(path) > 0):
            next_node = path[0]
        else:
            return steering_angle, True

        if(next_node != target_node):
            reached_target = True
            return steering_angle, reached_target

        closest_node = PathPlanning.find_closest_node(x,y)

        cd_x = PathPlanning.G.nodes[closest_node]['x']
        cd_y = PathPlanning.G.nodes[closest_node]['y']
         

        node_x = PathPlanning.G.nodes[next_node]['x']
        node_y = PathPlanning.G.nodes[next_node]['y']

        print("Node: ", next_node)
        print("Node_x: " + str(node_x) + " X: " + str(x))
        print("Node_y: " + str(node_y) + " Y: " + str(y))

        #if( )

        #Upwards
        if( 80 <= start_yaw <= 100 ):#(node_x > cd_x and node_y < cd_y) or (node_x == cd_x and node_y < cd_y) or (node_x < cd_x and node_y < cd_y)):
            central_node = complete_path[complete_path.index(next_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']
            

            if((node_x < cd_x and node_y < cd_y) and not PathPlanning.in_range(central_node_x, central_node_y, x, y)):
                print("Left Turn")

                node_x = central_node_x
                node_y = central_node_y

                angle_rad = math.atan(float(node_x - x) / float(y - node_y))
                steering_angle = int(angle_rad * 180.0 / math.pi)

            elif (node_x > cd_x and node_y < cd_y):
                steering_angle = 15

            else:

                angle_rad = math.atan(float(node_x - x) / float(y - node_y))
                steering_angle = int(angle_rad * 180.0 / math.pi)
            
            print("Upwards")
        
        #Left-to-right
        elif(-10 <= start_yaw <= 10): #(node_x > cd_x and node_y > cd_y) or (node_x > cd_x and node_y == cd_y) or (node_x > cd_x and node_y < cd_y)):
            central_node = complete_path[complete_path.index(next_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']
            
            if(node_x > cd_x and node_y < cd_y and not PathPlanning.in_range(central_node_x, central_node_y, x, y)):

                node_x = central_node_x
                node_y = central_node_y
                angle_rad = -math.atan(float(y - node_y) / float(node_x - x))
                steering_angle = int(angle_rad * 180.0 / math.pi)

            elif((node_x > cd_x and node_y > cd_y)):
                steering_angle = 15

            else:
                angle_rad = -math.atan(float(y - node_y) / float(node_x - x))
                steering_angle = int(angle_rad * 180.0 / math.pi)
                
            print("Left-to-right ", steering_angle)
        #Downwards
        elif(-100 <= start_yaw <= -80):#(node_x < cd_x and node_y > cd_y) or (node_x == cd_x and node_y > cd_y) or (node_x > cd_x and node_y > cd_y)):
            central_node = complete_path[complete_path.index(next_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']
            
            if((node_x > cd_x and node_y > cd_y) and not PathPlanning.in_range(central_node_x, central_node_y, x, y)):
                print("Left Turn")
                node_x = central_node_x
                node_y = central_node_y
                angle_rad = -math.atan(float(node_x - x) / float(y - node_y))
                steering_angle = int(angle_rad * 180.0 / math.pi)
            elif((node_x < cd_x and node_y > cd_y)):
                steering_angle = 15
            else:
                angle_rad = -math.atan(float(node_x - x) / float(y - node_y))
                steering_angle = int(angle_rad * 180.0 / math.pi)

            print("Downwards")
        #Right-to-left
        elif(-180 <= start_yaw <= -170 or 170 <= start_yaw <= 180):#(node_x < cd_x and node_y < cd_y) or (node_x < cd_x and node_y == cd_y) or (node_x < cd_x and node_y > cd_y)):
            central_node = complete_path[complete_path.index(next_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']

            if((node_x < cd_x and node_y > cd_y)):
                print("Left Turn")
                node_x = central_node_x
                node_y = central_node_y

                angle_rad = math.atan(float(y - node_y) / float(node_x - x))
                steering_angle = int(angle_rad * 180.0 / math.pi)
            elif(node_x < cd_x and node_y < cd_y):
                steering_angle = 15
            else:

                angle_rad = math.atan(float(y - node_y) / float(node_x - x))
                steering_angle = int(angle_rad * 180.0 / math.pi)

            print("Right-to-left")
        
        print("Target is: " + str(target_node))
        return steering_angle, reached_target

    

   
    @staticmethod
    def distance_from_imaginary_node(central_node_x, central_node_y, target_node_x, target_node_y, x, y):
        im_node = [0,0]
        im_node[0] = (central_node_x + target_node_x) / 2.0
        im_node[1] = (central_node_y + target_node_y) / 2.0

        distance_from_im = distance.euclidean((x,y), (im_node[0],im_node[1]))

        return distance_from_im

    @staticmethod
    def in_range(node_x,node_y, x, y):
        threshold = 0.1
        if( node_x - threshold <= x < node_x + threshold and node_y - threshold <= y <= node_y + threshold):
            return True
        else:
            return False

    @staticmethod
    def in_vertical_range(node_y, y):
        threshold = 0.2
        if(node_y - threshold <= y <= node_y + threshold):
            return True
        else:
            False