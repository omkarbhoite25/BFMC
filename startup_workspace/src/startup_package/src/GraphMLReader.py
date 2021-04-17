import os
import networkx as netx
from networkx.algorithms.shortest_paths import astar


class GraphMLReader():
    '''
    For reading the GraphML files and computing paths
    '''    
    
    def __init__(self, filepath):
        if not (filepath and os.path.exists(filepath)):
            raise Exception("GraphML map file not found") 
        
        print("Loading map ...")
        self.map = netx.readwrite.graphml.read_graphml(filepath) 
        self.filepath = filepath
        print("GraphML File loaded successfully") 

    def get_node_coordinates(self, node):
        '''
        Get node coordinates 
        '''
        
        if isinstance(node, int): node = str(node)
        return (self.map.nodes[node]['x'], self.map.nodes[node]['y'])

    def get_path_from_src_to_dest(self, src, dst):
        '''
        Compute path for the given source to destination
        The function uses AStar and without any heuristics
        '''
        # :TODO Add heuristics here

        if isinstance(src, int): src = str(src)
        if isinstance(dst, int): dst = str(dst)

        print("Computing shortest path ...")
        path = astar.astar_path(self.map, src , dst)
        print("Shortest path computed \nPath: %s \nPath Length: %s" % (str(path), len(path)))

        return path
