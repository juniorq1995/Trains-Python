#https://www.bogotobogo.com/python/python_graph_data_structures.php
#https://www.bogotobogo.com/python/python_Dijkstras_Shortest_Path_Algorithm.php
# 1. The distance of the route A-B-C.
# 2. The distance of the route A-D.
# 3. The distance of the route A-D-C.
# 4. The distance of the route A-E-B-C-D.
# 5. The distance of the route A-E-D.
# 6. The number of trips starting at C and ending at C with a maximum of 3 stops.  In the sample data below, there are two such trips: C-D-C (2 stops). and C-E-B-C (3 stops).
# 7. The number of trips starting at A and ending at C with exactly 4 stops.  In the sample data below, there are three such trips: A to C (via B,C,D); A to C (via D,C,D); and A to C (via D,E,B).
# 8. The length of the shortest route (in terms of distance to travel) from A to C.
# 9. The length of the shortest route (in terms of distance to travel) from B to B.
# 10. The number of different routes from C to C with a distance of less than 30.  In the sample data, the trips are: CDC, CEBC, CEBCDC, CDCEBC, CDEBC, CEBCEBC, CEBCEBCEBC.

# Split A into two nodes, called START and GOAL.

# For any edge A->x add an edge START->x

# For any edge y->A add an edge y->GOAL

# Keep all other edges unchanged.

import sys
import heapq


class Vertex:
    def __init__(self, node):
        self.id = node
        # Connected nodes and their weights
        self.adjacentNodes = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacentNodes[neighbor] = weight

    def modify_neighbor(self,oldNeighbor,newNeighbor):
        self.adjacentNodes[newNeighbor] = self.adjacentNodes.pop(oldNeighbor)

    def get_connections(self):
        return self.adjacentNodes.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacentNodes[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacentNodes: ' + str([x.id for x in self.adjacentNodes])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0
    def __iter__(self):
        return iter(self.vert_dict.values())

    def reset(self):
        for v in self:
            v.set_distance(sys.maxint)
            v.set_previous(None)
            v.visited = False

    def node_split(self,node):
        # Create a new node that is a lower-case version
        miniMe = node.get_id().lower()
        self.add_vertex(miniMe)
        # For all vertices in graph
        for v in self:
            for w in v.get_connections():
                if(w.get_id() == node.get_id()):
                    v.modify_neighbor(w,self.get_vertex(miniMe))
        return

    def merge_nodes(self,original, copy):
        # Replace the copies in the edges with the original
        for v in self:
            for w in v.get_connections():
                if(w.get_id() == copy.get_id()):
                    v.modify_neighbor(w,original)
        # Reassign the distance to original node
        original.set_distance(copy.get_distance())
        # Remove the copy from the vertex list
        del self.vert_dict[copy.get_id()]
        return

    def add_vertex(self, node):
        if node not in self.vert_dict:
            self.num_vertices = self.num_vertices + 1
            new_vertex = Vertex(node)
            self.vert_dict[node] = new_vertex
            return new_vertex
        else:
            return
    
    def remove_vertex(self,node):
        if node not in self.vert_dict:
            return
        else:
            #self.get_vertex(node) = None
            self.vert_dict.pop(node)
            return
    
    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None
    
    def add_edge(self, frm, to, cost=0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        #self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def dijkstra_shortest_path(agraph, start, target):
    #print '''Dijkstra's shortest path'''
    # Set the distance for the start node to zero 
    start.set_distance(0)
    # If start and target are same nodes
    # Split the node into two
    needsMerging = False
    if(start.get_id() == target.get_id()):
        # Splits the node
        # creating a clone at the ends of edges
        agraph.node_split(start)
        target = agraph.get_vertex(target.get_id().lower())
        needsMerging = True
        

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in agraph]
    heapq.heapify(unvisited_queue)
    while len(unvisited_queue):
        # Pops a vertex with the smallest distance 
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        if(start.get_id() == target.get_id() and current.get_id == start.get_id()):
            unvisited_queue.append(current)
            pass
        else:
            current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacentNodes:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                #print 'updated : current = %s next = %s new_dist = %s' \
                        #%(current.get_id(), next.get_id(), next.get_distance())
            #else:
                #print 'not updated : current = %s next = %s new_dist = %s' \
                        #%(current.get_id(), next.get_id(), next.get_distance())
    # Now undo the split by merging the two nodes into one again
    if(needsMerging):
        agraph.merge_nodes(start,target)

def shortest_path(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest_path(v.previous, path)
    return 

def custom_path_distance(agraph,path):
    distance = 0
    containsNeighbor = False
    for x in range(0,len(path)-1):
        #reset checker
        containsNeighbor = False
        # Get next vertex from list
        vert = agraph.get_vertex(path[x])
        for w in vert.get_connections():
            # Get each edge
            # Check if edge is 
            if(w.get_id() == path[x+1]):
                distance = distance + vert.get_weight(w)
                containsNeighbor = True
                break
        if(not containsNeighbor):
            return 0
        
    return distance

def count_paths(agraph,start,target,constraint,mode):
    count = 0
    # Mode for Max Distance NotIncl
    if(mode == 0):
        count = count_paths_max_distance(start,target,constraint,0,0)
    # Mode for Exact Stops
    if(mode == 1):
        count = count_paths_exact_stops(start,target,constraint,0,0)
    # Mode for Max Stops Incl
    if(mode == 2):
        count = count_paths_max_stops(start,target,constraint,0,0)


    return count

def count_paths_max_distance(start,target,maxDist,currDist,pathCount):
      
    # // If current vertex is same as destination,  
    # // then increment count 
    if (start.get_id() == target.get_id() and currDist > 0): 
        pathCount = pathCount + 1
  
    for w in start.get_connections():
        if(currDist + start.get_weight(w) < maxDist):
            pathCount = count_paths_max_distance(w,target,maxDist,currDist + start.get_weight(w),pathCount)
    return pathCount

def count_paths_exact_stops(start,target,exactStops,currStops,pathCount):
      
    # // If current vertex is same as destination,  
    # // then increment count 
    if (start.get_id() == target.get_id() and currStops > 0 and currStops == exactStops): 
        pathCount = pathCount + 1
  
    for w in start.get_connections():
        if(currStops < exactStops):
            pathCount = count_paths_exact_stops(w,target,exactStops,currStops + 1,pathCount)
    return pathCount

def count_paths_max_stops(start,target,maxStops,currStops,pathCount):
      
    # // If current vertex is same as destination,  
    # // then increment count 
    if (start.get_id() == target.get_id() and currStops > 0): 
        pathCount = pathCount + 1
        
    #print 'here'
    for w in start.get_connections():
        if(currStops < maxStops):
            pathCount = count_paths_max_stops(w,target,maxStops,currStops + 1,pathCount)
    return pathCount

def import_graph(filename):
    f = open(filename)
    g = Graph()
    line = f.readline()
    edges = line.split(', ')
    
    for e in edges:
        info = list(e)
        g.add_vertex(info[0])
        g.add_vertex(info[1])
        g.add_edge(info[0],info[1],int(''.join(info[2:])))
    return g



if __name__ == '__main__':
    #filename = raw_input()
    g = import_graph("input.txt")
    # print 'Graph data:'
    
    # print 'Output #1: %d' %(custom_path_distance(g,['A','B','C'])) # Should be 9
    # print 'Output #2: %d' %(custom_path_distance(g,['A','D'])) # Should be 5
    # print 'Output #3: %d' %(custom_path_distance(g,['A','D','C'])) # Should be 13
    # print 'Output #4: %d' %(custom_path_distance(g,['A','E','B','C','D'])) # Should be 22
    # print 'Output #5: %d' %(custom_path_distance(g,['A','E','D'])) # Should be NO SUCH ROUTE
    # print 'Output #6: %d' %(count_paths(g,g.get_vertex('C'),g.get_vertex('C'),3,2)) # Should be 2
    # print 'Output #7: %d' %(count_paths(g,g.get_vertex('A'),g.get_vertex('C'),4,1)) # Should be 3

    dijkstra_shortest_path(g,g.get_vertex('A'), g.get_vertex('C'))
    print 'Output #8: %s' %(g.get_vertex('C').get_distance()) # Should be 9

    # g.reset()
    dijkstra_shortest_path(g,g.get_vertex('B'), g.get_vertex('B'))
    print 'Output #9: %s' %(g.get_vertex('B').get_distance()) # Should be 9
    # print 'Output #10: %d' %(count_paths(g,g.get_vertex('C'),g.get_vertex('C'),30,0)) # Should be 7
