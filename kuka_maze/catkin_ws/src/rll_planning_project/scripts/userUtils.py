import pdb
import copy
import math
import numpy as np

def distance(pt1, pt2, ang1=0.0, ang2=0.0, unitDist=0.0):
    angleNormalization = unitDist * ( math.pi / (2*math.pi) )
    dist = ( (pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2 + ( angleNormalization*(ang1-ang2) )**2 )**0.5
    # OR drop theta in square root and just add that part individually: dist += abs( angleNormalization*(ang1-ang2) )
    return dist

# Binary Node
class Node:

    def __init__(self, value=(None, None), theta=None, depth=None):
        self.val = value
        self.theta = theta
        self.depth = depth
        self.childLeft = None
        self.childRight = None
        self.parentInPath = None

class KDTree:

    # Python does not have constructor overloading. Just need to 
    #set the default value to the args that it accepts
    # def __init__(self):
    #     self.tree = Node() # Create root node
    def __init__(self, value=(None, None), theta=None, doubleTree=False):
        self.doubleTree = doubleTree
        # Create root node
        if(not self.doubleTree):
            self.root = Node(value=value, theta=theta, depth=0)
        else:
            self.xroot = Node(value=value, theta=theta, depth=0)
            self.yroot = Node(value=value, theta=theta, depth=0)


    '''
    Insert a node into xtree or ytree
    '''
    def insertHelper(self, nodeIns, rootAxis='x'):
        # print("Inserting a node: ", nodeIns.val, ", ", nodeIns.theta)
        if(not self.doubleTree):
            node = self.root
            dim = 0
        else:
            if(rootAxis == 'x'):
                node = self.xroot
                dim = 0
            elif(rootAxis == 'y'):
                node = self.yroot
                dim = 1
            else:
                print("Invalid dimension.")

        # 'depth' is used to simply set the depth of the node (for distance calculation. 
        # 'dim' is what is used to check the x or y axis
        depth = 0
        while(1):

            nodeIns.depth = depth+1
            dim = dim % 2

            if(nodeIns.val[dim] <= node.val[dim]):
                if(node.childLeft is None): # Create a new node here
                    node.childLeft = nodeIns
                    break # done inserting
                else: # continue on with the search of empty node
                    node = node.childLeft

            else:
                if(node.childRight is None):
                    node.childRight = nodeIns
                    break
                else:
                    node = node.childRight

            depth += 1
            dim += 1

    '''
    Insert a node into kdtree
    '''
    def insert(self, nodeIns):
        # Perform a deepcopy while inserting nodes so that cross referencing between nodes does not happen, 
        # especially in the case of dualTree
        if(not self.doubleTree):
            self.insertHelper(copy.deepcopy(nodeIns), rootAxis='x')
        else:
            self.insertHelper(copy.deepcopy(nodeIns), rootAxis='x')
            self.insertHelper(copy.deepcopy(nodeIns), rootAxis='y')


    '''
    Search for a point that is within distThresh. 

    If not found within given thresh, just return latest node explored, 
    as that would be closest point to given point.

    Same implementation as insert, except that in the end the latest searched node is returned
    and a flag is sent back. (if the node returned is within the given threshold or is just the closest point)
    '''
    def searchHelper(self, nodeSearch, distThresh, getPathKD=False, rootAxis='x'):

        ## Will there be duplicate value entries?? But with different theta values??
        ## Change distance metric to account for this if so
        if(not self.doubleTree):
            node = self.root
            dim = 0
        else:
            if(rootAxis == 'x'):
                node = self.xroot
                dim = 0
            elif(rootAxis == 'y'):
                node = self.yroot
                dim = 1
            else:
                print("Invalid dimension.")
        nodes = []
        dists = []
        distFlag = False
        if(getPathKD):
            path = []
        else:
            path = None

        depth = 0
        # Search till None node is reached. Once None is reached, return latest node value
        while(1):
            nodeSearch.depth = depth
            dim = dim % 2
            
            if(getPathKD):
                path.append(node)

            dist = distance(node.val, nodeSearch.val)
            # dist = distance(node.val, nodeSearch.val, node.theta, nodeSearch.theta, unitDist=0.01)

            if(dist <= distThresh):
                # Don't just directly concider the first point to be the closest/valid point. 
                # Multiple may satisfy distance requirement. So add all that satisfy to a list
                # Pick best from list at the end.
                nodes.append(node)
                dists.append(dist)
                distFlag = True

            if(nodeSearch.val[dim] <= node.val[dim]):
                if(node.childLeft is None): # Just return the latest node
                    nodes.append(node)
                    break
                else: # continue on with the search of empty node
                    node = node.childLeft
            else:
                if(node.childRight is None):
                    nodes.append(node)
                    break
                else:
                    node = node.childRight

            depth += 1
            dim += 1
        
        ''' 
        Find closest node among shortlisted nodes
        '''
        def findClosest(nodes, dists):
            minLoc = np.argmin(dists)
            return nodes[minLoc]

        if(len(nodes) == 1):
            node = nodes[0]
        else:
            # If multiple entries exist, means atleast one optimal pt exist. So exclude the 
            # last added sub-optimal point due to reaching the None branch. This way nodes
            # and dists will be of equal length
            nodes = nodes[0:-1]
            node = findClosest(nodes, dists)

        return (node, distFlag, path)

    def search(self, nodeSearch, distThresh, getPathKD=False, rootAxis='x'):
        if(not self.doubleTree):
            node, distFlag, path = self.searchHelper(nodeSearch, distThresh, getPathKD, rootAxis='x')
            treeType = 'x'
        else:
            nodeX, distFlagX, pathX = self.searchHelper(nodeSearch, distThresh, getPathKD, rootAxis='x')
            nodeY, distFlagY, pathY = self.searchHelper(nodeSearch, distThresh, getPathKD, rootAxis='y')

            # send out min of both of these trees
            if(distance(nodeSearch.val, nodeX.val) <= distance(nodeSearch.val, nodeY.val)):
            # if(distance(nodeSearch.val, nodeX.val, nodeSearch.theta, nodeX.theta, unitDist=0.01) 
            #     <= distance(nodeSearch.val, nodeY.val, nodeSearch.theta, nodeY.theta, unitDist=0.01)):
                node = nodeX
                distFlag = distFlagX
                path = pathX
                treeType = 'x'
            else:
                node = nodeY
                distFlag = distFlagY
                path = pathY
                treeType = 'y'
        
        return node, distFlag, path, treeType

            
        
