import numpy as np
import cv2
import math
import obstacle_map
import time
from queue import PriorityQueue
import argparse
import imageio

def load_map(fname = None):
    if fname is not None :
        map_ = cv2.imread(fname)
        return map_
    world= 255*np.ones((200,300,3))
    rc=0
    obstacle_map.obstacle_circle(world)
    obstacle_map.obstacle_ellipse(world)
    obstacle_map.obstacle_rhombus(world)
    obstacle_map.obstacle_rectangle(world)
    obstacle_map.obstacle_polygon(world)

    cv2.imwrite('./map.jpg',world)
    return world


def isValidNode(map_,x,y):
    rows,cols = map_.shape[:2]
    if 0 <= x < rows and 0 <= y < cols and (map_[x][y]==(255,255,255)).all():
        return True
    else :
        return False


def getStartNode(map_, loc):
    rows, cols = map_.shape[:2]
    row = rows-loc[1]-1; col = loc[0]
    if not isValidNode(map_, row, col):
        print('Start Node not within available map range. Please enter again!')
        return None
    return row, col


def getGoalNode(map_, loc):
    rows, cols = map_.shape[:2]
    row = rows-loc[1]-1; col = loc[0]
    if not isValidNode(map_, row, col):
        print('Goal Node not within available map range. Please enter again! ')
        return None
    return row, col

# A node structure for our search tree
class Node:
    # A utility function to create a new node
    # visited : flag to identify visited nodes
    # parent : coordinate location of the parent
    # cost : cost of reaching current node from start node
    def __init__(self, visited=False,parent=None, cost = math.inf ):
        self.visited = visited
        self.parent = parent
        self.cost = cost

def updateNeighbours(arr, map_, curr_node,queue):
    x,y = curr_node
    ## top node
    if isValidNode(map_,x-1,y):
        if (arr[x][y].cost + 1 < arr[x-1][y].cost):
            arr[x-1][y].cost = arr[x][y].cost + 1
            arr[x-1][y].parent = curr_node
            queue.put((arr[x-1][y].cost,(x-1,y)))

    ## top-left node
    if isValidNode(map_,x-1,y-1):
        if (arr[x][y].cost + 1.41 < arr[x-1][y-1].cost):
            arr[x-1][y-1].cost = arr[x][y].cost + 1.41
            arr[x-1][y-1].parent = curr_node
            queue.put((arr[x-1][y-1].cost,(x-1,y-1)))

    ## left node
    if isValidNode(map_,x,y-1):
        if (arr[x][y].cost + 1 < arr[x][y-1].cost):
            arr[x][y-1].cost = arr[x][y].cost + 1
            arr[x][y-1].parent = curr_node
            queue.put((arr[x][y-1].cost,(x,y-1)))

    ## bottom-left node
    if isValidNode(map_,x+1,y-1):
        if (arr[x][y].cost + 1.41 < arr[x+1][y-1].cost):
            arr[x+1][y-1].cost = arr[x][y].cost + 1.41
            arr[x+1][y-1].parent = curr_node
            queue.put((arr[x+1][y-1].cost,(x+1,y-1)))

    ## bottom_node
    if isValidNode(map_,x+1,y):
        if (arr[x][y].cost + 1 < arr[x+1][y].cost):
            arr[x+1][y].cost = arr[x][y].cost + 1
            arr[x+1][y].parent = curr_node
            queue.put((arr[x+1][y].cost,(x+1,y)))

    ## bottom-right node
    if isValidNode(map_,x+1,y+1):
        if (arr[x][y].cost + 1.41 < arr[x+1][y+1].cost):
            arr[x+1][y+1].cost = arr[x][y].cost + 1.41
            arr[x+1][y+1].parent = curr_node
            queue.put((arr[x+1][y+1].cost,(x+1,y+1)))

    ## right node
    if isValidNode(map_,x,y+1):
        if (arr[x][y].cost + 1 < arr[x][y+1].cost):
            arr[x][y+1].cost = arr[x][y].cost + 1
            arr[x][y+1].parent = curr_node
            queue.put((arr[x][y+1].cost,(x,y+1)))

    ## top-right node
    if isValidNode(map_,x-1,y+1):
        if (arr[x][y].cost + 1.41 < arr[x-1][y+1].cost):
            arr[x-1][y+1].cost = arr[x][y].cost + 1.41
            arr[x-1][y+1].parent = curr_node
            queue.put((arr[x-1][y+1].cost,(x-1,y+1)))

    return arr

def findMinCost(arr):
    curr_node = (-1,-1)
    min_cost = math.inf
    for row in range(len(arr)):
        for col in range(len(arr[row])):
            if arr[row][col].cost < min_cost and not arr[row][col].visited:
                curr_node = (row,col)
                min_cost = arr[row][col].cost
    return curr_node


def tracePath(arr, img, goal_node):
    nodes = []
    images = []
    output = './output.gif'
    curr_node = goal_node
    #img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    while curr_node is not None:
        nodes.append(curr_node)
        curr_node = arr[curr_node[0]][curr_node[1]].parent
    nodes = nodes[::-1]
    for curr_node in nodes:
        #img_curr = cv2.circle(img, (curr_node[1], curr_node[0]), 3, (255, 0, 0), -1)
        img[curr_node[0]][curr_node[1]] = (0, 0, 255)
        images.append(np.uint8(img.copy()))
        img[curr_node[0]][curr_node[1]] = (0, 0, 0)

    imageio.mimsave(output, images,fps=55)
    #saveVideo(images,output)

def saveVideo(images,output='path.gif'):
    h,w = images[0].shape[:2]
    imageio.mimsave(output, images)
    #out = cv2.VideoWriter(output,cv2.VideoWriter_fourcc('M','J','P','G'), 1, (w,h))
    #for img in images:
    #    img = np.uint8(img)
    #    #cv2.imshow('path traced', img)
    #    #cv2.waitKey(10)
    #    out.write(img)
    #out.release()

def exploredPath(map_,arr):
    img = map_.copy()
    rows,cols = map_.shape[:2]
    for row in range(rows):
        for col in range(cols):
            if arr[row][col].visited:
                img[row][col] == (0, 255, 255)
    return img

def main(args):
    map_ = load_map()
    rows, cols = map_.shape[:2]
    start_node = getStartNode(map_, args['start_node'])
    goal_node = getStartNode(map_, args['goal_node'])
    if not (start_node and goal_node):
        exit(1)
    arr = np.array([[Node() for j in range(cols)] for i in range(rows)])
    arr[start_node[0]][start_node[1]].visited = True
    arr[start_node[0]][start_node[1]].cost = 0
    queue = PriorityQueue()
    queue.put((arr[start_node[0]][start_node[1]].cost, start_node))
    start_time = time.time()
    exploredList= []
    img = map_.copy()
    img[goal_node[0]][goal_node[1]] = (0,0,255)
    while queue:
        curr_node = queue.get()[1]
        if curr_node == goal_node:
            algo_time = time.time()
            print('found Goal in {}s at cost {}!!'.format(algo_time-start_time, arr[goal_node[0]][goal_node[1]].cost))
            tracePath(arr, img, goal_node)
            break
        arr[curr_node[0]][curr_node[1]].visited = True
        arr = updateNeighbours(arr, map_, curr_node, queue)
        img[curr_node[0]][curr_node[1]] = (0, 255, 0)
        exploredList.append(np.uint8(img.copy()))
        #cv2.imshow('explored', img)
        #cv2.waitKey(1)
    #saveVideo(exploredList,'explored.gif')
    imageio.mimsave('explored.gif', exploredList, fps=55)
    cv2.destroyAllWindows()


if __name__=='__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--radius", required=False, help="Radius of the robot",
                    default=1, type=int)
    ap.add_argument("-c", "--clearance", required=False, help="desired clearance to avoid obstacles",
                    default=1, type=int)
    ap.add_argument("-s", "--start_node", required=True, help="Location (x,y) of start node",
                    nargs='+', type=int)
    ap.add_argument("-g", "--goal_node", required=True, help="Location (x,y) of goal node",
                    nargs='+', type=int)
    args = vars(ap.parse_args())
    #print(args)
    main(args)
