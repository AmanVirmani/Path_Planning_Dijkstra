import numpy as np
import cv2
import math
import time
import obstacle_map
from queue import PriorityQueue

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

def isValidNode(map_,x,y,r):
    rows,cols = map_.shape[:2]
    if 0 <= x-r and x+r < rows and 0 <= y-r and y+r < cols:
        if not detectCollision(map_, (x, y), r):
            return True
        else :
            return False
    else:
        return False

def detectCollision(img, center,radius):
    for i in range(2*radius+1):
        for j in range(2*radius+1):
            if i**2+j**2 <= radius**2:
                if not ((img[center[0]+i][center[1]+j]==(255,255,255)).all() and (img[center[0]+i][center[1]-j]==(255,255,255)).all()\
                        and (img[center[0]-i][center[1]-j]==(255,255,255)).all() and (img[center[0]-i][center[1]+j]==(255,255,255)).all()):
                    return True
    return False

def getRC():
   r = int(input('Enter radius of the robot: '))
   c = int(input('Enter desired clearance: '))
   return r,c


def getStartNode(map_,r):
    print("Enter the start co-ordinates")
    rows, cols= map_.shape[:2]
    while True :
        ## Cartesian Form
        x = int(input("x_intial is: "))
        y = int(input("y_intial is: "))
        ## image coordinates
        row = rows-y-1 ; col = x
        if not isValidNode(map_, row, col,r):
            print('Input Node not within available map range. Please enter again!')
        else:
            break;
    return (row, col)

def getGoalNode(map_,r):
    print("Enter the goal co-ordinates")
    rows, cols= map_.shape[:2]
    while True:
        ## Cartesian Form
        x = int(input("x_goal is: "))
        y = int(input("y_goal is: "))
        ## image coordinates
        row = rows-y-1 ; col = x
        if not isValidNode(map_, row, col, r):
            print('Input Node not within available map range. Please enter again! ')
        else:
            break;
    return (row, col)

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

def updateNeighbours(arr, map_, curr_node,queue,r):
    x,y = curr_node
    ## top node
    if isValidNode(map_,x-1, y, r):
        if (arr[x][y].cost + 1 < arr[x-1][y].cost):
            arr[x-1][y].cost = arr[x][y].cost + 1
            arr[x-1][y].parent = curr_node
            queue.put((arr[x-1][y].cost,(x-1,y)))

    ## top-left node
    if isValidNode(map_,x-1,y-1,r):
        if (arr[x][y].cost + 1.41 < arr[x-1][y-1].cost):
            arr[x-1][y-1].cost = arr[x][y].cost + 1.41
            arr[x-1][y-1].parent = curr_node
            queue.put((arr[x-1][y-1].cost,(x-1,y-1)))

    ## left node
    if isValidNode(map_,x,y-1,r):
        if (arr[x][y].cost + 1 < arr[x][y-1].cost):
            arr[x][y-1].cost = arr[x][y].cost + 1
            arr[x][y-1].parent = curr_node
            queue.put((arr[x][y-1].cost,(x,y-1)))

    ## bottom-left node
    if isValidNode(map_,x+1,y-1,r):
        if (arr[x][y].cost + 1.41 < arr[x+1][y-1].cost):
            arr[x+1][y-1].cost = arr[x][y].cost + 1.41
            arr[x+1][y-1].parent = curr_node
            queue.put((arr[x+1][y-1].cost,(x+1,y-1)))

    ## bottom_node
    if isValidNode(map_,x+1,y,r):
        if (arr[x][y].cost + 1 < arr[x+1][y].cost):
            arr[x+1][y].cost = arr[x][y].cost + 1
            arr[x+1][y].parent = curr_node
            queue.put((arr[x+1][y].cost,(x+1,y)))

    ## bottom-right node
    if isValidNode(map_,x+1,y+1,r):
        if (arr[x][y].cost + 1.41 < arr[x+1][y+1].cost):
            arr[x+1][y+1].cost = arr[x][y].cost + 1.41
            arr[x+1][y+1].parent = curr_node
            queue.put((arr[x+1][y+1].cost,(x+1,y+1)))

    ## right node
    if isValidNode(map_,x,y+1,r):
        if (arr[x][y].cost + 1 < arr[x][y+1].cost):
            arr[x][y+1].cost = arr[x][y].cost + 1
            arr[x][y+1].parent = curr_node
            queue.put((arr[x][y+1].cost,(x,y+1)))

    ## top-right node
    if isValidNode(map_,x-1,y+1,r):
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

def tracePath(arr,map_,goal_node,r):
    images= []
    output = './output_rigid.avi'
    curr_node = goal_node
    img = map_.copy()
    while curr_node is not None:
        cv2.circle(img, (curr_node[1],curr_node[0]), r, (0,0,255), -1)
        images.append(img)
        cv2.circle(img, (curr_node[1],curr_node[0]), r, (0,0,0), -1)
        curr_node = arr[curr_node[0]][curr_node[1]].parent
    images = images[::-1]
    saveVideo(images,output)

def saveVideo(images,output='path.avi'):
    h,w = images[0].shape[:2]
    out = cv2.VideoWriter(output,cv2.VideoWriter_fourcc('M','J','P','G'), 1, (w,h))
    images= np.uint8(images)
    for img in images:
        cv2.imshow('path traced',img)
        cv2.waitKey(10)
        out.write(img)
    out.release()

def exploredPath(map_,arr):
    img = map_.copy()
    rows,cols = map_.shape[:2]
    for row in range(rows):
        for col in range(cols):
            if arr[row][col].visited:
                img[row][col]==(0,255,255)
    return img

def main():
    r,c = getRC()
    map_ = load_map()
    start_node = getStartNode(map_,r+c)
    goal_node = getGoalNode(map_,r+c)
    rows, cols = map_.shape[:2]
    arr = np.array([[Node() for j in range(cols)] for i in range(rows)])
    arr[start_node[0]][start_node[1]].visited = True
    arr[start_node[0]][start_node[1]].cost = 0
    queue = PriorityQueue()
    queue.put((arr[start_node[0]][start_node[1]].cost, start_node))
    start_time = time.time()
    img = map_.copy()
    img[goal_node[0]][goal_node[1]] = (0,0,255)
    while queue:
        curr_node = queue.get()[1]
        if (curr_node == goal_node):
            algo_time = time.time()
            print('found Goal in {}s at cost {}!!'.format(algo_time-start_time, arr[goal_node[0]][goal_node[1]].cost))
            tracePath(arr,map_,goal_node,r)
            break
        arr[curr_node[0]][curr_node[1]].visited = True
        arr = updateNeighbours(arr, map_, curr_node,queue,r+c)
        img[curr_node[0]][curr_node[1]] = (0,255,0)
        cv2.imshow('explored',img)
        cv2.waitKey(10)
        #exploredList.append(img)
    #saveVideo(exploredList,'explored.avi')
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
