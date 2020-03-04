import numpy as np
import cv2
import math

def load_map(fname = None):
    if fname is not None :
        map = cv2.imread(fname)
        return map

    world= 255*np.ones((200, 300, 3), np.uint8)
    cv2.circle(world, (225,50), 25, (0, 0, 0), -1)      #actual obstacle space
    cv2.ellipse(world, (150,100), (40,20), 0, 0, 360, 0, -1)
    p1= [25, 15]
    p2= [75, 15]
    p3= [100, 50]
    p4= [75, 80]
    p5= [50, 50]
    p6= [20, 80]
    poly= np.array([p1,p2,p3,p4,p5,p6],np.int32)
    cv2.fillConvexPoly(world, poly, 255)
    cv2.imwrite('./map.jpg',world)
    return world

def isValidNode(x,y):
    if 0 <= x < 200 and 0<= y < 300 :
        return True
    else :
        return False

def getStartNode():
    print("Enter the start co-ordinates")
    x,y = -1,-1
    while True :
        x = int(input("x_intial is: "))
        y = int(input("y_intial is: "))
        if not isValidNode(x,y) :
            print('Input Node not within map range. Please enter again! (0,300) ')
        else:
            break;
    return (x, y)

def getGoalNode():
    print("Enter the goal co-ordinates")
    x,y = -1,-1
    while True:
        x = int(input("x_goal is: "))
        y = int(input("y_goal is: "))
        if not isValidNode(x,y) :
            print('Input Node not within map range. Please enter again! (0,300) ')
        else:
            break;
    return (x, y)

# A node structure for our search tree
class Node:
    # A utility function to create a new node
    # state : 3X3 array to reperesent puzzle board
    # index : to identify node
    # parent : pointer to parent node
    # left : pointer to left child
    # right : pointer to right child
    # up : pointer to up child
    # down : pointer to down child

    def __init__(self, visited=False,parent=None, cost = math.inf ):
        self.visited = visited
        self.parent = parent
        self.cost = cost

def updateNeighbours(arr, map_, curr_node):
    x,y = curr_node
    ## top node
    if isValidNode(x-1,y) and map_[x-1,y] == (255,255,255):
        if (arr[x][y].cost + 1 < arr[x-1][y].cost ):
            arr[x-1][y].cost = arr[x][y].cost + 1
            arr[x-1][y].parent = curr_node

    ## top-left node
    if isValidNode(x-1,y-1) and map_[x-1,y] == (255,255,255):
        if (arr[x][y].cost + 1.41 < arr[x-1][y-1].cost ):
            arr[x-1][y-1].cost = arr[x][y].cost + 1.41
            arr[x-1][y-1].parent = curr_node

    ## left node
    if isValidNode(x,y-1) and map_[x-1,y] == (255,255,255):
        if (arr[x][y].cost + 1 < arr[x][y-1].cost ):
            arr[x][y-1].cost = arr[x][y].cost + 1
            arr[x][y-1].parent = curr_node

    ## bottom-left node
    if isValidNode(x+1,y-1) and map_[x-1,y] == (255,255,255):
        if (arr[x][y].cost + 1.41 < arr[x+1][y-1].cost ):
            arr[x+1][y-1].cost = arr[x][y].cost + 1.41
            arr[x+1][y-1].parent = curr_node

    ## bottom_node
    if isValidNode(x+1,y) and map_[x-1,y] == (255,255,255):
        if (arr[x][y].cost + 1 < arr[x+1][y].cost ):
            arr[x+1][y].cost = arr[x][y].cost + 1
            arr[x+1][y].parent = curr_node

    ## bottom-right node
    if isValidNode(x+1,y+1) and map_[x-1,y] == (255,255,255):
        if (arr[x][y].cost + 1.41 < arr[x+1][y+1].cost ):
            arr[x+1][y+1].cost = arr[x][y].cost + 1.41
            arr[x+1][y+1].parent = curr_node

    ## right node
    if isValidNode(x,y+1) and map_[x,y+1] == (255,255,255):
        if (arr[x][y].cost + 1 < arr[x][y+1].cost ):
            arr[x][y+1].cost = arr[x][y].cost + 1
            arr[x][y+1].parent = curr_node

    ## top-right node
    if isValidNode(x+1,y+1) and map_[x+1,y+1] == (255,255,255):
        if (arr[x][y].cost + 1.41 < arr[x][y+1].cost ):
            arr[x][y+1].cost = arr[x][y].cost + 1.41
            arr[x][y+1].parent = arr[x][y]

    return arr

def findMinCost(arr):
    curr_node = (0,0)
    min_cost = arr[0][0].cost
    for row in range(len(arr)):
        for col in range(len(arr[row])):
            if arr[row][col].cost < min_cost:
                curr_node = (row,col)
    return curr_node

def tracePath(arr,map_,goal_node):
    images= []
    curr_node = goal_node
    while curr_node is not None:
        img = map_
        img[curr_node[0]][curr_node[1]] = (0,0,255)
        images.append(img)

    images = images[::-1]
    saveVideo(images,output)

def saveVideo(images,output='path.avi'):
    h,w = images[0].shape[:2]
    out = cv2.VideoWriter(output,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (w,h))
    for img in images:
        out.write(img)
    out.release()


def findDijkstraGoal(arr, map_, goal_node):
    curr_node = findMinCost(arr)
    if curr_node == goal_node:
        print('reached Goal')
        tracePath(arr,map_,goal_node)
    else :
        arr[curr_node[0]][curr_node[1]].visited = True
        arr = updateNeighbours(arr, map_, curr_node)
        findDijkstraGoal(arr,map_,goal_node)

def main():
    map_ = load_map()
    start_node = getStartNode()
    goal_node = getGoalNode()
    rows, cols = map_.shape[:2]
    arr = [[Node() for j in range(cols)] for i in range(rows)]
    arr[start_node[0]][start_node[1]].visited = True
    arr[start_node[0]][start_node[1]].cost = 0
    findDijkstraGoal(arr,map_, goal_node)


if __name__=='__main__':
    main()
    #print('start_node is : {}'.format(getStartNode()))
    #rint('goal_node is : {}'.format(getGoalNode()))
