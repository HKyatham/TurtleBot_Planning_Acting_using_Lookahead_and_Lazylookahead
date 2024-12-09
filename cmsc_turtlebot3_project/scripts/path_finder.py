from queue import PriorityQueue
import numpy as np
import cv2
import time
# import gtpyhop

class Path_Finder:
    def __init__(self, start_index=(100,100), goal_index=(650,100), step=10, clerance = 20):
        self.goal_index = goal_index
        self.step = step
        # self.l = l
        self.map = np.zeros((700, 200, 3), dtype=np.uint8)
        self.V = np.zeros((200//10, 700//10), dtype=bool)  # Matrix to store visited nodes
        self.map_Generator(clerance)
        if (self.check_Obstacle(start_index) or self.check_Obstacle(goal_index) or
                start_index[0] < 0 or start_index[1] < 0 or goal_index[0] < 0 or goal_index[1] < 0 or
                start_index[0] > 699 or start_index[1] > 199 or goal_index[0] > 699 or goal_index[1] > 199):
            raise Exception("Start or Goal indices are in Obstacle space.")
        # cv2.imshow("", cv2.rotate(self.map, cv2.ROTATE_90_COUNTERCLOCKWISE))
        # cv2.waitKey(0)
        # self.graph = {tuple(start_index): [-1, -1, 0]}
        # self.costNode = {tuple(start_index): round(0 + self.euclidean(start_index), 1)}
        # self.closed_set = set()
        # self.path = []

    def map_Generator(self,l):
        start = time.time()
        x = np.arange(700).reshape((700, 1))
        y = np.arange(200).reshape((1, 200))

        condition1 = (x < l) | (y < l) | (x > 699-l) | (y > 199-l)
        condition2 = (x > 200-l) & (x < 225 + l) & (y > 100 - l)
        condition3 = (x > 575 - l) & (x < 600 + l) & (y < 150 + l) & (y > 50 - l)

        # condition8 = (x < 5) | (y < 5) | (x > 1194) | (y > 494)
        condition4 = (x > 200) & (x < 225) & (y > 100)
        condition5 = (x > 575) & (x < 600) & (y < 150) & (y > 50)
          
        final_condition = condition1 | condition2 | condition3 
        self.obstacle_indices = np.vstack(np.where(final_condition == True)).T
        self.map[final_condition] = [0, 0, 255]
        self.map[condition4 | condition5 ] = [255, 0, 0]
        # print(self.obstacle_indices)
        print("Map Generation time: ", time.time() - start)

    def check_Obstacle(self, index):
        if index[0] <= 0 or index[1] <=0 or index[0] >= 699 or index[1] >= 199:
            return True
        return np.any(np.all(self.obstacle_indices == [index[0], index[1]], axis=1))

    def Up(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        new_position[1]+=step
        # print(current_position)
        if(self.check_Obstacle(new_position)):
            # print("Inside up if")
            return (tc, cost, current_position)
        else:
            # print("Inside up else")
            return (round(cost+step + self.euclidean([new_position[0], new_position[1]]), 1),cost+step,tuple(new_position))
    def UpRight(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        new_position[0]+=step
        new_position[1]+=step
        if(self.check_Obstacle(new_position)):
            return (tc, cost, current_position)
        else:
            return (round(cost+1.2 * step + self.euclidean([new_position[0], new_position[1]]), 1),round(cost+1.2 * step,1),tuple(new_position))
    def Right(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        new_position[0]+=step
        # new_position[1]+=1
        if(self.check_Obstacle(new_position)):
            return (tc, cost, current_position)
        else:
            return (round(cost+step + self.euclidean([new_position[0], new_position[1]]), 1),cost+step,tuple(new_position))
    def DownRight(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        new_position[0]+=step
        new_position[1]-=step
        if(self.check_Obstacle(new_position)):
            return (tc,cost, current_position)
        else:
            return (round(cost+1.2 * step + self.euclidean([new_position[0], new_position[1]]), 1),round(cost+1.2 * step,1),tuple(new_position))
    def Down(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        # new_position[0]-=1
        new_position[1]-=step
        if(self.check_Obstacle(new_position)):
            return (tc,cost, current_position)
        else:
            return (round(cost+step + self.euclidean([new_position[0], new_position[1]]), 1),cost+step,tuple(new_position))
    def DownLeft(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        new_position[0]-=step
        new_position[1]-=step
        if(self.check_Obstacle(new_position)):
            return (tc,cost, current_position)
        else:
            return (round(cost+1.2 * step + self.euclidean([new_position[0], new_position[1]]), 1),round(cost+1.2 * step,1),tuple(new_position))
    def Left(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        new_position[0]-=step
        # new_position[1]-=1
        if(self.check_Obstacle(new_position)):
            return (tc,cost, current_position)
        else:
            return (round(cost+step + self.euclidean([new_position[0], new_position[1]]), 1),cost+step,tuple(new_position))
    def UpLeft(self, current_position, cost, tc, step):
        new_position = list(current_position).copy()
        new_position[0]-=step
        new_position[1]+=step
        if(self.check_Obstacle(new_position)):
            return (tc,cost, current_position)
        else:
            return (round(cost+1.2 * step + self.euclidean([new_position[0], new_position[1]]), 1),round(cost+1.2 * step,1),tuple(new_position))
    
    def move(self, i, index, cost, tc):
        if(i == 0):
            return self.Up(index, cost, tc, self.step)
        elif(i == 1):
            return self.UpRight(index, cost, tc, self.step)
        elif(i == 2):
            return self.Right(index, cost, tc, self.step)
        elif(i == 3):
            return self.DownRight(index, cost, tc, self.step)
        elif(i == 4):
            return self.Down(index, cost, tc, self.step)
        elif(i == 5):
            return self.DownLeft(index, cost, tc, self.step)
        elif(i == 6):
            return self.Left(index, cost, tc, self.step)
        else:
            return self.UpLeft(index, cost, tc, self.step)
        # theta1 = (theta + index[2]) % 360
        # if self.check_Obstacle([x, y]) or x < 0 or x > 699 or y < 0 or y > 199:
        #     return [tc, cost, index]
        # return [round(cost + self.l + self.euclidean([x, y]), 1), cost + self.l, (x, y)]

    def node_exists(self, index,graph):
        return tuple(index) in graph

    def euclidean(self, index):
        return np.sqrt((self.goal_index[0] - index[0]) ** 2 + (self.goal_index[1] - index[1]) ** 2)

    def checkGoal(self, index):
        return (index[0] - self.goal_index[0]) ** 2 + (index[1] - self.goal_index[1]) ** 2 <= 100 

    def Astar(self, start_index, goal_index):
        start = time.time()
        graph = {tuple(start_index): [-1, -1, 0]}
        costNode = {tuple(start_index): round(0 + self.euclidean(start_index), 1)}
        closed_set = set()
        path = []
        self.goal_index = goal_index
        queue = PriorityQueue()
        d = [round(0 + self.euclidean(start_index), 1), 0, start_index]  # Convert start_index to tuple
        queue.put(d)

        while not queue.empty():
            # print("In A Star loop graph...",graph)
            tc, cost, index = queue.get()
            if self.checkGoal(index):
                if not self.node_exists(self.goal_index,graph):
                    graph[self.goal_index] =  index
                # self.goal_index = index
                print("Goal Reached...")
                print("A star time: ", time.time() - start)
                return self.shortest_path(path,graph)

            if index in closed_set:
                continue

            closed_set.add(index)

            # Update visited matrix
            # self.V[index[1] // 10, index[0] // 10] = True

            for i in range(8):
                d = self.move(i, index, cost, tc)

                if not self.node_exists(d[2],graph):
                    # Check if the move is within threshold and not visited
                    # if not self.V[d[2][1] // 10, d[2][0] // 10]:
                    graph[d[2]] = index # Convert d[2] to tuple
                    costNode[d[2]] = d[0]
                    queue.put(d)
                else:
                    if d[0] < costNode[d[2]]:
                        queue.put(d)
                        costNode[d[2]] = d[0]
                        graph[d[2]] = index
        return self.shortest_path(path,graph)
        # return self.path


    def checker(self,graph):
        for index in graph:
            self.map[index[0]][index[1]] = [0, 255, 0]
        cv2.imshow("Checker", cv2.rotate(self.map, cv2.ROTATE_90_COUNTERCLOCKWISE))
        cv2.waitKey(0)
        
    def shortest_path(self,path,graph):
        # print("In A Star loop graph...",graph)
        i = self.goal_index
        while i[0] != -1:
            j = graph[i]
            if j[0] != -1:
                path.append(j) 
            i = j if j[0] != -1 else (-1,)
        return path[::-1]