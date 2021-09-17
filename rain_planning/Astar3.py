import matplotlib.pyplot as plt
import numpy as np
import math
import operator
from shapely.geometry import Point, Polygon, LineString
from itertools import product,permutations
import time

obstacle_list=[[(2, 10), (7, 10), (6, 7), (4, 7), (4, 9), (2, 9)],[(3, 1), (3, 6), (4, 6), (4, 1)],[(7, 3), (7, 8), (9, 8), (9, 3)]]

class Node:
    def __init__(self,x,y,cost,parent,nearest_nodes,index):
        self.x=x
        self.y=y
        self.parent=parent
        self.cost=cost
        self.nearest_nodes=nearest_nodes
        self.index=index

class Astar:
    def __init__(self):
        self.start = [0,0]
        self.goal = [10,10]
        self.rate = 0.8
        #self.num_nodes = 1000
    
    def within(self,obstacle_list,rnd_node):
        isWithin = False
        point = Point(rnd_node[0],rnd_node[1])
        if(point.within(Polygon(obstacle_list[0]))==True or point.within(Polygon(obstacle_list[1]))==True or point.within(Polygon(obstacle_list[2]))==True):
            isWithin = True

        return isWithin

    def cross(self,obstacle_list,rnd_node,x,y):
        isCrosses=False
        point=Point(rnd_node[0],rnd_node[1])
        node=(x,y)
        coord=(rnd_node,node)
        line=LineString(coord)
        if(line.crosses(Polygon(obstacle_list[0]))==True or line.crosses(Polygon(obstacle_list[1]))==True or line.crosses(Polygon(obstacle_list[2]))==True or point.touches(Polygon(obstacle_list[0]))==True or point.touches(Polygon(obstacle_list[1])) ==True or point.touches(Polygon(obstacle_list[2]))==True):
            isCrosses=True
        return isCrosses
    
    def node_gen(self):
        self.nodes=[]
        self.nodes.append(Node(self.start[0],self.start[1],0,0,[],0))
        a=np.arange(0,10.5,0.5).tolist()
        b=list(product(a,repeat=2))
        for coord in b:
            if(self.within(obstacle_list,coord)==True):
                continue
            self.nodes.append(Node(coord[0],coord[1],0,0,[],len(self.nodes)))
            for i in range(0,len(self.nodes)-1):
                dist=math.sqrt((coord[0]-self.nodes[i].x)**2 + (coord[1]-self.nodes[i].y)**2)
                if(dist<self.rate and self.cross(obstacle_list,coord,self.nodes[i].x,self.nodes[i].y)==False):
                    self.nodes[i].nearest_nodes.append(len(self.nodes)-1)
                    self.nodes[len(self.nodes)-1].nearest_nodes.append(i)

    def search(self):
        self.node_gen()
        self.open_list=[]
        IsGoal=False
        for node in self.nodes:
            node.cost=np.inf
        self.nodes[0].cost=0
        for index in self.nodes[0].nearest_nodes:
            self.nodes[index].cost=math.sqrt((self.nodes[0].x-self.nodes[index].x)**2 + (self.nodes[0].y-self.nodes[index].y)**2) + math.sqrt((self.nodes[index].x-self.goal[0])**2 + (self.nodes[index].y-self.goal[1])**2)
            self.nodes[index].parent=0
        
        for node in self.nodes:
            self.open_list.append([node.index,node.cost])
        self.open_list.sort(key=lambda x:x[1])

        start_time=time.time()
        while(IsGoal==False and len(self.open_list)>0):
            if(self.open_list[0][0]==len(self.nodes)-1): 
                #len(nodes)-1 is the last nodes index 
                IsGoal=True
            
            for index in self.nodes[self.open_list[0][0]].nearest_nodes:
                cost=self.open_list[0][1] + math.sqrt((self.nodes[self.open_list[0][0]].x-self.nodes[index].x)**2 + (self.nodes[self.open_list[0][0]].y-self.nodes[index].y)**2) + math.sqrt((self.nodes[index].x-self.goal[0])**2 + (self.nodes[index].y-self.goal[1])**2)
                if(cost<self.nodes[index].cost):
                    self.nodes[index].cost=cost
                    self.nodes[index].parent=self.open_list[0][0]
            
            self.open_list.pop(0)
            for array in self.open_list:
                array[1]=self.nodes[array[0]].cost
            self.open_list.sort(key=lambda x:x[1])

        path1=self.plot_path()
        print(time.time() - start_time)

    def plot_path(self):
        '''
        poly1=Polygon(obstacle_list[0])
        poly2=Polygon(obstacle_list[1])
        poly3=Polygon(obstacle_list[2])
        x1,y1=poly1.exterior.xy
        x2,y2=poly2.exterior.xy
        x3,y3=poly3.exterior.xy
        plt.plot(x1,y1)
        plt.plot(x2,y2)
        plt.plot(x3,y3)
        '''
        current=len(self.nodes)-1
        plot_list=[]
        
        while(current!=0):
            plot_list.append(self.nodes[current])
            current=self.nodes[current].parent
        

        plot_list.append(Node(self.start[0],self.start[1],0,0,[],0))
        plot_list.reverse()
        i=0

    
        '''
        while i<len(plot_list)-1:
            plt.plot([plot_list[i].x,plot_list[i+1].x],[plot_list[i].y,plot_list[i+1].y],color='blue')
            i=i+1
        for node in plot_list:
            plt.scatter(node.x,node.y,color='blue',s=30)
        '''
   
if __name__ == '__main__':
    print("start planning")
    astar=Astar()
    astar.search()
    plt.show()
