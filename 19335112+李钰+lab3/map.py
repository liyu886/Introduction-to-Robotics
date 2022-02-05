import math
from PIL import Image
import numpy as np
import networkx as nx
import copy

BLACK='B'
WHITE='W'
BORDER = '@'

class DrawMap():
    """ 读进一张图片，二值化成为有障碍物的二维网格化地图，并提供相关操作
    """
    def __init__(self,imgage_file):
        """图片变二维数组"""
        temp_map = []
        img = Image.open(imgage_file)
        img_gray = img.convert('L')  # 地图灰度化,0-black,255-white
        img_arr = np.array(img_gray)
        img_binary = np.where(img_arr<127,0,255)
        
        for x in range(img_binary.shape[0]):
            temp_row = []
            for y in range(img_binary.shape[1]):
                status = WHITE if img_binary[x,y] == 255 else BLACK 
                temp_row.append(status)
            temp_map.append(temp_row)

        self.map = temp_map
        self.cols = len(self.map[0])
        self.rows = len(self.map)

        '''对障碍物稍做膨胀，'''
        for x in range(0, self.rows):
            for y in range(0, self.cols):
                if self.map[x][y] == WHITE:
                    for i in range(0,5):
                        if y > i and y < self.cols - i and (self.map[x][y-i] == BLACK or self.map[x][y+i] == BLACK):
                            self.map[x][y] = BORDER

        
    def is_valid_xy(self, x,y):
        if x < 0 or x >= self.rows or y < 0 or y >= self.cols:
            return False
        return True
    
    def e_distance(self, xy1, xy2):
        """两个像素点之间的欧几里得距离"""
        dis = 0
        for (x1, x2) in zip(xy1, xy2):
            dis += (x1 - x2)**2
        return dis**0.5

    def m_distance(self,xy1,xy2):
        """两个像素点之间的曼哈顿距离"""
        dis = 0
        for x1,x2 in zip(xy1,xy2):
            dis+=abs(x1-x2)
        return dis

    def out_black(self, xy1, xy2):
        """碰撞检测 两点之间的连线是否经过障碍物"""
        steps = max(abs(xy1[0]-xy2[0]), abs(xy1[1]-xy2[1])) # 取横向、纵向较大值，确保经过的每个像素都被检测到
        xs = np.linspace(xy1[0],xy2[0],steps+1)
        ys = np.linspace(xy1[1],xy2[1],steps+1)
        for i in range(1, steps - 1): # 第一个节点和最后一个节点是 xy1，xy2，无需检查
            if self.map[math.ceil(xs[i])][math.ceil(ys[i])] != WHITE:
                return False
            if self.map[math.ceil(xs[i] - 2)][math.ceil(ys[i])] != WHITE:
                return False
            if self.map[math.ceil(xs[i])][math.ceil(ys[i] - 1)]:
                return False
            
        return True

    def drawpath(self,path):
        out = []
        for x in range(self.rows):
            temp = []
            for y in range(self.cols):
                if self.map[x][y]==BLACK:
                    temp.append(0)
                elif self.map[x][y]==WHITE:
                    temp.append(255)
                else:
                    temp.append(255)
            out.append(temp)
        for x,y in path:
            out[x][y] = 0
            out[x][y - 1] = 0
            out[x - 1][y] = 0
            if x + 1 < self.rows - 1:
                out[x + 1][y] = 0
            if y + 1 < self.cols - 1:
                out[x][y + 1] = 0
        out = np.array(out)
        img = Image.fromarray(np.uint8(out))
        img.show()


def path_length(path):
    """计算路径长度"""
    l = 0
    for i in range(len(path)-1):
        x1,y1 = path[i]
        x2,y2 = path[i+1]
        if x1 == x2 or y1 == y2:
            l+=1.0
        else:
            l+=1.4
    return l



class PRM(DrawMap):
    def __init__(self, img_file, **param):
        """ 随机路线图算法(Probabilistic Roadmap, PRM)
        **param: 关键字参数，用以配置规划参数
                start: 起点坐标 (x,y)
                end: 终点左边 (x,y)
                num_sample: 采样点个数，默认100 int
                distance_neighbor: 邻域距离，默认100 float
        """
        DrawMap.__init__(self,img_file)
        
        self.num_sample = param['num_sample'] if 'num_sample' in param else 100
        self.distance_neighbor = param['distance_neighbor'] if 'distance_neighbor' in param else 100
        self.G = nx.Graph() # 无向图，保存构型空间的完整连接属性
        
    def learn(self):
        # 随机采样节点
        while len(self.G.nodes)<self.num_sample:
            XY = (np.random.randint(0, self.rows),np.random.randint(0, self.cols)) # 随机取点
            if self.is_valid_xy(XY[0],XY[1]) and self.map[XY[0]][XY[1]] == WHITE: # 不是障碍物点
                self.G.add_node(XY)
        # 邻域范围内进行碰撞检测，加边
        for node1 in self.G.nodes:
            for node2 in self.G.nodes:
                if node1==node2:
                    continue
                dis = self.e_distance(node1,node2)
                if dis<self.distance_neighbor and self.out_black(node1,node2):
                    self.G.add_edge(node1,node2,weight=dis) # 边的权重为 欧几里得距离
    
    def find_path(self,startXY=None,endXY=None):
        """ 使用学习得到的无障碍连通图进行寻路
        """
        # 寻路时再将起点和终点添加进图中，以便一次学习多次使用 
        temp_G = copy.deepcopy(self.G)
        startXY = tuple(startXY) if startXY else (60, self.cols-60)
        endXY = tuple(endXY) if endXY else (self.rows-65,35)
        temp_G.add_node(startXY)
        temp_G.add_node(endXY)
        for node1 in [startXY, endXY]: # 将起点和目的地连接到图中
            for node2 in temp_G.nodes:
                dis = self.e_distance(node1,node2)
                if dis<self.distance_neighbor and self.out_black(node1,node2):
                    temp_G.add_edge(node1,node2,weight=dis) # 边的权重为 欧几里得距离
        # 直接调用networkx中求最短路径的方法
        path = nx.shortest_path(temp_G, source=startXY, target=endXY)
        
        return self.construct_path(path)

    def construct_path(self, path):
        """find_path寻路得到的是连通图的节点，扩展为经过的所有像素点"""
        out = []
        for i in range(len(path)-1):
            xy1,xy2=path[i],path[i+1]
            steps = max(abs(xy1[0]-xy2[0]), abs(xy1[1]-xy2[1])) # 取横向、纵向较大值，确保经过的每个像素都被检测到
            xs = np.linspace(xy1[0],xy2[0],steps+1)
            ys = np.linspace(xy1[1],xy2[1],steps+1)
            for j in range(0, steps+1): 
                out.append((math.ceil(xs[j]), math.ceil(ys[j])))
        return out
        
#======= test case ==============
prm = PRM('maze.png',num_sample=500,distance_neighbor=200)
prm.learn()
path = prm.find_path()
prm.drawpath(path)
print('Path length:',path_length(path))

