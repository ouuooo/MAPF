"""

Graph generation for sipp 

author: Ashwin Bose (@atb033)

See the article: 10.1109/ICRA.2011.5980306

"""

import argparse
import yaml
from bisect import bisect

class State(object):
    def __init__(self, position=(-1,-1), t=0, interval=(0,float('inf'))):
        self.position = tuple(position)
        self.time = t
        self.interval = interval
        # position在t时刻占用self.sipp_graph[neighbour].interval_list中的interval
        # 用来记录agent在position可以待的时间[t, interval[1]]

    def __lt__(self, other):
        return self.time<other.time

class SippGrid(object):
    def __init__(self):
        # self.position = ()
        self.interval_list = [(0, float('inf'))]
        self.f = float('inf')
        self.g = float('inf')
        self.parent_state = State()     # 父节点

    def split_interval(self, t, last_t = False):
        """
        input: t, last_t=len(schedule)-1
        output: 整理self.interval_list
        Function to generate safe-intervals
        """
        m_time = 3
        for interval in self.interval_list:
            if last_t:
                if t<=interval[0]:
                    self.interval_list.remove(interval)
                elif t>interval[1]:
                    continue
                else:
                    self.interval_list.remove(interval)
                    self.interval_list.append((interval[0], t-m_time))
            else:
                if t == interval[0]:
                    self.interval_list.remove(interval)
                    if t+1 <= interval[1]:
                        self.interval_list.append((t+m_time, interval[1]))
                elif t == interval[1]:
                    self.interval_list.remove(interval)
                    if t-1 <= interval[0]:
                        self.interval_list.append((interval[0],t-m_time))
                # 判断 t between interval[0] and interval[1]
                elif bisect(interval,t) == 1:
                    self.interval_list.remove(interval)
                    self.interval_list.append((interval[0], t-m_time))
                    self.interval_list.append((t+m_time, interval[1]))
            self.interval_list.sort()

class SippGraph(object):
    def __init__(self, map):
        self.map = map
        self.dimensions = map["map"]["dimensions"]

        self.main_path = [tuple(w) for w in map["map"]["main_path"]]
        self.obstacles = [tuple(v) for v in map["map"]["obstacles"]]        
        self.dyn_obstacles = map["dynamic_obstacles"]

        self.sipp_graph = {}
        self.init_graph()
        self.init_intervals()

    def init_graph(self):
        '''
        添加dimensions内所有点,初始定义图
        self.sipp_graph={(i,j):SippGride(interval_list, f, g, parent_state)}
        '''
        for i in range(self.dimensions[0]):
            for j in range(self.dimensions[1]):
                grid_dict = {(i,j):SippGrid()}
                self.sipp_graph.update(grid_dict)

    def init_intervals(self):
        '''
        根据动态障碍,更新(i, j)空闲时间
        self.sipp_graph[dyn_obstacles.position].intervals_list=[]
        '''
        # 如果dynamic_obstacles True, return
        if not self.dyn_obstacles: return
        for schedule in self.dyn_obstacles.values():
            # for location in schedule:
            for i in range(len(schedule)):
                location = schedule[i]
                last_t = i == len(schedule)-1

                position = (location["x"],location["y"])
                t = location["t"]

                # 更新intervals
                self.sipp_graph[position].split_interval(t, last_t)
                # print(str(position) + str(self.sipp_graph[position].interval_list))     

    def is_valid_position(self, position):
        '''
        output: (position in dimensions) and (position not obstacles)
        '''
        dim_check = position[0] in range(self.dimensions[0]) and  position[1] in range(self.dimensions[1])
        obs_check = position not in self.obstacles
        # print(dim_check)
        return dim_check and obs_check

    def get_valid_neighbours(self, position):
        '''
        output: 4 neighbours(valid) of position
        '''
        neighbour_list = []

        if position in self.main_path:
            up = (position[0], position[1]+1)
            if self.is_valid_position(up): neighbour_list.append(up)

            down = (position[0], position[1]-1)
            if self.is_valid_position(down): neighbour_list.append(down)

            left = (position[0]-1, position[1])
            if self.is_valid_position(left): neighbour_list.append(left)

            right = (position[0]+1, position[1])
            if self.is_valid_position(right): neighbour_list.append(right)
        else:
            up = (position[0]+1, position[1])
            if self.is_valid_position(up): neighbour_list.append(up)

            down = (position[0]-1, position[1])
            if self.is_valid_position(down): neighbour_list.append(down)

        return neighbour_list


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic dyn_obstacles")
    args = parser.parse_args()
    
    with open(args.map, 'r') as map_file:
        try:
            map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    graph = SippGraph(map)
    # print(graph.get_valid_neighbours((0,0)))
    # print(graph.sipp_graph[(1,1)].interval_list)
    # print(graph.get_valid_neighbours((1,2)))

if __name__ == "__main__":
    main()

