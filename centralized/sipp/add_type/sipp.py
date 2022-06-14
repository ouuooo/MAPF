"""

SIPP implementation  

author: Ashwin Bose (@atb033)

See the article: DOI: 10.1109/ICRA.2011.5980306

"""

import argparse
import yaml
from math import fabs
from graph_generation_type import SippGraph, State
import heapq


class SippPlanner(SippGraph):
    def __init__(self, map, agent_id):
        # 类的继承
        SippGraph.__init__(self, map)

        # agent0: start,goal,name
        self.start = tuple(map["agents"][agent_id]["start"])
        self.goal = tuple(map["agents"][agent_id]["goal"])
        self.name = map["agents"][agent_id]["name"]
        self.open = []

    def get_successors(self, state):
        '''
        input: state-->State()
        output: neighbours-->list(State())
        '''
        successors = []
        m_time = 1      # stemp_cost_time
        # 找到neighbours坐标
        neighbour_list = self.get_valid_neighbours(state.position)

        # 找到neighbours可到达的时间
        for neighbour in neighbour_list:
            start_t = state.time + m_time
            end_t = state.interval[1] + m_time
            # state处不等待的到达时间start_t, 等待最久的到达时间end_t
            # 预计占用neighbour的时间段(start_t, end_t)是否冲突
            for i in self.sipp_graph[neighbour].interval_list:
                # 最早出发没有位置 or 等待到最后一刻没有位置
                if i[0] > end_t or i[1] < start_t:
                    # 冲突
                    continue
                # i = neighbour可用的interval_time
                time = max(start_t, i[0]) 
                s = State(neighbour, time, i)
                successors.append(s)
        return successors

    def get_heuristic(self, position):
        '''
        input: (x,y)
        output: |x-x0|+|y-y0|
        '''
        return fabs(position[0] - self.goal[0]) + fabs(position[1]-self.goal[1])

    def compute_plan(self):
        self.open = []
        goal_reached = False
        cost = 1

        s_start = State(self.start, 0)      # 起点

        # 起点的g=0
        self.sipp_graph[self.start].g = 0
        # 估计起点代价
        f_start = self.get_heuristic(self.start)
        # 更新到起点.f
        self.sipp_graph[self.start].f = f_start     # self.sipp_graph[start]--> SippGrid()

        # 将起点加入到open_list
        heapq.heappush(self.open, (f_start, s_start))
        # self.open.append((f_start, s_start))

        while (not goal_reached):
            if self.open == {}: 
                # Plan not found
                return 0
            # s = (self.open.pop(0))[1] = s_start--> State()
            s = heapq.heappop(self.open)[1]     # 为什么没有排序
            # 计算邻接点
            successors = self.get_successors(s)
    
            # for neighbor in neighbors
            for successor in successors:
                # 去除已经走过的点
                if self.sipp_graph[successor.position].g > self.sipp_graph[s.position].g + cost:
                    self.sipp_graph[successor.position].g = self.sipp_graph[s.position].g + cost
                    self.sipp_graph[successor.position].parent_state = s

                    turn_cost = 0
                    if successor.position != self.start:
                        before_p = self.sipp_graph[s.position].parent_state.position
                        next_p = successor.position
                        if (before_p[0]-next_p[0])*(before_p[1]-next_p[1]):
                            # 转弯耗费代价
                            turn_cost = 3

                    if successor.position == self.goal:
                        print("Plan successfully calculated!!")
                        goal_reached = True
                        break

                    self.sipp_graph[successor.position].f = self.sipp_graph[successor.position].g + self.get_heuristic(successor.position) + turn_cost
                    heapq.heappush(self.open, (self.sipp_graph[successor.position].f, successor))
                    # self.open.append((self.sipp_graph[successor.position].f, successor))
                    # bisect.insort(self.open, (self.sipp_graph[successor.position].f, successor))

        # Tracking back
        start_reached = False
        plan = []
        current = successor     # goal
        while not start_reached:
            plan.insert(0,current)
            if current.position == self.start:
                start_reached = True
            current = self.sipp_graph[current.position].parent_state

        # 增加转弯等待
        count_time = 0
        self.plan = []
        for index,current in enumerate(plan):
            # count_time += 1
            self.plan.append(State(current.position, current.time+count_time))
            if current.position != self.goal and current.position != self.start:
                after_position = plan[index+1].position
                before_posiont = plan[index-1].position
                if (after_position[0]-before_posiont[0])*(after_position[1]-before_posiont[1]) != 0:
                    count_time += 1
                    self.plan.append(State(current.position, current.time+count_time))


        return 1
            
    def get_plan(self):
        path_list = []

        # first setpoint
        setpoint = self.plan[0]
        temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
        path_list.append(temp_dict)

        for i in range(len(self.plan)-1):
            for j in range(self.plan[i+1].time - self.plan[i].time-1):
                x = self.plan[i].position[0]
                y = self.plan[i].position[1]
                t = self.plan[i].time
                setpoint = self.plan[i]
                temp_dict = {"x":x, "y":y, "t":t+j+1}
                path_list.append(temp_dict)

            setpoint = self.plan[i+1]
            temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
            path_list.append(temp_dict)

        # 修改输出格式
        path_turn = []
        for i in range(len(path_list)):
            # 加上起点和终点
            if i == 0 or i ==len(path_list)-1:
                path_turn.append(path_list[i])
            elif path_list[i-1]['x'] == path_list[i]['x'] and path_list[i-1]['y'] == path_list[i]['y']:
                path_turn.append(path_list[i])
        #print(path_list)
        data_wait = {self.name:path_turn}
        
        data = {self.name:path_list}
        return data, data_wait



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic obstacles")
    parser.add_argument("output", help="output file with the schedule")
    
    args = parser.parse_args()
    
    with open(args.map, 'r') as map_file:
        try:
            map = yaml.load(map_file, Loader=yaml.FullLoader)
            # {agents:[{start:[0,0],goal:[2,2],name:agent0}, {start:[2,2],goal:[0,0],name:agent1}],
            #  map:{dimensions:[3,3], obstacles:[(0,1), (2,1)]}}
        except yaml.YAMLError as exc:
            print(exc)

    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # compute first plan
    sipp_planner = SippPlanner(map,0)

    if sipp_planner.compute_plan():
        plan = sipp_planner.get_plan()
        output["schedule"].update(plan)
        with open(args.output, 'w') as output_yaml:
            yaml.safe_dump(output, output_yaml)  
    else: 
        print("Plan not found")


if __name__ == "__main__":
    main()