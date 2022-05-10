"""

Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)

"""
import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
from a_star import AStar
import time


class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y

    # 为==运算符提供支持
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    # 对应于调用内置的str()函数将该对象转换成字符串
    def __str__(self):
        return str((self.x, self.y))


class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    # 对应于调用内置的hash()函数来获取该对象的hash码
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))

    def is_equal_except_time(self, state):
        # 当前位置state.location是否位于self.location处
        return self.location == state.location

    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    VERTEX = 1
    EDGE = 2

    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'


class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time)+str(self.location))

    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'


class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        # (state_1.time, state_1.location, state_2.location)
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2

    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2

    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))

    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'


class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])


class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension      # [x_range, y_range]
        self.obstacles = obstacles      # [(obs_x01, obs_y01), (obs_x02, obs_y02)]

        self.agents = agents        # [{'srart': , 'goal': , 'name': }, {'srart': , 'goal': , 'name': }]
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # 加入判断是否可以action
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors

    def get_first_conflict(self, solution):
        # max_t-最大cost花费
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            # 全部agents的agent两两组合
            for agent_1, agent_2 in combinations(solution.keys(), 2):
            # combinations(iterable, r)-求列表或生成器iterable中指定数目r的元素不重复的所有组合
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                # 判断发生节点占用冲突
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)
                # 判断发生相遇（相向）冲突
                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        # 节点占用冲突
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint
        # 相遇（相向）冲突
        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        """
        output: agent_name在t时刻的位置
        """
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        # 判断节点state(time, (x,y))可行
        '''
        x在dimension内
        y在dimension内
        (time, (x,y))不在vertex_constraint内
        (x, y)不是静态障碍
        '''
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1, state_2):
        # 判断边(state_1.location, state_2.location)可行
        '''
        state_1 = (time, location_x1, location_y1)
        state_2 = (time+1, location_x2, location_y2)
        '''
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        # h: 预估到终点的代价
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    def is_at_goal(self, state, agent_name):
        # agent_name是否到达终点
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})
            # update-将参数字典加入指定字典，有相同的键会替换为参数的值
            # {agent['agent0']:{'start':(0,location_x,location_y), 'goal':(0,location_x,location_y)}}

    def compute_solution(self):
        """
        output: solution = [{agent:local_solution},...]
        """
        solution = {}
        for agent in self.agent_dict.keys():
            # agent = 'agent_name'
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            # setdefault(key, value)-设置key的默认值为value。key的值存在时不修改，不存在key时添加，并且其值为value
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
            # solution-所有agent的路径
        return solution

    def compute_solution_cost(self, solution):
        """
        output: 所有agent的路径的总长,即消耗的能源
        """
        return sum([len(path) for path in solution.values()])


class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash(self.cost)

    def __lt__(self, other):
        return self.cost < other.cost

class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()

    def search(self):
        """
        high level search
        """
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            # 选取open_set中cost最小的solution
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            # 将方案P的constraint_dict同步到env
            self.env.constraint_dict = P.constraint_dict
            # 发现冲突
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                print("solution found")

                return self.generate_plan(P.solution)

            # 处理冲突
            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                # P的constraint_dict更新到env中
                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution):
        """
        output: plan = {'agent0': [{'t': 0, 'x': 0, 'y': 0}, {'t': 1, 'x': 0, 'y': 0}, {'t': 2, 'x': 1, 'y': 0}, {'t': 3, 'x': 2, 'y': 0}], 
        'agent1': [{'t': 0, 'x': 2, 'y': 0}, {'t': 1, 'x': 1, 'y': 0}, {'t': 2, 'x': 1, 'y': 1}, {'t': 3, 'x': 1, 'y': 0}, {'t': 4, 'x': 0, 'y': 0}]}
        """
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # {'agents': [{'start': [0, 0], 'goal': [2, 0], 'name': 'agent0'}, {'start': [2, 0], 'goal': [0, 0], 'name': 'agent1'}], 
    # 'map': {'dimensions': [3, 3], 'obstacles': [(0, 1), (2, 1)]}}

    dimension = param["map"]["dimensions"]      # [x_range, y_range]
    obstacles = param["map"]["obstacles"]       # [(obs_x01, obs_y01), (obs_x02, obs_y02)]
    agents = param['agents']        # [{'srart': , 'goal': , 'name': }, {'srart': , 'goal': , 'name': }]

    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    if not solution:
        print(" Solution not found" )
        return

    # Write to output filepip
    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # {'cost': 9, 
    # 'schedule': {'agent0': [{'t': 0, 'x': 0, 'y': 0}, {'t': 1, 'x': 0, 'y': 0}, {'t': 2, 'x': 1, 'y': 0}, {'t': 3, 'x': 2, 'y': 0}], 
    # 'agent1': [{'t': 0, 'x': 2, 'y': 0}, {'t': 1, 'x': 1, 'y': 0}, {'t': 2, 'x': 1, 'y': 1}, {'t': 3, 'x': 1, 'y': 0}, {'t': 4, 'x': 0, 'y': 0}]}}

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

start = time.time()
if __name__ == "__main__":
    main()
end = time.time()
print(end-start)

