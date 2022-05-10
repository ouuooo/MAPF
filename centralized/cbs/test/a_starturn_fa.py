"""

AStar search

author: Ashwin Bose (@atb033)

"""
import math

from numpy import fabs


class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        # h-预估到达终点的代价
        # 判断是否到达终点
        self.get_neighbors = env.get_neighbors      # 修改搜索邻居的约束

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        total_xy = [self.location(current)]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
            total_xy.append(self.location(current))
        # print(total_xy)
        return total_path[::-1]

    def search(self, agent_name):
        """
        low level search 
        """
        initial_state = self.agent_dict[agent_name]["start"]
        initial_xy = self.location(initial_state)
        goal = self.agent_dict[agent_name]["goal"]
        goal_xy = self.location(goal)
        step_cost = 1       # 每步的消耗，修改加上转弯代价
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = self.admissible_heuristic(initial_xy, goal_xy)

        while open_set:
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)

            if self.is_at_goal(self.location(current), goal_xy):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost + self.if_turn(came_from, current, neighbor)       # 当前state预估的g

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(self.location(neighbor), goal_xy)
        return False

    def if_turn(self, came_from, current, neighbor):
        turn_cost = 0
        while current in came_from.keys():
            neighbor_xy = self.location(neighbor)
            camefrom_xy = self.location(came_from[current])
            if abs(neighbor_xy[0]-camefrom_xy[0]) or abs(neighbor_xy[1]-camefrom_xy[1]):
                turn_cost = 1
            break
        return turn_cost

    def admissible_heuristic(self, local, goal):
        return math.fabs(goal[0]-local[0])+math.fabs(goal[1]-local[1])

    def is_at_goal(self, local, goal):
        return local==goal

    def location(self, loc):
        return [loc.location.x, loc.location.y]

