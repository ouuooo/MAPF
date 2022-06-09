"""

Extension of SIPP to multi-robot scenarios

author: Ashwin Bose (@atb033)

See the article: 10.1109/ICRA.2011.5980306

"""

import argparse
import yaml
from math import fabs
from graph_generation_type import SippGraph, State
from sipp import SippPlanner
import random
import time


def find_coordinate(map, symple):
    '''提取标号为symple的坐标'''
    result = []
    for index1, value1 in enumerate(map):
        if symple in value1:
            row = index1
            for index2,value2 in enumerate(list(map[index1])):
                if symple == value2:
                    column = index2
                    result.append((column, row))
    return result


def full_cell(agents, mainpath, obstacles, dimension):
    '''随机生成指定数量的静态障碍'''
    locals = set()
    for agent in agents:
        locals.add((agent['start'][0], agent['start'][1]))
        locals.add((agent['goal'][0], agent['goal'][1]))
    for local in locals:
        x=local[0]
        if local in obstacles:
            obstacles.remove(local)
        while not (x<0) and ((x, local[1]) not in mainpath):
            if (x, local[1]) in obstacles:
                obstacles.remove((x, local[1]))
            x-=1
        x=local[0]+1
        while not (x>dimension[0]-1) and ((x, local[1]) not in mainpath):
            if (x, local[1]) in obstacles:
                obstacles.remove((x, local[1]))
            x+=1
            
    # 障碍数目
    obs = random.sample(obstacles, int(len(obstacles)/1))
    return obs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", help="input file containing map and dynamic obstacles", default="centralized\\sipp\\add_type\\data\\warehouse_input_30x30_10.yaml")
    parser.add_argument("--output", help="output file with the schedule", default="centralized\\sipp\\add_type\\data\\warehouse_output_10.yaml")
    
    args = parser.parse_args()
    
    # Read Map
    with open(args.map, 'r') as map_file:
        try:
            map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # Output file
    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = map["map"]["dimensions"]      # [x_range, y_range]
    cell = find_coordinate(map["map"]["detail"], '.')       # [(obs_x01, cell_y01), (obs_x02, obs_y02)]
    mainpath = find_coordinate(map["map"]["detail"], 'o') 
    agents = map['agents']        # [{'srart': , 'goal': , 'name': }, {'srart': , 'goal': , 'name': }]
    obstacles_full = full_cell(agents, mainpath, cell, dimension)
    print(len(cell), len(obstacles_full))

    map["map"].update({"obstacles":obstacles_full})
    map["map"].update({"main_path":mainpath})

    output["obstacles"] = obstacles_full

    # {agents:[{start:[0,0],goal:[2,2],name:agent0}, {start:[2,2],goal:[0,0],name:agent1}],
    #  map:{dimensions:[3,3], obstacles:[(0,1), (2,1)]}}
    for i in range(len(map["agents"])):
        sipp_planner = SippPlanner(map,i)
    
        if sipp_planner.compute_plan():
            plan = sipp_planner.get_plan()
            # plan = {agent_name:path_list}
            output["schedule"].update(plan)
            map["dynamic_obstacles"].update(plan)

            with open(args.output, 'w') as output_yaml:
                yaml.safe_dump(output, output_yaml)  
        else: 
            print("Plan not found")


start = time.time()
if __name__ == "__main__":
    main()
end = time.time()
print("time is ",end-start)
