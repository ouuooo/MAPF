import argparse
import yaml
from math import fabs
from graph_generation_type import SippGraph, State
from sipp import SippPlanner
import random
import time
from copy import deepcopy
import bisect
import visualize_full


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
    obs = random.sample(obstacles, int(len(obstacles)/3))
    return obs

def output_free_agent(task,schedule):
    """
    找到第一个到达终点的小车
    output: {'agent0': {'x': 15, 'y': 4, 't': 10, 'state': 0}}
    """
    for t_now in range(500):
        agents_location = []
        for agent_name,location_t in schedule.items():
            if len(location_t) < t_now+1:
                station = deepcopy(location_t[-1])
                station["t"] += 1   # 卸货/装货时间
                print("{0}在t={1}时刻空闲".format(agent_name,station["t"]))
                # station.update({"state":task[int(agent_name[-1])]["state"]})
                return {agent_name:station}
            else:
                # t_now时刻 小车坐标
                agents_location.append({agent_name:location_t[t_now]})

def run_agent(task,schedule):
    """
    找到第一个到达终点的小车
    output: {'agent0': {'x': 15, 'y': 4, 't': 10, 'state': 0}}
    """
    free = False
    for t_now in range(500):
        for agent_name,location_t in schedule.items():
            station = location_t.pop(0)
            if not location_t:
                agent_name_free = deepcopy(agent_name)
                station_free = deepcopy(station)
                station_free["t"] = station["t"] + 1   # 卸货/装货时间
                print("{0}在t={1}时刻空闲".format(agent_name_free,station_free["t"]))
                # station_free.update({"state":task[int(agent_name_free[-1])]["state"]})
                free = True
        if free:
            return [agent_name_free, station_free]


def agents_location(free, wait_list, task, schedule, new_task):
    """
    output: [{'goal': [15, 4], 'name': 'agent0', 'start': [9, 2], 'state': 0}]
    """
    time_compute = 3
    x_range = 8
    y_range = 8
    t_stop = 2
    # agent 在 t_now 时刻空闲
    agent = free[0]
    t_now = free[1]["t"]
    location = (free[1]["x"],free[1]["y"])

    # 整理路径时间窗
    path_turntime = {}
    for a,path in wait_list.items():
        path_turntime.setdefault(a,[])
        for i in range(len(path)):
            path_turntime[a].append(path[i]["t"])

    agent_next_location = []
    x = t_now+time_compute
    for a,time_w in path_turntime.items():
        if a == agent:
            state = {'goal': new_task, 'name': a, 'start': [location[0],location[1]], 'state': task[int(a[-1])]["state"]}
            agent_next_location.append(state)
            continue
        t_now_index = bisect.bisect_left(time_w,t_now)
        x_index = bisect.bisect_left(time_w,x)

        # 判断此a小车schedule[a][t_now]  距空闲小车的距离
        location_a = (schedule[a][t_now]["x"],schedule[a][t_now]["y"])
        if abs(location_a[0]-location[0])<x_range or abs(location_a[1]-location[1])<y_range:
            stop_index = bisect.bisect_left(time_w,t_now+t_stop)

            # 有拐点: 终点/拐弯
            if stop_index > t_now_index:
                if stop_index == len(time_w):
                    s = schedule[a][-1]
                    print("{0}距{1}太近，到目标点{2}停下".format(a,agent,(s["x"],s["y"])))
                else:
                    s = schedule[a][time_w[t_now_index]]
                    print("{0}距{1}太近，到临近拐点{2}停下".format(a,agent,(s["x"],s["y"])))
            # 无拐点
            else:
                s = schedule[a][t_now+t_stop]
                print("{0}距{1}太近，立刻刹车预计到点{2}停下".format(a,agent,(s["x"],s["y"])))

        else:
            # 中间 暂停点
            if x_index != t_now_index:
                # a小车在[t_now,x]时间段内到达终点 / 中间要拐弯
                s = schedule[a][time_w[t_now_index]]
                if x_index == len(time_w):
                    print("{0}不影响{1}，到临近目标点{2}停下".format(a,agent,(s["x"],s["y"])))
                else:
                    print("{0}不影响{1}，到临近拐点{2}停下".format(a,agent,(s["x"],s["y"])))
            # 不停
            # if x_index - t_now_index == 1:
            else:
                s = schedule[a][x]
                print("{0}不影响{1}，不停下，假想的等待点为{2}".format(a,agent,(s["x"],s["y"])))
                
        state = {'goal': task[int(a[-1])]["goal"], 'name': a, 'start': [s["x"],s["y"]], 'state': task[int(a[-1])]["state"]}
        agent_next_location.append(state)

    return agent_next_location

def run_agents_location(free, wait_list, task, schedule, new_task):
    """
    output: [{'goal': [15, 4], 'name': 'agent0', 'start': [9, 2], 'state': 0}]
    """
    time_compute = 3
    x_range = 8
    y_range = 8
    t_stop = 2
    agent = free[0]
    t_now = free[1]["t"]
    location = (free[1]["x"],free[1]["y"])

    # 整理路径时间窗
    path_turntime = {}
    for a,path in wait_list.items():
        path_turntime.setdefault(a,[])
        for i in range(len(path)):
            path_turntime[a].append(path[i]["t"])

    agent_next_location = []
    x = t_now+time_compute
    for a,time_w in path_turntime.items():
        if a == agent:
            state = {'goal': new_task[0], 'name': a, 'start': [location[0],location[1]], 'state': new_task[1]}
            agent_next_location.append(state)
            continue
        t_now_index = bisect.bisect_left(time_w,t_now)
        x_index = bisect.bisect_left(time_w,x)

        # 判断此a小车schedule[a][0]  距空闲小车的距离
        location_a = (schedule[a][0]["x"],schedule[a][0]["y"])
        if abs(location_a[0]-location[0])<x_range or abs(location_a[1]-location[1])<y_range:
            stop_index = bisect.bisect_left(time_w,t_now+t_stop)

            # 有拐点: 终点/拐弯
            if stop_index > t_now_index:
                if stop_index == len(time_w):
                    s = schedule[a][-1]
                    print("{0}距{1}太近，到目标点{2}停下".format(a,agent,(s["x"],s["y"])))
                else:
                    s = schedule[a][time_w[t_now_index]-t_now]
                    print("{0}距{1}太近，到临近拐点{2}停下".format(a,agent,(s["x"],s["y"])))
            # 无拐点
            else:
                s = schedule[a][0+t_stop]
                print("{0}距{1}太近，立刻刹车预计到点{2}停下".format(a,agent,(s["x"],s["y"])))

        else:
            # 中间 暂停点
            if x_index != t_now_index:
                # a小车在[0,x]时间段内到达终点 / 中间要拐弯
                s = schedule[a][time_w[t_now_index]-t_now]
                if x_index == len(time_w):
                    print("{0}不影响{1}，到临近目标点{2}停下".format(a,agent,(s["x"],s["y"])))
                else:
                    print("{0}不影响{1}，到临近拐点{2}停下".format(a,agent,(s["x"],s["y"])))
            # 不停
            else:
                s = schedule[a][x-t_now]
                print("{0}不影响{1}，不停下，假想的等待点为{2}".format(a,agent,(s["x"],s["y"])))
                
        state = {'goal': task[int(a[-1])]["goal"], 'name': a, 'start': [s["x"],s["y"]], 'state': task[int(a[-1])]["state"]}
        agent_next_location.append(state)

    return agent_next_location

def main():
    parser = argparse.ArgumentParser()
    data_map = r"centralized\sipp\add_type\data\warehouse_input_30x30_10_state.yaml"
    data_output = r"centralized\sipp\add_type\data\warehouse_output_10.yaml"
    data_wait = r"centralized\sipp\add_type\data\wait_location.yaml"
    parser.add_argument("--map", help="input file containing map and dynamic obstacles", default=data_map)
    parser.add_argument("--output", help="output file with the schedule", default=data_output)
    parser.add_argument("--wait", help="output file with the schedule", default=data_wait)
    
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

    with open(args.wait, 'r') as output_yaml:
        try:
            wait_location = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)


    dimension = map["map"]["dimensions"]      # [x_range, y_range]
    cell = find_coordinate(map["map"]["detail"], '.')       # [(obs_x01, cell_y01), (obs_x02, obs_y02)]
    mainpath = find_coordinate(map["map"]["detail"], 'o') 
    agents = map['agents']        # [{'srart': , 'goal': , 'name': }, {'srart': , 'goal': , 'name': }]
    obstacles_full = full_cell(agents, mainpath, cell, dimension)
    # print(len(cell), len(obstacles_full))

    map["map"].update({"obstacles":obstacles_full})
    map["map"].update({"main_path":mainpath})

    output["obstacles"] = obstacles_full

    # 路径计算
    """# {agents:[{start:[0,0],goal:[2,2],name:agent0}, {start:[2,2],goal:[0,0],name:agent1}],
    #  map:{dimensions:[3,3], obstacles:[(0,1), (2,1)]}}
    for i in range(len(map["agents"])):
        sipp_planner = SippPlanner(map,i)
        if map["agents"][i]["state"] == 0:
            sipp_planner.obstacles = []
    
        if sipp_planner.compute_plan():
            plan,plan_wait = sipp_planner.get_plan()
            # plan = {agent_name:path_list}
            output["schedule"].update(plan)
            wait_location.update(plan_wait)
            map["dynamic_obstacles"].update(plan)

            with open(args.output, 'w') as output_yaml:
                yaml.safe_dump(output, output_yaml)  
            with open(args.wait, 'w') as output_yaml:
                yaml.safe_dump(wait_location, output_yaml)  
        else: 
            print("Plan not found")"""

    # new_task = input()

    new_task = [[20,16], 0]
    while new_task != "none":
        # 模拟执行任务
        # 输出空闲小车
        free_agent = run_agent(map["agents"], output["schedule"])
        print("\n-------- 增添新任务 ---------- ")
        print("'goal':{0[0]}, 'name':{1[0]}, 'start':{1[1]}, 'state':{0[1]}\n".format(new_task,[free_agent[0],[free_agent[1]['x'],free_agent[1]['y']]]))

        # 新任务发布时所有小车的坐标
        agents_plan = run_agents_location(free_agent, wait_location, map["agents"], output["schedule"], new_task)
        print(agents_plan)

        for i in range(len(map["agents"])):
            sipp_planner = SippPlanner(map,i)
            if map["agents"][i]["state"] == 0:
                sipp_planner.obstacles = []

            if sipp_planner.compute_plan():
                plan,plan_wait = sipp_planner.get_plan()    # plan = {agent_name:path_list}

                # schedule, wait_location在agent不同等待时间的基础上更新，主要是补齐等待一起出发的指令
                output["schedule"].update(plan)
                wait_location.update(plan_wait)

                map["dynamic_obstacles"].update(plan)

            else: 
                print("Plan not found")
        # new_task = input()

    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)  
    with open(args.wait, 'w') as output_yaml:
        yaml.safe_dump(wait_location, output_yaml)  
    # visualize_full.visual(map, output)


start = time.time()
if __name__ == "__main__":
    main()
end = time.time()
print("time is ",end-start)
