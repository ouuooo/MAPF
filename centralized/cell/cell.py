
from threading import local


class Location:
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y

    # 为==运算符提供支持
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    # 对应于调用内置的str()函数将该对象转换成字符串
    def __str__(self):
        return str((self.x, self.y))

class is_empty:
    def __init__(self) -> None:
        pass

class Local:
    OBSTACLES = 0
    CELL = 1
    MAINPATH = 2
    
    def __init__(self, location):
        self.type = -1
        self.locaton = location

class cell:
    def __init__(self):
        self.location = Local.location
        self.time = 0
        self.lifter = 0
        self.num = 0

class cell_num:
    def __init__(self):
        self.number = 0
        self.locations = [0]
        self.time = 0
        self.lifter = 0

class envornment:
    def __init__(self) -> None:
        pass

    def agent_dict(self):
        return 0

    def admissible_heuristic(self):
        return

    def is_at_goal(self):
        return


    def get_neighbors(self):
        return 

