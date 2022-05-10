import os
import sys
import time
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import colors
import argparse
import yaml

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

import env

cell_num = pd.read_excel('file/ex_cellnum_dp.xlsx')
cell_sort = cell_num.sort_values(by='time')
number = cell_num.loc[:, 'num'].values
w_s = cell_num.loc[:, 'weight'].values
v_s = cell_num.loc[:, 'time'].values
y_s = cell_num.loc[:, 'y'].values
y_max = np.array(y_s).max()
lifter_s = cell_num.loc[:, 'lifter'].values
v = cell_sort.loc[:, 'time'].values
v_0 = (v_s - v[0])/(v[-1] - v[0])
n = number[-1]+1
c = [300, 0]
p = [0.9, 0.1]

parser = argparse.ArgumentParser()
parser.add_argument("param", help="input file containing map and obstacles")
args = parser.parse_args()

# Read from input file
with open(args.param, 'r') as param_file:
    try:
        param = yaml.load(param_file, Loader=yaml.FullLoader)
    except yaml.YAMLError as exc:
        print(exc)


def same(plan):
    p = []
    for x in plan:
        p.extend(x)
    ps = np.unique(p)
    if len(ps) < d:
        return True
    else:
        return False


def order(orders):
    random.shuffle(orders)
    order_s = []
    for i in orders:
        order_s.append(i)
    return order_s


def roll(orders):
    ord = np.array([0]*d*N).reshape(N, d)
    ord_roll = np.array(orders)
    for i in range(N):
        ord_roll = np.roll(ord_roll, 10)
        ord[i] = ord_roll
    return ord


def limitation(num):
    w, v, a, time_c, b = [], [], [], [], []
    c_cumsum = np.array(c).cumsum()
    for i in num:
        w.append(w_s[i])
        v.append(v_s[i])
    walk = np.array(w).cumsum()
    time_a = np.array(v).cumsum()
    for i in range(len(c)):
        if i == 0:
            a.append((walk >= c_cumsum[i]).argmax())
            b.append(num[:a[i] + 1])
        else:
            aa = (walk >= c_cumsum[i]).argmax()
            if walk[aa]-walk[a[i-1]] >= c[i]:
                a.append(aa)
                b.append(num[a[i-1] + 1:a[i] + 1])
            else:
                a.append(aa+1)
                b.append(num[a[i - 1] + 1:a[i] + 1])
    time_c = time_a[a]
    time_aa = np.dot(np.array(time_c), np.array(p).T)
    return b, time_aa, a


def run(numbers):
    answer, f1, a = limitation(numbers)
    y_an = [0]*(y_max+1)
    lifter_mean = [0]*3
    for i in range(len(c)):
        for an in answer[i]:
            # y_std.append(y0[an])    # answer的y值
            y_an[y_s[an]] = y_an[y_s[an]] + w_s[an]*p[i]  # 每个y使用的巷道数
            # lifter_std.append(lifter[an])       # answer的提升机
            lifter_mean[lifter_s[an]] = lifter_mean[lifter_s[an]] + w_s[an]*p[i]
    f2 = sum((y_an - sum(np.array(c)*np.array(p)) / (y_max + 1))**2)
    f3 = sum((lifter_mean - sum(np.array(c)*np.array(p)) / 3)**2)
    if k == G:
        y_a = [0] * (y_max + 1)
        lifter_m = [0] * 3
        for i in range(len(c)):
            for an in answer[i]:
                y_a[y_s[an]] = y_a[y_s[an]] + w_s[an]  # 每个y使用的巷道数
                lifter_m[lifter_s[an]] = lifter_m[lifter_s[an]] + w_s[an]
        plt.scatter(range(y_max + 1), np.round(y_an, decimals=3))
        plt.plot(range(y_max + 1), [sum(np.array(c) * np.array(p)) / (y_max + 1)] * (y_max + 1))
        plt.ylim((0, 30))
        plt.show()
        print(lifter_m, np.round(lifter_mean, decimals=3))
    return f1, f2, f3, answer


def F_m():
    f_min = [np.array(f_1).min(), np.array(f_2).min(), np.array(f_3).min()]
    f_max = [np.array(f_1).max(), np.array(f_2).max(), np.array(f_3).max()]
    return f_min, f_max


def F(f1, f2, f3):
    e = 0.0001
    if k >=0:
        f_m = [f1, f2, f3]
        for i in range(3):
            if f_m[i] < f_min[i]:
                f_min[i] = f_m[i]
            if f_m[i] > f_max[i]:
                f_max[i] = f_m[i]
    # f1 = (f1 - f_min[0]+e) / (f_max[0] - f_min[0]+e)
    f2 = (f2 - f_min[1]+e) / (f_max[1] - f_min[1]+e)
    f3 = (f3 - f_min[2]+e) / (f_max[2] - f_min[2]+e)
    a1 = 1  # 433
    a2 = 0
    a3 = 0

    f = a1*f1+a2*f2+a3*f3
    return f


def change(an):
    # 坐标转换
    fram = pd.read_excel('file/ex_cellnum1.xlsx')
    x, y = [], []
    for i in an:
        for j in range(len(fram.cell_x[fram.cell_num == i].values)):
            x.append(fram.cell_x[fram.cell_num == i].values[j])
            y.append(fram.cell_y[fram.cell_num == i].values[j])
    # for j in range(len(x)):
        # print('(', x[j], ',', y[j], ')')
    return x, y


def crossover_and_mutation(father, mother, CROSSOVER_RATE = 0.9):
    # 孩子先得到父亲的全部基因（这里我把一串二进制串的那些0，1称为基因）
    child0 = np.full((n), -1)
    if np.random.rand() < CROSSOVER_RATE:  # 产生子代时不是必然发生交叉，而是以一定的概率发生交叉
        cross_points = random.sample(range(n), int(n/3))
        child0[cross_points] = father[cross_points]
        index = 0
        for i in mother:
            if i not in child0:
                if child0[index] == -1:
                    child0[index] = i
                else:
                    index += 1
        # np.delete(child0, cross_points)
        # np.insert(child0, cross_points0, father[cross_points])
        # cross_points = random.sample(range(n-int(n/20)), int(n/20))
        # np.insert(child[:-int(n/20)], cross_points, child[-int(n/20):])
        # child[cross_points] = child[-int(n/20):]
        # 孩子得到位于交叉点后的母亲的基因
    child0 = mutation(child0)  # 每个后代有一定的机率发生变异

    return child0


def mutation(child):
    MUTATION_RATE = 0.003
    ch = child
    if np.random.rand() < MUTATION_RATE:    # 以MUTATION_RATE的概率进行变异
        ch = order(number)
    return ch


# SFLA参数
N = 300  # 种群数，有100中分配方案
n_n = 20  # 子群中的青蛙数量，10个分配方案为一组
d = n  # 解元素的个数
m = N // n_n  # 子群数量，共有10组
L = int(n/20)  # 组内优化更新最差青蛙次数，组内优化的最差解的步数。
G = 500  # 种群迭代次数
P = []
x_plot = []
y_plot = []
y1 = []
start = time.time()
t = []
k = -1
f_1, f_2, f_3 = [], [], []
count = 0
a_m = [0]*N

# step1 生成蛙群，生成由0~2组成有9个元素的数组
# t = roll(number)
for i in range(N):
    # to = t[i].tolist()
    to = order(number)
    t.append(to)
    t_an1, t_an2, t_an3, an = run(to)
    for j in an:
        a_m[i] += len(j)
    f_1.append(t_an1)
    f_2.append(t_an2)
    f_3.append(t_an3)

a_max = np.array(a_m).max()
populations = np.array(t).reshape(N, d)
f_min, f_max = F_m()
f = np.array(F(f_1, f_2, f_3))
populations = populations[np.argsort(f), :]
f = f[np.argsort(f)]

Xg = populations[0]  # 首个全局最优解

for k in range(G):
    # step2 划分子群
    # step3 局部搜索
    for i in range(m):
        temp = crossover_and_mutation(populations[i], populations[random.randrange(0, N)])
        f_temp1, f_temp2, f_temp3, ans = run(temp)
        if same(ans):
            temp = order(number)
            f_temp1, f_temp2, f_temp3, _ = run(temp)
        f_temp = F(f_temp1, f_temp2, f_temp3)
        if f_temp < f[-m + i - 1]:
            populations[-m + i - 1] = temp
            f[-m + i - 1] = f_temp
        else:
            populations[i] = populations[0]
            populations[-m + i - 1] = order(number)
            f_temp1, f_temp2, f_temp3, _ = run(populations[-m + i - 1])
            f[-m + i - 1] = F(f_temp1, f_temp2, f_temp3)
    populations = populations[np.argsort(f), :]
    f = f[np.argsort(f)]

    # print(round(f[0], 4), k)
    x_plot.append(k)
    y_plot.append(f[0])
    y1.append(f[-1])
plt.plot(x_plot, y_plot, 'orange')
plt.show()
w, v = [], []
for i in populations[0]:
    w.append(w_s[i])
    v.append(v_0[i])
answer, _, a = limitation(populations[0])
x_aa, y_aa = [], []
for i in answer:
    x_a, y_a = change(i)
    x_aa.append(x_a)
    y_aa.append(y_a)
k = G
_ = run(populations[0])
if len(x_aa) == sum(c):
    print('full')
else:
    print(len(x_aa[0]))

end = time.time()
print('forg', end-start, count)

Env = env.Env()


def drawmap(rows, cols, x_a, y_a):
    frame = pd.read_excel('file/ex_cellnum.xlsx')
    key_x = frame.loc[:, 'cell_x'].values
    key_y = frame.loc[:, 'cell_y'].values
    obsIndex = Env.zhuzi
    main_path = Env.main_path
    # 创建全部为空地的地图栅格，其中空地以数字1表征
    filed = np.ones((rows, cols))

    # 修改栅格地图中障碍物的数值，其中以数值5表示
    for x in obsIndex:
        filed[x[0], x[1]] = 2
    for x in range(len(key_x)):
        filed[key_x[x], key_y[x]] = 3
    for i in range(len(x_a[0])):
        filed[x_a[0][i], y_a[0][i]] = 4
    for i in range(len(x_a[1])):
        filed[x_a[1][i], y_a[1][i]] = 6
    for x in main_path:
        filed[x[0], x[1]] = 7

    # 绘制图像，利用matplotlib的热力图进行绘制
    # 设置色条的范围，从0~8
    cmap = colors.ListedColormap(['none', 'gray', 'black', 'white',
                                  'red', 'royalblue', 'blue', 'green', 'blue'])

    # 绘图函数
    fig, ax = plt.subplots(figsize=(11, 8))

    # 绘制热力图
    # 其中vmin和vmax对应栅格地图数值的颜色与cmap一一对应
    # cbar设置false将色条设置为不可见
    sns.heatmap(filed, cmap=cmap, vmin=0, vmax=8, square=True, linewidths=1, linecolor='darkgray', ax=ax, cbar=False)

    # 设置图标题
    ax.set_title('draw map by python')
    ax.set_ylabel('rows')
    ax.set_xlabel('cols')

    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top')

    # 显示图像
    plt.show()

    # 设置图标的数字个数文字，放在plt.show下面能居中
    ax.set_xticks(np.arange(cols))
    ax.set_xticklabels(np.arange(cols))
    ax.set_yticks(np.arange(rows))
    ax.set_yticklabels(np.arange(rows))


rows = 42
cols = 49

drawmap(rows, cols, x_aa, y_aa)
