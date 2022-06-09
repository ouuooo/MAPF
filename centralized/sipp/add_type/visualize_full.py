#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import argparse
import math
import random
import imageio

Colors = ['orange', 'blue', 'green']


class Animation:
  def __init__(self, map, schedule):
    self.map = map
    self.schedule = schedule
    self.combined_schedule = {}
    self.combined_schedule.update(self.schedule["schedule"])

    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    self.fig = plt.figure(frameon=False, figsize=(12 * aspect, 12))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0.1,right=0.9,bottom=0.1,top=0.9, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    xmax = map["map"]["dimensions"][0] - 0.5
    ymax = map["map"]["dimensions"][1] - 0.5

    # 设置主次坐标刻度，打开网格
    xmajorLocator = MultipleLocator(1)
    ymajorLocator = MultipleLocator(1)
    self.ax.xaxis.set_major_locator(xmajorLocator)
    self.ax.yaxis.set_major_locator(ymajorLocator)
    xminorLocator = MultipleLocator(0.5)
    yminorLocator = MultipleLocator(0.5)
    self.ax.xaxis.set_minor_locator(xminorLocator)
    self.ax.yaxis.set_minor_locator(yminorLocator)
    self.ax.xaxis.grid(True, which='minor')
    self.ax.yaxis.grid(True, which='minor')
    self.ax.spines['bottom'].set_position(('data', -0.5))

    # self.ax.relim()
    
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # self.ax.set_xticks(np.arange(xmin, xmax, 1))
    # self.ax.set_yticks(np.arange(ymin, ymax, 1))

    # plt.axis('off')
    # self.ax.axis('tight')
    # self.ax.axis('on')

    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='gray'))
    mainpath = find_coordinate(map["map"]["detail"], ['o','t'])
    for o in mainpath:
      x, y = o[0], o[1]
      self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='gray', edgecolor='white'))
    for s in self.schedule["obstacles"]:
      x, y = s[0], s[1]
      self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black', edgecolor='gray'))

    # create agents:
    self.T = 0
    # draw goals first
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      self.patches.append(Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=Colors[0], edgecolor='black', alpha=0.5))
      plt.text(d["goal"][0]-0.12, d["goal"][1]-0.12, d["name"][-1])
      self.patches.append(Rectangle((d["start"][0] - 0.25, d["start"][1] - 0.25), 0.5, 0.5, facecolor=Colors[0], edgecolor='black', alpha=0.5))
      plt.text(d["start"][0]-0.12, d["start"][1]-0.12, d["name"][-1])
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      name = d["name"]
      self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.3, facecolor=Colors[0], edgecolor='black')
      self.agents[name].original_face_color = Colors[0]
      self.patches.append(self.agents[name])
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''))
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      self.artists.append(self.agent_names[name])

    # self.ax.set_axis_off()
    # self.fig.axes[0].set_visible(False)
    # self.fig.axes.get_yaxis().set_visible(False)

    # self.fig.tight_layout()

    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1)*3,
                               interval=50,
                               blit=True)
    # fig:绘制动画的画布；func：动画参数；frams: 动画长度，依次循环包含的帧数，在函数运行时，其值会传给动画函数update(i)的形参i
    # init_func：动画的其实状态；interval:跟新频率；blit: 是否更新整张图，False更新整张图，True只更新有变化的点                               

  def save(self, file_name, speed):
    # self.anim.save(file_name, "ffmpeg", fps=10 * speed, dpi=200)
    # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})
    self.anim.save(file_name, writer="pillow")
    # imageio.mimsave(file_name, self.animate_func, fps=1)

  def show(self):
    plt.show()    # 显示图片

  def init_func(self):
    # 将图形加入图中
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):
    for agent_name, agent in self.combined_schedule.items():
      pos = self.getState(i/3, agent)
      p = (pos[0], pos[1])
      self.agents[agent_name].center = p
      self.agent_names[agent_name].set_position(p)

    # reset all colors
    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)

    # check drive-drive collisions
    agents_array = [agent for _,agent in self.agents.items()]
    for i in range(0, len(agents_array)):
      for j in range(i+1, len(agents_array)):
        d1 = agents_array[i]
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 0.7:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0]["x"]), float(d[0]["y"])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"])])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
    else:
      return np.array([float(d[-1]["x"]), float(d[-1]["y"])])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos


def find_coordinate(map, symples):
    result = []
    for symple in symples:
      for index1, value1 in enumerate(map):
          if symple in value1:
              row = index1
              for index2,value2 in enumerate(list(map[index1])):
                  if symple == value2:
                      column = index2
                      result.append((column, row))
    return result


def full_cell(agents, mainpath, obstacles, dimension):
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

    obs = random.sample(obstacles, 1000)
    return obs


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=200, help="speedup-factor")
  args = parser.parse_args()


  with open(args.map) as map_file:
    map = yaml.load(map_file, Loader=yaml.FullLoader)

  with open(args.schedule) as states_file:
    schedule = yaml.load(states_file, Loader=yaml.FullLoader)

  animation = Animation(map, schedule)

  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()
