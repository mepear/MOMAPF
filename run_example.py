from multiprocessing import Pool
import re
import sys

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from momapf import mocbs_new, arguments
from momapf.utils import Map


def Run(args, index=None, agent=None, use_bound=None, use_disjoint_bound=None, use_joint=None, use_caching=None):
  """
  Run different scenarios
  """
  if index == None:
    index = args['index']

  f1 = open(file='./benchmark/{}/{}-random-{}.scen'.format(args['experiment_name'], args['experiment_name'], index), mode='rb')
  f2 = open(file='./benchmark/{}/{}.map'.format(args['experiment_name'], args['experiment_name']), mode='rb')
  cost_grids = np.load(file='./benchmark/{}/{}-matrix.npy'.format(args['experiment_name'], args['experiment_name']))
  hill_grid = np.load(file='./benchmark/{}/{}-hill.npy'.format(args['experiment_name'], args['experiment_name']))

  if use_bound == None:
    if args['use_cost_bound'] == 'False':
      use_cost_bound = False
    else:
      use_cost_bound = True
  else:
    use_cost_bound = use_bound

  if use_disjoint_bound == None:
    exit()

  if use_joint == None:
    if args['use_joint_splitting'] == 'False':
      use_joint_splitting = False
    else:
      use_joint_splitting = True
  else:
    use_joint_splitting = use_joint

  if use_caching == None:
    if args['use_caching'] == 'False':
      caching = False
    else:
      caching = True
  else:
    caching = use_caching

  if args['draw_graph'] == 'False':
    draw_graph = False
  else:
    draw_graph = True

  if agent != None:
    agent_num = agent
  else:
    agent_num = args['robot_num']

  _ = f1.readline()
  _ = f2.readline()
  datapat_1 = re.compile(b'height\s(.*)\n')
  datapat_2 = re.compile(b'(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\n')

  sx_list = []
  sy_list = []
  gx_list = []
  gy_list = []

  data = f2.readline()
  match = datapat_1.match(data)
  height = int(match.group(1))
  data = f2.readline()
  datapat_1 = re.compile(b'width\s(.*)\n')
  match = datapat_1.match(data)
  width = int(match.group(1))
  grids = np.ones((height, width), dtype=np.int)
  _ = f2.readline()

  for _ in range(agent_num):
    data = f1.readline()
    match = datapat_2.match(data)
    sx_list.append(int(match.group(5)))
    sy_list.append(int(match.group(6)))
    gx_list.append(int(match.group(7)))
    gy_list.append(int(match.group(8)))

  for i in range(height):
    data = f2.readline()
    for j in range(width):
      if data[j] == 46 or data[j] == 71: # This represent '.' and 'G'
        grids[i][j] = 0

  sx = np.array(sx_list)  # start x = column in grid image
  sy = np.array(sy_list)  # start y = rows in grid image, the kth component corresponds to the kth robot.
  gx = np.array(gx_list)  # goal x
  gy = np.array(gy_list)  # goal y

  cgrids = [cost_grids[0], cost_grids[1]]
  clist = args['cost_name']

  G = Map(grids, cgrids, hill_grid, clist)

  ### Invoke MO-CBS planner ###
  success, res_path, res_cost, result_dict = mocbs_new.RunMocbsMAPF(G, sx, sy, gx, gy, np.inf,
                    1000, use_disjoint_bound=use_disjoint_bound, use_cost_bound=use_cost_bound, use_joint_splitting=use_joint_splitting, draw_graph=draw_graph, use_caching=caching)

  # print(success)
  print("Paretal Optimal Paths Number:", len(res_cost))
  print(res_cost)
  # print(open_list_res)
  print("Number of close list:", result_dict['closed_num'])
  print("Number of low level calls:", result_dict['low_level_calls'])
  print("Total Low Level Time:", result_dict['low_level_time'])
  print("Total Time:", result_dict['time'])
  print("Branching Factors: ", result_dict['branch_factor'])
  # print(res_path)

  df = pd.DataFrame({'success': [], "res_num": [], "time": [], "low_level_calls": [], "branch_factor": []})
  df.to_csv("./benchmark/{}-result/plot_{}_{}.csv".format(args['experiment_name'], agent_num, index, int(use_cost_bound) + int(use_joint_splitting) + int(use_disjoint_bound)), index=False, sep=',')

  data = [success, len(res_cost), result_dict['time'], result_dict['low_level_calls'], result_dict['branch_factor']]
  df = pd.read_csv('./benchmark/{}-result/plot_{}_{}.csv'.format(args['experiment_name'], agent_num, index, int(use_cost_bound) + int(use_joint_splitting) + int(use_disjoint_bound)))
  df.loc[1] = data
  df.to_csv("./benchmark/{}-result/plot_{}_{}.csv".format(args['experiment_name'], agent_num, index, int(use_cost_bound) + int(use_joint_splitting) + int(use_disjoint_bound)), index=False, sep=',')
  return


def main(args, index=None, agent=None, use_bound=None, use_disjoint_bound=None, use_splitting=None, use_caching=None):
  Run(args, index, agent, use_bound, use_disjoint_bound, use_splitting, use_caching)
  return


if __name__ == '__main__':
  args = vars(arguments.get_args(sys.argv[1:]))
  print("begin of main")
  pool = Pool(processes=30)
  for j in range(5, 21):
    for i in range(1, 26):
      pool.apply_async(main, (args, i, j, True, True, True, True))
      pool.apply_async(main, (args, i, j, True, True, False, True))
      pool.apply_async(main, (args, i, j, True, False, False, True))
      pool.apply_async(main, (args, i, j, False, False, False, True))
    pool.close()
    pool.join()
  # main(args, use_disjoint_bound=True)
  print("end of main")