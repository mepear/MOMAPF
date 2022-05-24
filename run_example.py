from multiprocessing import Pool
import re
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from momapf import mocbs_new, arguments
from momapf.utils import Map


def Run(args, index=None, use_bound=None):
  """
  Run different scenarios
  """
  if index == None:
    index = args['index']

  f1 = open(file='./benchmark/{}/{}-random-{}.scen'.format(args['experiment_name'], args['experiment_name'], index), mode='rb')
  f2 = open(file='./benchmark/{}/{}.map'.format(args['experiment_name'], args['experiment_name']), mode='rb')
  cost_grids = np.load(file='./benchmark/{}/{}-matrix.npy'.format(args['experiment_name'], args['experiment_name']))
  # cost_grids = np.random.randint(1, 3, (20, 32, 32))
  # np.save('./benchmark/{}/{}-matrix.npy'.format(args['experiment_name'], args['experiment_name']), cost_grids)

  if use_bound == None:
    if args['use_cost_bound'] == 'False':
      use_cost_bound = False
    else:
      use_cost_bound = True
  else:
    use_cost_bound = use_bound

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

  # for i in range(32):
  #   print(grids[i])
  # exit()

  sx = np.array(sx_list)  # start x = column in grid image
  sy = np.array(sy_list)  # start y = rows in grid image, the kth component corresponds to the kth robot.
  gx = np.array(gx_list)  # goal x
  gy = np.array(gy_list)  # goal y

  cgrids = [cost_grids[0], cost_grids[1]]
  clist = args['cost_name']

  G = Map(grids, cgrids, clist)

  ### Invoke MO-CBS planner ###
  success, res_path, res_cost, result_dict=mocbs_new.RunMocbsMAPF(G, sx, sy, gx, gy, np.inf, 1500, use_cost_bound=use_cost_bound)

  print(success)
  print("Paretal Optimal Paths Number:", len(res_cost))
  print(res_cost)
  # print(open_list_res)
  print("Number of close list:", result_dict['closed_num'])
  print("Number of low level calls:", result_dict['low_level_calls'])
  print("Total Low Level Time:", result_dict['low_level_time'])
  print("Total Time:", result_dict['time'])
  print("Branching Factors: ", result_dict['branch_factor'])
  # print(res_path)

  # df = pd.DataFrame({'success': [], "res_num": [], "close_list_num": [], "low_level_num": [], "time": []})
  # df.to_csv("./benchmark/{}-result/plot_{}_{}.csv".format(args['experiment_name'], index, int(use_cost_bound)), index=False, sep=',')
  #
  # data = [success, len(res_cost), close_list_res, low_level_calls, time_res]
  # df = pd.read_csv('./benchmark/{}-result/plot_{}_{}.csv'.format(args['experiment_name'], index, int(use_cost_bound)))
  # df.loc[1] = data
  # df.to_csv("./benchmark/{}-result/plot_{}_{}.csv".format(args['experiment_name'], index, int(use_cost_bound)), index=False, sep=',')
  return


def main(args, index=None, use_bound=None):
  Run(args, index, use_bound)
  return


if __name__ == '__main__':
  args = vars(arguments.get_args(sys.argv[1:]))
  print("begin of main")
  # pool = Pool(processes=15)
  # for i in range(1, 26):
  #   pool.apply_async(main, (args, i, True, ))
  #   pool.apply_async(main, (args, i, False, ))
  # pool.close()
  # pool.join()
  main(args)
  print("end of main")
