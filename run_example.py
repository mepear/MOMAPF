import context
import time
from multiprocessing import Pool
import re
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import common 
import mocbs
import arguments
import moastar # NAMOA*
import momstar


def Run(args, index=None, use_bound=None):
  """
  Run different scenarios
  """
  if index == None:
    index = args['index']

  f1 = open(file='./benchmark/{}/{}-random-{}.scen'.format(args['experiment_name'], args['experiment_name'], index), mode='rb')
  f2 = open(file='./benchmark/{}/{}.map'.format(args['experiment_name'], args['experiment_name']))
  cost_grids = np.load(file='./benchmark/{}/{}-matrix.npy'.format(args['experiment_name'], args['experiment_name']))

  if use_bound == None:
    if args['use_cost_bound'] == 'False':
      use_cost_bound = False
    else:
      use_cost_bound = True
  else:
    use_cost_bound = use_bound

  agent_num = args['robot_num']
  _ = f1.readline()
  datapat = re.compile(b'(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\n')

  sx_list = []
  sy_list = []
  gx_list = []
  gy_list = []

  for i in range(agent_num):
    data = f1.readline()
    match = datapat.match(data)
    sx_list.append(int(match.group(5)))
    sy_list.append(int(match.group(6)))
    gx_list.append(int(match.group(7)))
    gy_list.append(int(match.group(8)))

  grids = np.zeros((16, 16), dtype=np.int)

  sx = np.array(sx_list)  # start x = column in grid image
  sy = np.array(sy_list)  # start y = rows in grid image, the kth component corresponds to the kth robot.
  gx = np.array(gx_list)  # goal x
  gy = np.array(gy_list)  # goal y

  cgrids = [cost_grids[0], cost_grids[1]]
  cvecs = np.ones((agent_num, 2))
  clist = args['cost_name']
  cdim = len(clist)

  ##################################################################
  #### choose one of the planner to run by uncommenting the code ###
  ##################################################################

  ### Invoke MO-CBS planner ###
  success, res_path, res_cost, time_res, open_list_res, close_list_res, low_level_time, low_level_calls=mocbs.RunMocbsMAPF(grids,
                              sx, sy, gx, gy, cvecs, cgrids, cdim, np.inf, 1500, clist, expansion_mode=0, use_cost_bound=use_cost_bound)

  #### Invoke NAMOA* planner ###
  # res = moastar.RunMoAstarMAPF(grids, sx, sy, gx, gy, cvecs, cgrids, cdim, 1.0, 0.0, np.inf, 100)

  #### Invoke MOM* planner ###
  # res = momstar.RunMoMstarMAPF(grids, sx, sy, gx, gy, cvecs, cgrids, cdim, 1.0, 0.0, np.inf, 10)

  # print(success)
  print("Paretal Optimal Paths Number:", len(res_cost))
  print(res_cost)
  # print(open_list_res)
  print("Number of close list:", close_list_res)
  print("Number of low level calls:", low_level_calls)
  print("Total Low Level Time:", low_level_time)
  print("Total Time:", time_res)
  # print(res_path)

  # df = pd.DataFrame({'success': [], "res_num": [], "close_list_num": [], "low_level_num": [], "time": []})
  # df.to_csv("./benchmark/empty-16-16-result/plot_{}_{}.csv".format(index, int(use_cost_bound)), index=False, sep=',')
  #
  # data = [success, len(res_cost), close_list_res, low_level_calls, time_res]
  # df = pd.read_csv('./benchmark/empty-16-16-result/plot_{}_{}.csv'.format(index, int(use_cost_bound)))
  # df.loc[1] = data
  # df.to_csv("./benchmark/empty-16-16-result/plot_{}_{}.csv".format(index, int(use_cost_bound)), index=False, sep=',')
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