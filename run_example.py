import context
import time
import re

import numpy as np
from numpy import random
import matplotlib.pyplot as plt

import common 
import mocbs
import moastar # NAMOA*
import momstar


def RunToyExample():
  """
  Run different scenarios
  """
  f = open(file='benchmark/empty-16-16/empty-16-16-random-2.scen', mode='rb')
  cost_grids = np.load(file='benchmark/empty-16-16/16-16-random-matrix.npy')

  agent_num = 4
  _ = f.readline()
  datapat = re.compile(b'(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\t(.*)\n')

  sx_list = []
  sy_list = []
  gx_list = []
  gy_list = []

  for i in range(agent_num):
    data = f.readline()
    match = datapat.match(data)
    sx_list.append(int(match.group(5)))
    sy_list.append(int(match.group(6)))
    gx_list.append(int(match.group(7)))
    gy_list.append(int(match.group(8)))

  grids = np.zeros((16, 16))

  sx = np.array(sx_list)  # start x = column in grid image
  sy = np.array(sy_list)  # start y = rows in grid image, the kth component corresponds to the kth robot.
  gy = np.array(gy_list)  # goal y
  gx = np.array(gx_list)  # goal x

  cvecs = np.ones((agent_num, 2))
  cgrids = [cost_grids[2], cost_grids[3]] # the mth component corresponds to the mth objective.
  # cost for agent-i to go through an edge c[m] = cvecs[i][m] * cgrids[m][vy,vx], where vx,vy are the target node of the edge.

  cdim = len(cvecs[0])

  ##################################################################
  #### choose one of the planner to run by uncommenting the code ###
  ##################################################################

  ### Invoke MO-CBS planner ###
  res_path, res_cost, time_res, open_list_res, close_list_res, low_level_time, low_level_calls=mocbs.RunMocbsMAPF(grids,
                              sx, sy, gx, gy, cvecs, cgrids, cdim, np.inf, 2000, expansion_mode=0, use_cost_bound=False)

  #### Invoke NAMOA* planner ###
  # res = moastar.RunMoAstarMAPF(grids, sx, sy, gx, gy, cvecs, cgrids, cdim, 1.0, 0.0, np.inf, 100)

  #### Invoke MOM* planner ###
  # res = momstar.RunMoMstarMAPF(grids, sx, sy, gx, gy, cvecs, cgrids, cdim, 1.0, 0.0, np.inf, 10)

  # print(res_path)
  print(len(res_cost))
  print(open_list_res)
  print(close_list_res)
  print(low_level_calls)
  print(low_level_time)
  print(time_res)

  return


def main():
  RunToyExample()
  return


if __name__ == '__main__':
  print("begin of main")
  main()
  print("end of main")