"""
Author: Zhongqiang (Richard) Ren
Version@2021
Remark: some of the code is redundant and needs a clean up.
"""
import numpy as np
import copy
import time
import sys

import common as cm
import moastar
import mostastar as mosta
import itertools as itt
from copy import deepcopy
from ll_solver import LLSolver

######
MOCBS_INIT_SIZE_LIMIT = 800*1000
OPEN_ADD_MODE = 2
######

def ReturnCost(element):
  return element[2]

def EnforceUnitTimePath(lv,lt):
  """
  Given a path (without the final node with infinite timestamp), 
   insert missing (v,t) to ensure every pair of adjacent (v,t) 
   has time difference of one.
  """
  dt = 1
  nlv = list()
  nlt = list()
  for ix in range(len(lt)-1):
    nlv.append(lv[ix])
    nlt.append(lt[ix])
    if lt[ix+1]-lt[ix] > 1.001:
      ct = lt[ix]
      while lt[ix+1] - ct > 1.001:
        nlv.append(lv[ix])
        nlt.append(ct+1)
        ct = ct + 1
  # end for
  nlv.append(lv[-1])
  nlt.append(lt[-1])
  
  # Change operation in class MocbsSol to this function
  nlv.append(nlv[-1])
  nlt.append(np.inf)
  return nlv, nlt
  
class MocbsConstraint:
  """
  MocbsConstraint
  """
  def __init__(self, i, va,vb, ta,tb, j=-1, flag=-1):
    """
    create a constraint, if a single point, then va=vb
    """
    self.i = i # i<0, iff not valid
    self.va = va
    self.vb = vb
    self.ta = ta
    self.tb = tb
    self.j = j # undefined by default, this is used for MA-CBS
    self.flag = flag # flag = 1, vertex conflict, flag = 2 swap conflict

  def __str__(self):
    return "{i:"+str(self.i)+",va:"+str(self.va)+",vb:"+str(self.vb)+\
      ",ta:"+str(self.ta)+",tb:"+str(self.tb)+",j:"+str(self.j)+",flag:"+str(self.flag)+"}"

class MocbsSol:
  """
  The solution in CBS high level node. A dict of paths for all robots.
  """
  def __init__(self):
    self.paths = dict()
    return

  def __str__(self):
    return str(self.paths)

  def AddPath(self, i, lv, lt):
    """
    lv is a list of loc id
    lt is a list of time (adjacent element increases with a step size of one)
    """
    # add a final infinity interval
    nlv,nlt = EnforceUnitTimePath(lv,lt)
    self.paths[i] = [nlv,nlt]
    return 

  def DelPath(self, i):
    self.paths.pop(i)
    return

  def GetPath(self, i):
    return self.paths[i]

  def CheckConflict(self, i,j):
    ix = 0
    while ix < len(self.paths[i][1])-1:
      for jx in range(len(self.paths[j][1])-1):
        jtb = self.paths[j][1][jx+1]
        jta = self.paths[j][1][jx]
        itb = self.paths[i][1][ix+1]
        ita = self.paths[i][1][ix]
        iva = self.paths[i][0][ix]
        ivb = self.paths[i][0][ix+1]
        jva = self.paths[j][0][jx]
        jvb = self.paths[j][0][jx+1]
        overlaps, t_lb, t_ub = cm.ItvOverlap(ita,itb,jta,jtb)
        if not overlaps:
          continue
        if ivb == jvb: # vertex conflict at ivb (=jvb)
          return [MocbsConstraint(i, ivb, ivb, t_lb+1, t_lb+1, j, 1), MocbsConstraint(j, jvb, jvb, t_lb+1, t_lb+1, i, 1)] # t_ub might be inf?
          # use min(itb,jtb) to avoid infinity
        if (ivb == jva) and (iva == jvb): # swap location
          return [MocbsConstraint(i, iva, ivb, t_lb, t_lb+1, j, 2), MocbsConstraint(j, jva, jvb, t_lb, t_lb+1, i, 2)]
      ix = ix + 1
    return []

class MocbsNode:
  """
  High level search tree node
  """
  def __init__(self, id0, cvec, num_robots, true_low_bound=None, upper_bound=None, sol=MocbsSol(), cstr=MocbsConstraint(-1,-1,-1,-1,-1,-1), parent=-1):
    """
    id = id of this high level CT node
    sol = an object of type CCbsSol.
    cstr = a list of constraints, either empty or of length 2.
      newly added constraint in this node, to get all constraints, 
      need to backtrack from this node down to the root node.
    parent = id of the parent node of this node.
    """
    self.id = id0
    self.sol = sol
    self.cstr = cstr
    self.cvec = cvec
    self.parent = -1 
    self.root = -1 # to which tree it belongs

    ## This is the added upper bound and lower bound
    if (true_low_bound != None):
      self.true_lower_bound = true_low_bound
    else:
      self.true_lower_bound = np.zeros(num_robots)
    if (upper_bound != None):
      self.upper_bound = upper_bound
    else:
      self.upper_bound = np.ones(num_robots) * np.inf
    return

  def __str__(self):
    str1 = "{id:"+str(self.id)+",cvec:"+str(self.cvec)+",par:"+str(self.parent)
    return str1+",cstr:"+str(self.cstr)+",sol:"+str(self.sol)+",true_lower_bound:"+str(self.true_lower_bound)+",upper_bound:"+str(self.upper_bound)+"}"

  def CheckConflict(self):
    """
    check for conflicts along paths of all pairs of robots.
    record the first one conflict.
    Notice that one conflict should be splitted to 2 constraints.
    """
    done_set = set()
    for k1 in self.sol.paths:
      for k2 in self.sol.paths:
        if k2 in done_set or k2 == k1:
          continue
        # check for collision
        res = self.sol.CheckConflict(k1,k2)
        if len(res) > 0:
          return res
      # end for k2
      done_set.add(k1) # auxiliary
    return [] # no conflict

class MocbsSearch:
  """
  """
  def __init__(self, G, sx_list, sy_list, gx_list, gy_list, cvecs, expansion_mode, time_limit, use_cost_bound=False):
    """
    arg grids is a 2d static grid.
    """
    ## Instance Related args
    self.grids = G.map
    (self.yd, self.xd) = self.grids.shape
    self.sx_list = deepcopy(sx_list)
    self.sy_list = deepcopy(sy_list)
    self.gx_list = deepcopy(gx_list)
    self.gy_list = deepcopy(gy_list)
    self.num_robots = len(sx_list)
    self.cdim = G.num_objective
    self.cvecs = copy.deepcopy(cvecs) # for multi-dimensional cost.
    self.cgrids = G.cost_grids # for multi-dimensional cost.
    self.expansion_mode = expansion_mode
    self.time_limit = time_limit
    self.clist = G.cost_list
    self.ll_starter_list = [LLSolver(G, sy_list[i] * self.xd + sx_list[i], gy_list[i] * self.xd + self.gx_list[i]) for i in range(self.num_robots)]

    # Search Related args
    self.nodes = dict() # high level nodes
    self.open_list = cm.PrioritySet()
    self.open_by_tree = dict()
    self.closed_set = set()
    self.num_closed_low_level_states = 0
    self.num_low_level_calls = 0
    self.total_low_level_time = 0
    self.node_id_gen = 1
    self.num_roots = 0
    self.curr_root = -1
    self.root_generated = 0
    self.nondom_goal_nodes = set() # a set of HL nodes that reaches goal with non-dom cost vec.

    # Cost Bound Related
    self.use_cost_bound = use_cost_bound
    return

  def Search(self, search_limit):
    """
    high level search
    """
    ########################################################
    #### init search ####
    ########################################################
    search_complete = True
    self.tstart = time.perf_counter()
    if (self.expansion_mode == 0 or self.expansion_mode == 1):
      init_success = self.InitSearch()
    else:
      init_success = self.InitSearch_OnDemand()

    find_first_feasible_sol = False
    first_sol = []
    first_sol_gvec = []

    if init_success == 0:
      search_complete = False
      output_res = (int(len(self.closed_set)), dict(), 0, \
                    float(time.perf_counter() - self.tstart), 0, int(self.num_roots), \
                    int(self.open_list.size()), find_first_feasible_sol, first_sol_gvec,
                    float(self.total_low_level_time), int(self.num_low_level_calls))
      return False, dict(), output_res, None, None, None, None, None

    best_g_value = -1
    reached_goal_id = -1

    # the following few lines is useful when self.expansion_mode == 1, init self.curr_root
    for k in self.open_by_tree:
      self.curr_root = k
      break

    ########################################################
    #### init search, END ####
    ########################################################
    while True:
      tnow = time.perf_counter()
      rd = len(self.closed_set)
      if (rd > search_limit) or (tnow - self.tstart > self.time_limit):
        search_complete = False
        break
      ########################################################
      #### pop a non-dominated from OPEN ####
      ########################################################
      if (self.expansion_mode == 0 or self.expansion_mode == 1):
        pop_succeed, popped = self.SelectNode()
      else:
        pop_succeed, popped = self.SelectNode_OnDemand()

      if not pop_succeed:
        break
      if self.GoalFilterNode(popped[1]):
        continue
      ########################################################
      #### pop a non-dominated from OPEN, END ####
      ########################################################

      ########################################################
      #### Expand a node ####
      ########################################################
      self.closed_set.add(popped[1])  # only used to count numbers
      curr_node = self.nodes[popped[1]]
      cstrs = self.FirstConflict(curr_node)

      if len(cstrs) == 0:  # no conflict, find a sol !!
        if not find_first_feasible_sol:
          find_first_feasible_sol = True
          first_sol = copy.deepcopy(curr_node)
          first_sol_gvec = curr_node.cvec
        self.RefineNondomGoals(curr_node.id)
        self.nondom_goal_nodes.add(curr_node.id)
        continue  # keep going

      for cstr in cstrs:
        new_id = self.node_id_gen
        self.node_id_gen = self.node_id_gen + 1
        self.nodes[new_id] = copy.deepcopy(curr_node)
        self.nodes[new_id].id = new_id
        self.nodes[new_id].parent = curr_node.id
        self.nodes[new_id].cstr = cstr
        sstats = self.Lsearch(new_id)  # this can generate multiple nodes
        self.UpdateStats(sstats)
        if sstats[2] == 0:
          # this branch fails, robot ri cannot find a consistent path.
          continue
        # node insertion into OPEN is done in Lsearch()
      ########################################################
      #### Expand a node, END ####
      ########################################################
      # end of for
    # end of while

    all_path_set = dict()
    all_cost_vec = dict()
    for nid in self.nondom_goal_nodes:
      hnode = self.nodes[nid]
      all_path_set[hnode.id] = hnode.sol.paths
      all_cost_vec[hnode.id] = hnode.cvec
    if (self.open_list.size() == 0) and (len(self.nondom_goal_nodes) > 0) and (self.root_generated == self.num_roots):
      search_success = True

    # output_res = (int(len(self.closed_set)), all_cost_vec, int(search_success), \
    #               float(time.perf_counter() - self.tstart), int(self.num_closed_low_level_states), \
    #               int(self.num_roots), int(self.open_list.size()), find_first_feasible_sol, first_sol_gvec,
    #               float(self.total_low_level_time), int(self.num_low_level_calls))
    time_res = round(time.perf_counter() - self.tstart)
    open_list_res = self.open_list.size()
    close_list_res = len(self.closed_set)
    low_level_time = round(self.total_low_level_time)
    low_level_calls = self.num_low_level_calls

    return search_complete, all_path_set, all_cost_vec, time_res, open_list_res, close_list_res, low_level_time, low_level_calls

  def InitSearch(self):
    """
    called at the beginning of the search.
    generate first High level node.
    compute individual optimal path for each robot.
    """
    self.pareto_idvl_path_dict = dict()
    for ri in range(self.num_robots):
      tnow = time.perf_counter()
      time_left = self.time_limit - (tnow - self.tstart)

      single_pareto_path, others = self.ll_starter_list[ri].find_path()
      self.pareto_idvl_path_dict[ri] = list()
      for path in single_pareto_path:
        lv = path[0]
        lt = path[1]
        nlv, nlt = EnforceUnitTimePath(lv, lt)
        new_dict = [nlv, nlt, path[2][0], 0, path[2]]
        self.pareto_idvl_path_dict[ri].append(new_dict)

      if self.use_cost_bound:
        # self.ComputePathCost(self.pareto_idvl_path_dict[ri], ri)
        self.pareto_idvl_path_dict[ri].sort(key=ReturnCost)
        for ind in range(len(self.pareto_idvl_path_dict[ri])):
          if ind == (len(self.pareto_idvl_path_dict[ri]) - 1):
            self.pareto_idvl_path_dict[ri][ind][3] = np.inf
          else:
            self.pareto_idvl_path_dict[ri][ind][3] = self.pareto_idvl_path_dict[ri][ind + 1][2]

      tnow = time.perf_counter()
      if (tnow - self.tstart > self.time_limit):
        print(" FAIL! timeout! ")
        return 0
    # end for

    # for too many root nodes, just terminates...
    init_size = 1
    for k in self.pareto_idvl_path_dict:
      init_size = init_size * len(self.pareto_idvl_path_dict[k])

    print("Initialization size:", init_size)
    if (init_size > MOCBS_INIT_SIZE_LIMIT):
      print("[CAVEAT] Too many roots to be generated for MO-CBS. Terminate. (why not use MO-CBS-t?)")
      self.num_roots = init_size
      return 0
    self.num_roots = init_size

    all_combi = list(itt.product(*(self.pareto_idvl_path_dict[ky] for ky in self.pareto_idvl_path_dict)))

    for jpath in all_combi:
      nid = self.node_id_gen
      self.nodes[nid] = copy.deepcopy(MocbsNode(nid, np.zeros(self.cdim), self.num_robots))
      self.nodes[nid].root = nid
      self.node_id_gen = self.node_id_gen + 1
      for ri in range(len(jpath)):
        self.nodes[nid].sol.paths[ri] = [jpath[ri][0], jpath[ri][1], jpath[ri][4]]
        if self.use_cost_bound:
          self.nodes[nid].true_lower_bound[ri] = jpath[ri][2]
          self.nodes[nid].upper_bound[ri] = jpath[ri][3]
      cvec = self.ComputeNodeCostObject(self.nodes[nid])  # update node cost vec and return cost vec

      self.open_by_tree[nid] = cm.PrioritySet()

      if OPEN_ADD_MODE == 1:
        self.open_list.add(np.sum(cvec), nid)
        self.open_by_tree[nid].add(np.sum(cvec), nid)
      elif OPEN_ADD_MODE == 2:
        self.open_list.add(tuple(cvec), nid)
        self.open_by_tree[nid].add(tuple(cvec), nid)

    print("Finish Initialization")
    return 1

  def ComputeNodeCostObject(self, nd):
    """
    Given a high level search node, compute the cost of paths in that node.
    """
    out_cost = np.zeros(self.cdim) # init M-dim cost vector
    for idx in nd.sol.paths:
      out_cost += np.array(nd.sol.paths[idx][2])
    nd.cvec = out_cost # update cost in that node
    return out_cost

  def BacktrackCstrs(self, nid):
    """
    given a node, trace back to the root, find all constraints relavant.
    """
    node_cs = list()
    swap_cs = list()
    cid = nid
    ri = self.nodes[nid].cstr.i
    while cid != -1:
      if self.nodes[cid].cstr.i == ri: # not a valid constraint
        # init call of mocbs will not enter this.
        cstr = self.nodes[cid].cstr
        if self.nodes[cid].cstr.flag == 1: # vertex constraint
          node_cs.append( (cstr.vb, cstr.tb) )
        elif self.nodes[cid].cstr.flag == 2: # edge constraint
          swap_cs.append( (cstr.va, cstr.vb, cstr.ta) )
          # node_cs.append( (cstr.va, cstr.tb) ) # since another robot is coming to v=va at t=tb
      cid = self.nodes[cid].parent
    return node_cs, swap_cs

  def PreProcess(self, node_cs, swap_cs):
    swap_dict = dict()
    for constraint in swap_cs:
      if constraint[0] not in swap_dict:
        swap_dict[constraint[0]] = dict()
      if constraint[2] not in swap_dict[constraint[0]]:
        swap_dict[constraint[0]][constraint[2]] = set()
      swap_dict[constraint[0]][constraint[2]].add(constraint[1])

    node_dict = dict()
    for constraint in node_cs:
      if constraint[0] not in node_dict:
        node_dict[constraint[0]] = set()
      node_dict[constraint[0]].add(constraint[1])
    return node_dict, swap_dict

  def LsearchPlanner(self, ri, node_cs, swap_cs, cost_upper_bound=np.inf):
    path_dict, lsearch_stats = mosta.RunMoSTAstar(self.grids, self.sx_list[ri], self.sy_list[ri], self.gx_list[ri], self.gy_list[ri], \
      self.cvecs[ri], self.cgrids, self.cdim, 1.0, 0.0, np.inf, self.time_limit-(time.perf_counter()-self.tstart), self.clist, False, node_cs, swap_cs, cost_upper_bound)
    return path_dict, lsearch_stats

  def Lsearch(self, nid):
    """
    low level search
    """
    nd = self.nodes[nid]
    ri = nd.cstr.i
    node_cs, swap_cs = self.BacktrackCstrs(nid)
    true_lower_bound = nd.true_lower_bound[ri]
    upper_bound = nd.upper_bound[ri]

    # call constrained NAMOA*

    node_dict, swap_dict = self.PreProcess(node_cs, swap_cs)
    path_list, lsearch_stats = self.ll_starter_list[ri].find_path(node_dict, swap_dict, upper_bound)
    ct = 0 # count of node generated

    # The added part and find cost of each path
    new_path_list = []
    for k in range(len(path_list)):
      nlv, nlt = EnforceUnitTimePath(path_list[k][0], path_list[k][1])
      new_path_list.append([nlv, nlt, path_list[k][2][0], path_list[k][2]])

    if self.use_cost_bound:
      new_path_list.sort(key=ReturnCost)


    path_list = new_path_list
    cost_list = []
    for i in new_path_list:
      cost_list.append(i[2])
    print(cost_list, true_lower_bound, upper_bound)

    for k in range(len(path_list)): # loop over all individual Pareto paths
      if self.use_cost_bound:
        if (path_list[k][2] >= upper_bound):
          break
        elif (k == len(path_list) - 1):
          new_nd = copy.deepcopy(self.nodes[nid])
          new_nd.upper_bound[ri] = upper_bound
          new_nd.true_lower_bound[ri] = max([true_lower_bound, path_list[k][2]])
          # print((new_nd.upper_bound[ri], new_nd.true_lower_bound[ri]))
        elif path_list[k][2] > true_lower_bound:
          new_nd = copy.deepcopy(self.nodes[nid])
          new_nd.true_lower_bound[ri] = path_list[k][2]
          new_nd.upper_bound[ri] = min([upper_bound, path_list[k+1][2]])
          # print((new_nd.upper_bound[ri], new_nd.true_lower_bound[ri]))
        elif path_list[k][2] <= true_lower_bound:
          if path_list[k+1][2] > true_lower_bound:
            new_nd = copy.deepcopy(self.nodes[nid])
            new_nd.upper_bound[ri] = min([upper_bound, path_list[k+1][2]])
            new_nd.true_lower_bound[ri] = true_lower_bound
            # print((new_nd.upper_bound[ri], new_nd.true_lower_bound[ri]))
          else:
            continue
      else:
        new_nd = copy.deepcopy(self.nodes[nid])

      new_nd.sol.DelPath(ri)
      new_nd.sol.paths[ri] = [path_list[k][0], path_list[k][1], path_list[k][3]]
      new_nd.cvec = self.ComputeNodeCostObject(new_nd)

      if self.GoalFilterNodeObject(new_nd):
        continue # skip this dominated node

      # a non-dom node, add to OPEN
      new_id = new_nd.id # the first node, self.nodes[nid] is ok
      if ct > 0: # generate a new node, assign a new id
        new_id = self.node_id_gen
        self.node_id_gen = self.node_id_gen + 1
        new_nd.id = new_id
      self.nodes[new_id] = new_nd # add to self.nodes

      ### ADD OPEN BEGIN
      if OPEN_ADD_MODE == 1:
        self.open_list.add(np.sum(new_nd.cvec), new_nd.id) # add to OPEN
        self.open_by_tree[new_nd.root].add(np.sum(new_nd.cvec), new_nd.id) # add to OPEN in the search tree it belongs to
      elif OPEN_ADD_MODE == 2:
        self.open_list.add(tuple(new_nd.cvec), new_nd.id) # add to OPEN
        self.open_by_tree[new_nd.root].add(tuple(new_nd.cvec), new_nd.id) # add to OPEN in the search tree it belongs to
      ### ADD OPEN END

      ct = ct + 1 # count increase
    return lsearch_stats

  def GoalFilterNode(self,nid):
    """
    filter HL node nid, if self.nodes[nid] is dominated by any HL nodes that reached goal.
    """
    for fid in self.nondom_goal_nodes:
      if cm.DomOrEqual( self.nodes[fid].cvec, self.nodes[nid].cvec ):
        return True
    return False # not filtered

  def GoalFilterNodeObject(self,hnode):
    """
    filter HL node hnode, if self.nodes hnode is dominated by any HL nodes that reached goal.
    """
    for fid in self.nondom_goal_nodes:
      if cm.DomOrEqual( self.nodes[fid].cvec, hnode.cvec ):
        return True
    return False # not filtered

  def RefineNondomGoals(self,nid): # should never be called ???
    """
    nid is a new HL node that reaches goal. Use it to filter self.nondom_goal_nodes
    """
    temp_set = copy.deepcopy(self.nondom_goal_nodes)
    for fid in self.nondom_goal_nodes:
      if cm.DomOrEqual( self.nodes[nid].cvec, self.nodes[fid].cvec):
        temp_set.remove(fid)
    self.nondom_goal_nodes = temp_set
    return

  def PathDictLexSort(self, path_dict, cvec_dict):
    """
    sort path_dict by lex order and return a list.
    """
    out_path_list = list()
    pq = cm.PrioritySet()
    for k in cvec_dict:
      pq.add( tuple( cvec_dict[k] ), k)
    while pq.size() > 0:
      cvec, k = pq.pop()
      out_path_list.append(path_dict[k])
    return out_path_list

  def InitSearch_OnDemand(self):
    """
    called at the beginning of the search. 
    generate first High level node.
    compute individual optimal path for each robot.
    """
    self.pareto_idvl_path_dict = dict()
    for ri in range(self.num_robots):
      tnow = time.perf_counter()

      time_left = self.time_limit - (tnow-self.tstart)

      # call constrained NAMOA*
      single_pareto_path, others = moastar.RunMoAstarSingleAgent(self.grids, self.sx_list[ri], self.sy_list[ri], \
        self.gx_list[ri], self.gy_list[ri], self.cvecs[ri], self.cgrids, self.cdim, \
        1.0, 0.0, 1e10, time_left )

      self.pareto_idvl_path_dict[ri] = list()

      for ref_key in single_pareto_path:
        lv = list()
        lt = list()
        curr_time = 0
        for pt in single_pareto_path[ref_key]:
          lv.append(pt[0]) # pt is a tuple of 1 element, e.g. (34,)
          lt.append(curr_time)
          curr_time = curr_time + 1
        self.pareto_idvl_path_dict[ri].append( (lv,lt) )
        
      tnow = time.perf_counter()
      if (tnow - self.tstart > self.time_limit):
        print(" FAIL! timeout! ")
        return 0
    # end for

    # update number of roots.
    init_size = 1
    for k in self.pareto_idvl_path_dict:
      init_size = init_size*len(self.pareto_idvl_path_dict[k])
    self.num_roots = init_size

    # init indices 
    self.init_tree_index_dict = dict()
    for ri in range(self.num_robots):
      self.init_tree_index_dict[ri] = 0

    # generate a root
    if not self.GenRoot_OnDemand():
      return 2 # this does not matter, if fail to generate, then OPEN depletes and the program terminates.

    self.UpdateIndices_OnDemand()
    return 1

  def GenRoot_OnDemand(self):

    if (self.init_tree_index_dict[self.num_robots-1] >= len(self.pareto_idvl_path_dict[self.num_robots-1]) ):
      return False, -1

    nid = self.node_id_gen
    self.nodes[nid] = copy.deepcopy(MocbsNode(nid, np.zeros(self.cdim), self.num_robots))
    self.nodes[nid].root = nid
    self.node_id_gen = self.node_id_gen + 1
    self.root_generated = self.root_generated + 1 
    
    for ri in range(self.num_robots):
      ri_path = self.pareto_idvl_path_dict[ri][self.init_tree_index_dict[ri]]
      self.nodes[nid].sol.AddPath(ri, ri_path[0], ri_path[1])
    cvec = self.ComputeNodeCostObject(self.nodes[nid]) # update node cost vec and return cost vec

    self.open_by_tree[nid] = cm.PrioritySet()
    if OPEN_ADD_MODE == 1:
      self.open_list.add(np.sum(cvec),nid)
      self.open_by_tree[nid].add(np.sum(cvec), nid)
    elif OPEN_ADD_MODE == 2:
      self.open_list.add(tuple(cvec),nid)
      self.open_by_tree[nid].add(tuple(cvec), nid)

    return True, nid

  def UpdateIndices_OnDemand(self):
    self.init_tree_index_dict[0] = self.init_tree_index_dict[0] + 1
    idx = 0
    while (idx < (self.num_robots-1)) and \
      (self.init_tree_index_dict[idx] >= len(self.pareto_idvl_path_dict[idx])):
      # add one to the "next digit"
      self.init_tree_index_dict[idx] = 0
      self.init_tree_index_dict[idx+1] = self.init_tree_index_dict[idx+1] + 1
      idx = idx + 1
    return 

  def UpdateStats(self, stats):
    self.num_closed_low_level_states = self.num_closed_low_level_states + stats[0]
    self.total_low_level_time = self.total_low_level_time + stats[3]
    self.num_low_level_calls = self.num_low_level_calls + 1
    return

  def ReconstructPath(self, nid):
    """
    """
    path_set = dict()
    for i in range(self.num_robots):
      lx = list()
      ly = list()
      lv = self.nodes[nid].sol.GetPath(i)[0]
      for v in lv:
        y = int(np.floor(v / self.xd))
        x = int(v % self.xd)
        ly.append(y)
        lx.append(x)
      lt = self.nodes[nid].sol.GetPath(i)[1]
      path_set[i] = [lx,ly,lt]
    return path_set

  def FirstConflict(self, nd):
    return nd.CheckConflict()

  def SelectNode(self):
    """
    Pop a node from OPEN. 
    consider self.expansion_mode, either MO-CBS, or MO-CBS-t.
    """
  
    if self.expansion_mode == 0 and self.open_list.size() == 0:
      return False, []
  
    if self.open_by_tree[self.curr_root].size() == 0:
      self.open_by_tree.pop(self.curr_root) # delete current tree 
      for k in self.open_by_tree: # move to next tree
        self.curr_root = k
        break
    if len(self.open_by_tree) == 0: # if all trees are depleted
      return False, []

    popped = (-1,-1) 
    if (self.expansion_mode == 1 or self.expansion_mode == 2):
      popped = self.open_by_tree[self.curr_root].pop()
      self.open_list.remove(popped[1])
    else:
      popped = self.open_list.pop() # pop_node = (f-value, high-level-node-id)

    print(popped)
    return True, popped

  def SelectNode_OnDemand(self):
    """
    Pop a node from OPEN. 
    consider self.expansion_mode=2, for MO-CBS-t on demand generation of roots.
    """
    if self.open_by_tree[self.curr_root].size() == 0:
      print(">>> MO-CBS-t, Move to next tree or terminate...")

      # generate a root
      genroot_success, root_id = self.GenRoot_OnDemand()
    
      if not genroot_success:
        return False, [] # fail to generate new node
      self.UpdateIndices_OnDemand()

      # delete old tree and move to newly generated tree
      self.open_by_tree.pop(self.curr_root) # delete curr tree 
      for k in self.open_by_tree: # move to next tree
        self.curr_root = k
        break

    # pop a node from current tree
    popped = (-1,-1) 
    popped = self.open_by_tree[self.curr_root].pop()
    self.open_list.remove(popped[1])

    return True, popped


def RunMocbsMAPF(G, sx, sy, gx, gy, cvecs, cdim, search_limit, time_limit, expansion_mode=2, use_cost_bound=False):
  if expansion_mode == 2:
    print("... Run MO-CBS-t ... ")
  elif expansion_mode == 0:
    print("... Run MO-CBS ... ")
  else:
    sys.exit("[ERROR] Are you kidding? Unknown expansion mode for MO-CBS!!!")

  truncated_cvecs = list()
  truncated_cgrids = list()

  for idx in range(len(cvecs)):
    truncated_cvecs.append(cvecs[idx][0:cdim])
  for idx in range(cdim):
    truncated_cgrids.append(G.cost_grids[idx])

  mocbs = MocbsSearch(G, sx, sy, gx, gy, truncated_cvecs, expansion_mode, time_limit, use_cost_bound=use_cost_bound)

  return mocbs.Search(search_limit)