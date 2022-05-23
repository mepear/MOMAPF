import numpy as np
import copy
import time

import common as cm
import itertools as itt
from copy import deepcopy
from ll_solver import LLSolver
from utils import CostBound, gen_splitting

######
MOCBS_INIT_SIZE_LIMIT = 800*1000
OPEN_ADD_MODE = 2
######

class MocbsConstraint:
  """
  MocbsConstraint
  """
  def __init__(self, i, va,vb, ta,tb, j=-1, flag=-1):
    """
    create a constraint, if a single point, then va=vb
    """
    self.i = i # i<0, iff not valid.
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

  def AddPath(self, i, path):
    # add a final infinity interval
    self.paths[i] = path
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
  def __init__(self, id0, cvec, num_robots, sol=MocbsSol(), cstr=MocbsConstraint(-1,-1,-1,-1,-1,-1), parent=-1):
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
    self.cost_bound_list = [CostBound(tuple(0 for _ in range(len(cvec))), [tuple(np.inf for _ in range(len(cvec)))]) for _ in range(num_robots)]
    self.parent = -1
    self.root = -1 # to which tree it belongs

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
  def __init__(self, G, sx_list, sy_list, gx_list, gy_list, time_limit, use_cost_bound=False):
    """
    G contains all information about the raw graph.
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
    self.cgrids = G.cost_grids # for multi-dimensional cost.
    self.time_limit = time_limit
    self.clist = G.cost_list
    self.ll_starter_list = [LLSolver(G, sy_list[i] * self.xd + sx_list[i], gy_list[i] * self.xd + self.gx_list[i]) for i in range(self.num_robots)]

    # Search Related args
    self.nodes = dict() # high level nodes
    self.open_list = cm.PrioritySet()
    self.closed_set = set()
    self.num_closed_low_level_states = 0
    self.num_low_level_calls = 0
    self.generated_node = 0
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
    init_success = self.InitSearch()

    find_first_feasible_sol = False
    first_sol = []
    first_sol_gvec = []

    if init_success == 0:
      return False, None, None, None

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
      pop_succeed, popped = self.SelectNode()

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

    time_res = round(time.perf_counter() - self.tstart)
    open_list_res = self.open_list.size()
    close_list_res = len(self.closed_set)
    low_level_time = round(self.total_low_level_time)
    low_level_calls = self.num_low_level_calls
    branch_factor = float(self.generated_node) / float(self.num_low_level_calls)

    result_dict = {'time': time_res, 'closed_num': close_list_res, 'low_level_time': low_level_time,
                   'low_level_calls': low_level_calls, 'branch_factor': branch_factor}

    return search_complete, all_path_set, all_cost_vec, result_dict

  def InitSearch(self):
    """
    called at the beginning of the search.
    generate first High level node.
    compute individual optimal path for each robot.
    """
    self.pareto_idvl_path_dict = dict()
    for ri in range(self.num_robots):

      single_pareto_path, _ = self.ll_starter_list[ri].find_path([tuple(np.inf for _ in range(self.cdim))])
      self.pareto_idvl_path_dict[ri] = single_pareto_path

      if self.use_cost_bound:
        trivial_lb = tuple(0 for _ in range(self.cdim))
        trivial_ub = [tuple(np.inf for _ in range(self.cdim))]
        self.pareto_idvl_path_dict[ri] = gen_splitting(trivial_lb, trivial_ub, single_pareto_path)

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
      self.nodes[nid] = copy.deepcopy(MocbsNode(nid, tuple(0 for _ in range(self.cdim)), self.num_robots))
      self.nodes[nid].root = nid
      self.node_id_gen = self.node_id_gen + 1
      for ri in range(self.num_robots):
        self.nodes[nid].sol.paths[ri] = [jpath[ri][1][0], jpath[ri][1][1], jpath[ri][0]]
        if self.use_cost_bound:
          self.nodes[nid].cost_bound_list[ri] = CostBound(jpath[ri][2], jpath[ri][3])
      cvec = self.ComputeNodeCostObject(self.nodes[nid])  # update node cost vec and return cost vec

      if OPEN_ADD_MODE == 1:
        self.open_list.add(np.sum(cvec), nid)
      elif OPEN_ADD_MODE == 2:
        self.open_list.add(tuple(cvec), nid)

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
      cid = self.nodes[cid].parent
    return node_cs, swap_cs

  def PreProcess(self, node_cs, swap_cs):
    swap_dict = dict()
    max_timestep = -1
    for constraint in swap_cs:
      if constraint[0] not in swap_dict:
        swap_dict[constraint[0]] = dict()
      if constraint[2] not in swap_dict[constraint[0]]:
        swap_dict[constraint[0]][constraint[2]] = set()
      swap_dict[constraint[0]][constraint[2]].add(constraint[1])
      max_timestep = max(constraint[2], max_timestep)

    node_dict = dict()
    for constraint in node_cs:
      if constraint[0] not in node_dict:
        node_dict[constraint[0]] = set()
      node_dict[constraint[0]].add(constraint[1])
      max_timestep = max(constraint[1], max_timestep)
    return node_dict, swap_dict, max_timestep

  def Lsearch(self, nid):
    """
    low level search
    """
    nd = self.nodes[nid]
    ri = nd.cstr.i
    node_cs, swap_cs = self.BacktrackCstrs(nid)
    lower_bound = nd.cost_bound_list[ri].lb
    upper_bound = nd.cost_bound_list[ri].ub

    # call MOA* and use upper bound to do pruning
    node_dict, swap_dict, max_timestep = self.PreProcess(node_cs, swap_cs)
    path_list, lsearch_stats = self.ll_starter_list[ri].find_path(upper_bound, node_dict, swap_dict, max_timestep)
    ct = 0 # count of node generated

    if self.use_cost_bound:
      path_list = gen_splitting(lower_bound, upper_bound, path_list)

    cost_list = []
    for i in path_list:
      cost_list.append(i[0])
    print(cost_list, lower_bound, upper_bound)

    for path_item in path_list: # loop over all individual Pareto paths
      new_nd = copy.deepcopy(self.nodes[nid])
      new_nd.sol.DelPath(ri)
      new_nd.sol.paths[ri] = [path_item[1][0], path_item[1][1], path_item[0]]
      new_nd.cvec = self.ComputeNodeCostObject(new_nd)
      if self.use_cost_bound:
        new_nd.cost_bound_list[ri] = CostBound(path_item[2], path_item[3])

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
      elif OPEN_ADD_MODE == 2:
        self.open_list.add(tuple(new_nd.cvec), new_nd.id) # add to OPEN
      ### ADD OPEN END

      ct = ct + 1 # count increase

    lsearch_stats.append(ct)
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

  def UpdateStats(self, stats):
    self.num_closed_low_level_states = self.num_closed_low_level_states + stats[0]
    self.total_low_level_time = self.total_low_level_time + stats[3]
    self.num_low_level_calls = self.num_low_level_calls + 1
    self.generated_node = self.generated_node + stats[4]
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
    """
  
    if self.open_list.size() == 0:
      return False, []
  
    popped = self.open_list.pop() # pop_node = (f-value, high-level-node-id)

    print(popped)
    return True, popped

def RunMocbsMAPF(G, sx, sy, gx, gy, search_limit, time_limit, use_cost_bound=False):

  mocbs = MocbsSearch(G, sx, sy, gx, gy, time_limit, use_cost_bound=use_cost_bound)

  return mocbs.Search(search_limit)
