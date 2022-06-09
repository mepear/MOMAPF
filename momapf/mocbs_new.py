import numpy as np
import copy
import time

import momapf.common as cm
import itertools as itt
from copy import deepcopy
from .ll_solver import LLSolver
from .utils import CostBound, gen_splitting

from graphviz import Digraph

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

    # flag = 1, negative vertex conflict
    # flag = 2, negative swap conflict
    # flag = 3, positive vertex conflict
    # flag = 4, positive swap conflict
    self.flag = flag

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

  def DelPath(self, i):
    self.paths.pop(i)
    return

  def GetPath(self, i):
    return self.paths[i]

  def CheckConflict(self, i, j, use_joint_splitting):
    ix = 0
    choice = 0 # np.random.randint(0, 2)
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
        if ivb == jvb: # vertex conflict at ivb
          if not use_joint_splitting:
            return [MocbsConstraint(i, ivb, ivb, t_lb+1, t_lb+1, j, 1),
                    MocbsConstraint(j, jvb, jvb, t_lb+1, t_lb+1, i, 1)]
          if choice == 0:
            return [MocbsConstraint(i, ivb, ivb, t_lb+1, t_lb+1, j, 1),
                    MocbsConstraint(i, ivb, ivb, t_lb+1, t_lb+1, j, 3)]
          if choice == 1:
            return [MocbsConstraint(j, jvb, jvb, t_lb + 1, t_lb + 1, i, 1),
                    MocbsConstraint(j, jvb, jvb, t_lb + 1, t_lb + 1, i, 3)]
        if (ivb == jva) and (iva == jvb): # swap location
          if not use_joint_splitting:
            return [MocbsConstraint(i, iva, ivb, t_lb, t_lb+1, j, 2),
                    MocbsConstraint(j, jva, jvb, t_lb, t_lb+1, i, 2)]
          if choice == 0:
            return [MocbsConstraint(i, iva, ivb, t_lb, t_lb+1, j, 2),
                    MocbsConstraint(i, iva, ivb, t_lb, t_lb+1, j, 4)]
          if choice == 1:
            return [MocbsConstraint(j, jva, jvb, t_lb, t_lb+1, i, 2),
                    MocbsConstraint(j, jva, jvb, t_lb, t_lb+1, i, 4)]
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
    self.cost_bound_list = [CostBound(tuple(0 for _ in range(len(cvec))),
                                      [tuple(np.inf for _ in range(len(cvec)))]) for _ in range(num_robots)]
    self.parent = parent
    self.root = -1 # to which tree it belongs

    return

  def __str__(self):
    str1 = "{id:"+str(self.id)+",cvec:"+str(self.cvec)+",par:"+str(self.parent)
    return str1+",cstr:"+str(self.cstr)+",sol:"+str(self.sol)+"}"

  def CheckConflict(self, use_joint_splitting):
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
        res = self.sol.CheckConflict(k1,k2,use_joint_splitting)
        if len(res) > 0:
          return res
      # end for k2
      done_set.add(k1) # auxiliary
    return [] # no conflict

class MocbsSearch:
  """
  """
  def __init__(self, G, sx_list, sy_list, gx_list, gy_list, time_limit, use_cost_bound=False, use_joint_splitting=False, draw_graph=False):
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
    self.use_joint_splitting = use_joint_splitting

    # Graph Related parameter
    self.draw_graph = draw_graph
    self.graph = Digraph(format='png')
    self.graph.node_attr["fixedsize"] = "true"
    self.graph.node_attr["width"] = "2"
    self.graph.graph_attr["dpi"] = '1000'
    self.init_list = []
    return

  def Search(self, search_limit):
    """
    high level search
    """
    if self.draw_graph:
      self.graph.node('Init')

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
        if cstr.flag in [1, 2]:
          sstats = self.Lsearch(new_id)  # this can generate multiple nodes
        elif cstr.flag in [3, 4]:
          sstats = self.new_Lsearch(new_id)
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
    if self.num_low_level_calls != 0:
      branch_factor = float(self.generated_node) / float(self.num_low_level_calls)
    else:
      branch_factor = None

    if self.draw_graph:
      self.graph.render("Tree")

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

  def BacktrackCstrs(self, nid, ri):
    """
    given a node, trace back to the root, find all constraints relavant.
    """
    node_cs = list()
    swap_cs = list()
    positive_cs = list()
    cid = nid
    while cid != -1:
      if self.nodes[cid].cstr.flag in [1, 2]: # not a valid constraint
        if self.nodes[cid].cstr.i == ri:
          cstr = self.nodes[cid].cstr
          if self.nodes[cid].cstr.flag == 1: # negative vertex constraint
            node_cs.append((cstr.vb, cstr.tb))
          elif self.nodes[cid].cstr.flag == 2: # negative edge constraint
            swap_cs.append((cstr.va, cstr.vb, cstr.ta))
      elif self.nodes[cid].cstr.flag in [3, 4]:
        cstr = self.nodes[cid].cstr
        if self.nodes[cid].cstr.flag == 3:
          if self.nodes[cid].cstr.i == ri:
            positive_cs.append((cstr.vb, cstr.tb))
          else:
            node_cs.append((cstr.vb, cstr.tb))
        elif self.nodes[cid].cstr.flag == 4:
          if self.nodes[cid].cstr.i == ri:
            positive_cs.append((cstr.va, cstr.ta))
            positive_cs.append((cstr.vb, cstr.tb))
          else:
            node_cs.append((cstr.va, cstr.ta))
            node_cs.append((cstr.vb, cstr.tb))
            swap_cs.append((cstr.vb, cstr.va, cstr.ta))
      cid = self.nodes[cid].parent
    return node_cs, swap_cs, positive_cs

  def PreProcess(self, node_cs, swap_cs, positive_cs):
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

    positive_dict = dict()
    for constraint in positive_cs:
      if constraint[1] not in positive_dict:
        positive_dict[constraint[1]] = constraint[0]
      elif positive_dict[constraint[1]] != constraint[0]:
        positive_dict = None
      max_timestep = max(constraint[1], max_timestep)

    return node_dict, swap_dict, positive_dict, max_timestep

  def Lsearch(self, nid):
    """
    low level search for one robot
    """
    nd = self.nodes[nid]
    ri = nd.cstr.i

    total_lsearch_stats = [0, None, True, 0, 0, 1]

    node_cs, swap_cs, positive_cs = self.BacktrackCstrs(nid, ri)
    lower_bound = nd.cost_bound_list[ri].lb
    upper_bound = nd.cost_bound_list[ri].ub

    node_dict, swap_dict, positive_dict, max_timestep = self.PreProcess(node_cs, swap_cs, positive_cs)

    if positive_dict == None:
      print("Conflict positive constraints!!!")
      return [0, [], True, time.perf_counter() - self.tstart, 0]

    path_list, lsearch_stats = self.ll_starter_list[ri].find_path(upper_bound, node_dict,
                                                                  swap_dict, positive_dict, max_timestep)

    total_lsearch_stats[0] = lsearch_stats[0]
    total_lsearch_stats[3] = lsearch_stats[3]

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

      total_lsearch_stats[4] += 1

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

      if self.draw_graph:
        constraint = new_nd.cstr
        parent_id = self.nodes[new_nd.parent].id
        label = ""
        if constraint.flag == 1:
          label += "Vertice "
        if constraint.flag == 2:
          label += "Edge "
        label += "Agent:{}\n Vertice:{}, Timestep:{}\n".format(constraint.i, constraint.va, constraint.ta)
        label += "Cost:"
        label += str(new_nd.sol.paths[ri][2])
        label += " Time:{}".format(int(lsearch_stats[3]))

        if parent_id <= self.num_roots:
          self.graph.node("Node_{}".format(parent_id))
          if parent_id not in self.init_list:
            self.init_list.append(parent_id)
            self.graph.edge("Init", "Node_{}".format(parent_id))

        self.graph.node("Node_{}".format(self.node_id_gen - 1), label=label)
        self.graph.edge("Node_{}".format(parent_id), "Node_{}".format(self.node_id_gen - 1))

      ct = ct + 1  # count increase

    return total_lsearch_stats

  def FindReplan(self, node):

    constraint_list = list()
    constraint = node.cstr
    robot_idx = node.cstr.i
    replan_list = list()
    if node.cstr.flag == 3:
      constraint_list.append((constraint.vb, constraint.tb))
    elif node.cstr.flag == 4:
      constraint_list.append((constraint.va, constraint.ta))
      constraint_list.append((constraint.vb, constraint.tb))
    for i in range(self.num_robots):
      if i != robot_idx:
        path = [node.sol.paths[i][0], node.sol.paths[i][1]]
        if self.is_conflict(path, constraint_list):
          replan_list.append(i)

    return replan_list

  def is_conflict(self, path, constraint_list):

    flag = False
    for position, time in constraint_list:
      if time >= len(path[1]) and path[0][-1] == position:
        flag = True
      if time < len(path[1]) and path[0][time] == position:
        flag = True
    if len(constraint_list) == 2:
      if constraint_list[0][1] <= len(path[1]) - 3:
        if path[0][constraint_list[0][1]] == constraint_list[1][0] \
                and path[0][constraint_list[1][1]] == constraint_list[0][0]:
         flag = True

    return flag

  def new_Lsearch(self, nid):
    """
    low level search with several robot re-planing
    """
    path_dict = dict()
    nd = self.nodes[nid]
    replan_list = self.FindReplan(nd)
    print("Replan List", replan_list)

    total_lsearch_stats = [0, None, True, 0, 0, 0]

    for ri in replan_list:

      node_cs, swap_cs, positive_cs = self.BacktrackCstrs(nid, ri)
      lower_bound = nd.cost_bound_list[ri].lb
      upper_bound = nd.cost_bound_list[ri].ub

      node_dict, swap_dict, positive_dict, max_timestep = self.PreProcess(node_cs, swap_cs, positive_cs)

      if positive_dict == None:
        print("Conflict positive constraints!!!")
        return [0, [], True, time.perf_counter() - self.tstart, 0]

      path_list, lsearch_stats = self.ll_starter_list[ri].find_path(upper_bound,
                                                                    node_dict, swap_dict, positive_dict, max_timestep)
      if self.use_cost_bound:
        path_list = gen_splitting(lower_bound, upper_bound, path_list)

      cost_list = []
      for i in path_list:
        cost_list.append(i[0])
      print(cost_list, lower_bound, upper_bound)

      path_dict[ri] = path_list

      total_lsearch_stats[0] += lsearch_stats[0]
      total_lsearch_stats[3] += lsearch_stats[3]

    total_lsearch_stats[5] = len(replan_list)

    all_combi = list(itt.product(*(path_dict[ri] for ri in replan_list)))

    for path_item in all_combi:
      new_nd = copy.deepcopy(self.nodes[nid])
      for i in range(len(replan_list)):
        new_nd.sol.DelPath(replan_list[i])
        new_nd.sol.paths[replan_list[i]] = [path_item[i][1][0], path_item[i][1][1], path_item[i][0]]
        if self.use_cost_bound:
          new_nd.cost_bound_list[replan_list[i]] = CostBound(path_item[i][2], path_item[i][3])
      new_nd.cvec = self.ComputeNodeCostObject(new_nd)

      total_lsearch_stats[4] = total_lsearch_stats[4] + 1
      if self.GoalFilterNodeObject(new_nd):
        continue # skip this dominated node

      # a non-dom node, add to OPEN
      new_id = new_nd.id # the first node, self.nodes[nid] is ok
      if total_lsearch_stats[4] > 0: # generate a new node, assign a new id
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


    return total_lsearch_stats

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
    self.num_low_level_calls = self.num_low_level_calls + stats[5]
    self.generated_node = self.generated_node + stats[4]
    return

  def FirstConflict(self, nd):
    return nd.CheckConflict(self.use_joint_splitting)

  def SelectNode(self):
    """
    Pop a node from OPEN. 
    """
  
    if self.open_list.size() == 0:
      return False, []
  
    popped = self.open_list.pop() # pop_node = (f-value, high-level-node-id)

    print(popped)
    return True, popped

def RunMocbsMAPF(G, sx, sy, gx, gy, search_limit, time_limit,
                 use_cost_bound=False, use_joint_splitting=False, draw_graph=False):

  mocbs = MocbsSearch(G, sx, sy, gx, gy, time_limit,
                      use_cost_bound=use_cost_bound, use_joint_splitting=use_joint_splitting, draw_graph=draw_graph)

  return mocbs.Search(search_limit)
