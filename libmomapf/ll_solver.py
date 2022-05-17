from heapq import heappush, heappop
from utils import DomChecker, tr
from copy import deepcopy
import numpy as np
import time

class LLNode:
    def __init__(self, loc, timestep, g_val, h_val, parent=None, indicator=None):
        self.loc = loc
        self.timestep = timestep
        self.g_val = g_val
        self.h_val = h_val
        self.f_val = tuple(g + h for g,h in zip(g_val, h_val))
        self.state = (loc, timestep)
        self.parent = parent
        self.indicator = indicator

    def __repr__(self):
        return f"ll node - loc: {self.loc} - t: {self.timestep} - g: {self.g_val} - h: {self.h_val}"

    def __eq__(self, other):
        return self.loc == other.loc and self.timestep == other.timestep

    def __hash__(self):
        return hash(self.loc) + 12 * hash(self.timestep)

    def __lt__(self, other):
        return self.f_val < other.f_val

def is_constrained(node_constraints, swap_constraints, ll_node, parent_node):
    loc = parent_node.loc
    timestep = parent_node.timestep
    new_loc = ll_node.loc
    new_timestep = ll_node.timestep
    if (loc in swap_constraints) and (timestep in swap_constraints[loc]) and (new_loc in swap_constraints[loc][timestep]):
        return True
    if new_loc in node_constraints:
        if new_timestep in node_constraints[new_loc]:
            return True
    return False

class LLSolver:
    def __init__(self, G, start, goal):
        self.G = G
        # G contains grids and cost info
        self.map = G.map
        self.cost_grids= G.cost_grids
        (self.y_length, self.x_length) = G.map.shape
        self.num_objective = G.num_objective
        self.cost_list = G.cost_list
        # an integer that record the start and goal location
        self.start = start
        self.goal = goal
        self.heuristic = G.compute_heuristic(goal)

    def get_heuristic(self, loc, indicator):
        heuristic_list = []
        for idx in range(self.num_objective):
            if self.cost_list[idx] == 'turning':
                heuristic_list.append(self.heuristic[idx][(loc, indicator)])
            else:
                heuristic_list.append(self.heuristic[idx][loc])
        return tuple(heuristic_list)

    def is_goal(self, ll_node, node_constraints):
        """
        Verify whether low level search node reaches the goal
        """
        if (ll_node.loc != self.goal):
            return False
        if ll_node.loc not in node_constraints:
            return True
        elif ll_node.timestep > max(node_constraints[ll_node.loc]):
            return True
        
        return False

    def get_children(self, node, upper_bound, max_timestep):
        children = []

        # generate children node here
        current_loc = node.loc
        current_y = int(np.floor(current_loc / self.x_length))
        current_x = int(current_loc % self.x_length)
        action_x = [-1, 0, 1, 0, 0]
        action_y = [0, -1, 0, 1, 0]
        for ax, ay in zip(action_x, action_y):
            next_x = current_x + ax
            next_y = current_y + ay
            next_loc = next_y * self.x_length + next_x
            if (ax == 0) and (ay == 0) and (node.timestep > max_timestep):
                continue
            if (next_x >= self.x_length) or (next_x < 0) or (next_y >= self.y_length) or (next_y < 0):
                continue
            if (self.map[next_y, next_x] > 0):
                continue
            g_val, new_indicator = self.G.get_g_val(node.g_val, current_loc, next_loc, node.indicator)
            heuristic = self.get_heuristic(next_loc, new_indicator)
            new_node = LLNode(next_loc, node.timestep + 1, g_val, heuristic, parent=node, indicator=new_indicator)
            if (heuristic[0] + g_val[0] < upper_bound):
                children.append(new_node)

        return children

    def reconstruct_path(self, sols):
        """
        input state is all sols that reach goal, return a list of all paths and related cost
        """
        path_list = []
        cost_vec_list = []

        for i in range(len(sols)):
            current_node = deepcopy(sols[i])
            reverse_path = []  # in reverse order
            cost_vec_list.append(current_node.g_val)
            path_list.append([None, None, current_node.g_val])
            while hasattr(current_node, 'parent'):
                reverse_path.append(current_node.loc)
                current_node = current_node.parent
            # reverse output path here.
            path = []
            times = []
            for idx in range(len(reverse_path)):
                path.append(reverse_path[len(reverse_path) - 1 - idx])
                times.append(idx)
            path.append(path[-1])
            times.append(np.inf)

            path_list[i][0] = path
            path_list[i][1] = times

        return path_list, cost_vec_list

    def find_path(self, node_constraints=[], swap_constraints=[], upper_bound=np.inf, max_timestep=-1):

        start_time = time.perf_counter()

        start = LLNode(self.start, 0, tuple(0 for _ in range(self.num_objective)), self.get_heuristic(self.start, None))
        open_l = [start]
        # !! DomChecker only works for 2 and 3 objectives
        closed = dict()
        sol_dom_checker = DomChecker()
        sols = []

        num_expand = 0
        while open_l:

            curr = heappop(open_l)
            if sol_dom_checker.is_dominated(tr(curr.f_val)):
                continue

            if self.is_goal(curr, node_constraints):
                sols.append(curr)
                sol_dom_checker.insert(tr(curr.f_val))
                continue

            if curr.state in closed and closed[curr.state].is_dominated(tr(curr.f_val)):
                continue

            if curr.state not in closed:
                closed[curr.state] = DomChecker()
            closed[curr.state].insert(tr(curr.f_val))

            num_expand += 1
            children = self.get_children(curr, upper_bound, max_timestep)
            for ch in children:
                if is_constrained(node_constraints, swap_constraints, ch, ch.parent):
                    continue
                if sol_dom_checker.is_dominated(tr(ch.f_val)):
                    continue
                if ch.state in closed and closed[ch.state].is_dominated(tr(ch.f_val)):
                    continue
                heappush(open_l, ch)

        path_list, cost_vec_list = self.reconstruct_path(sols)

        ll_info = [num_expand, cost_vec_list, True, time.perf_counter() - start_time]

        return path_list, ll_info
