import numpy as np

def tr(val):
    return val[1:]

def is_weakly_dominated(vec1, vec2):
    for n1, n2 in zip(vec1, vec2):
        if n1 < n2:
            return False
    return True

def is_weakly_dominated_it(vec, vec_list):
    for vec_2 in vec_list:
        if is_weakly_dominated(vec, vec_2):
            return True

    return False

def update_list_it(vec_list, vec):
    vec_list = [vec] + [v for v in vec_list if not is_weakly_dominated(v, vec)]
    return vec_list


class DomChecker:
    """
    testing:
ins = [(4,6), (2,9), (10,1), (8,3), (1,11), (3,5), (3,2),(2,2), (3,3), (1,10), (1,9), (1,13), (10 ,2),(1,9), (4,5)]
dc = DomChecker()
for i in ins:
    print(f"inserting {i}, is domed?: {dc.is_dominated(i)}")
    dc.insert(i)
    print(dc)
    """

    def __init__(self):
        self.vals = [] # sorted

    def insert(self, val):
        """
        insert new val and remove all dominated values
        """
        if not self.vals:
            self.vals.append(val)

        if self.vals[0][0] >= val[0]:
            j = 0
            if len(val) == 1:
                j = 1
            else:
                while j < len(self.vals):
                    if self.vals[j][1:] >= val[1:]:
                        j += 1
                    else:
                        break
            self.vals = [val] + self.vals[j:]
        if val >= self.vals[-1]:
            if self.vals[-1][0] != val[0]:
                self.vals.append(val)
            return


        # find insert point
        lb = 0
        ub = len(self.vals)
        i = (lb + ub) // 2
        while (ub - lb > 1):
            if self.vals[i][0] < val[0]:
                lb = i
            # elif self.vals[i] == val:
            #    return
            else:
                ub = i
            i = (lb + ub) // 2

        if self.vals[i][1] <= val[1]:
            return

        if self.vals[i+1][1] < val[1]:
            self.vals.insert(i + 1, val)
            return

        lb = i + 1
        ub = len(self.vals)
        j = (lb + ub) // 2
        while (ub - lb > 1):
            if self.vals[j][1] >= val[1]:
                lb = j
            else:
                ub = j
            j = (lb + ub) // 2
        self.vals = self.vals[: i + 1] +  [val] + self.vals[j+1:]

    def __repr__(self):
        return str(self.vals)

    def is_dominated(self, val):
        if not self.vals:
            return False
        # print(self.vals)

        if val[0] < self.vals[0][0] or (len(val) > 1 and val[1] < self.vals[-1][1]):
            return False
        if val[0] >= self.vals[0][0] and (len(val) == 1 or val[1] >= self.vals[0][1]):
            return True

        if val[1] >= self.vals[0][1]:
            return True
        lb = 0
        ub = len(self.vals)
        i = (lb + ub) // 2
        while (ub - lb > 1):
            if self.vals[i][0] <= val[0]:
                lb = i
                if val[1] >= self.vals[lb][1]:
                    return True
            else:
                ub = i
            i = (lb + ub) // 2

        return False

class Map:
    """
    Used to save map information and other map related cost information
    """
    def __init__(self, grid_map, cost_grids, cost_list):
        self.map = grid_map
        self.cost_grids = cost_grids
        (self.y_length, self.x_length) = self.map.shape
        self.cost_list = cost_list
        self.num_objective = len(cost_list)

    def compute_heuristic(self, goal):
        perfect_heuristic = {}
        for idx in range(self.num_objective):
            if self.cost_list[idx] == 'random':
                found_dict = {goal: 0}
                not_found_dict = dict()
                action_x = [1, 0, -1, 0]
                action_y = [0, 1, 0, -1]
                location = goal
                cy = int(np.floor(location / self.x_length))
                cx = int(location % self.x_length)
                while True:
                    for i in range(4):
                        cy_temp = cy + action_y[i]
                        cx_temp = cx + action_x[i]
                        location_temp = cy_temp * self.x_length + cx_temp
                        if (cx_temp >= self.x_length) or (cx_temp < 0) or (cy_temp >= self.y_length) or (cy_temp < 0):
                            continue
                        if self.map[cy_temp, cx_temp] > 0:
                            continue
                        if location_temp in found_dict:
                            continue
                        elif location_temp not in not_found_dict:
                            not_found_dict[location_temp] = found_dict[location] + \
                                                            self.cost_grids[idx][cy, cx]
                        else:
                            not_found_dict[location_temp] = min(
                                found_dict[location] + self.cost_grids[idx][cy, cx],
                                not_found_dict[location_temp])
                    # End for
                    location = min(not_found_dict, key=lambda x: not_found_dict[x])
                    cy = int(np.floor(location / self.x_length))
                    cx = int(location % self.x_length)
                    found_dict[location] = not_found_dict[location]
                    not_found_dict.pop(location)
                    if not not_found_dict:
                        break
                perfect_heuristic[idx] = found_dict
            elif self.cost_list[idx] == 'time' or self.cost_list[idx] == 'distance':
                found_dict = {goal: 0}
                not_found_dict = dict()
                action_x = [1, 0, -1, 0]
                action_y = [0, 1, 0, -1]
                location = goal
                cy = int(np.floor(location / self.x_length))
                cx = int(location % self.x_length)
                while True:
                    for i in range(4):
                        cy_temp = cy + action_y[i]
                        cx_temp = cx + action_x[i]
                        location_temp = cy_temp * self.x_length + cx_temp
                        if (cx_temp >= self.x_length) or (cx_temp < 0) or (cy_temp >= self.y_length) or (cy_temp < 0):
                            continue
                        if self.map[cy_temp, cx_temp] > 0:
                            continue
                        if location_temp in found_dict:
                            continue
                        elif location_temp not in not_found_dict:
                            not_found_dict[location_temp] = found_dict[location] + 1
                        else:
                            not_found_dict[location_temp] = min(found_dict[location] + 1,
                                                                not_found_dict[location_temp])
                    # End for
                    location = min(not_found_dict, key=lambda x: not_found_dict[x])
                    cy = int(np.floor(location / self.x_length))
                    cx = int(location % self.x_length)
                    found_dict[location] = not_found_dict[location]
                    not_found_dict.pop(location)
                    if not not_found_dict:
                        break
                perfect_heuristic[idx] = found_dict
            elif self.cost_list[idx] == 'turning':
                found_dict = {(goal, None): 0}
                not_found_dict = dict()
                action_x = [1, 0, -1, 0]
                action_y = [0, 1, 0, -1]
                location = goal
                indicator = None
                cy = int(np.floor(location / self.x_length))
                cx = int(location % self.x_length)
                while True:
                    for i in range(4):
                        cy_temp = cy + action_y[i]
                        cx_temp = cx + action_x[i]
                        location_temp = cy_temp * self.x_length + cx_temp
                        if (cx_temp >= self.x_length) or (cx_temp < 0) or (cy_temp >= self.y_length) or (cy_temp < 0):
                            continue
                        if self.map[cy_temp, cx_temp] > 0 or (indicator != None and location_temp - location == -indicator):
                            continue
                        if (location_temp, location_temp - location) in found_dict:
                            continue
                        elif (location_temp, location_temp - location) not in not_found_dict:
                            if location_temp - location == indicator or indicator == None:
                                not_found_dict[(location_temp, location_temp - location)] = found_dict[(location, indicator)]
                            else:
                                not_found_dict[(location_temp, location_temp - location)] = found_dict[(location, indicator)] + 1
                        elif location_temp - location == indicator or indicator == None:
                            not_found_dict[(location_temp, location_temp - location)] = min(found_dict[(location, indicator)],
                                                                            not_found_dict[(location_temp, location_temp - location)])
                        else:
                            not_found_dict[(location_temp, location_temp - location)] = min(found_dict[(location, indicator)] + 1,
                                                                            not_found_dict[(location_temp, location_temp - location)])
                    # End for
                    (location, indicator) = min(not_found_dict, key=lambda x: not_found_dict[x])
                    cy = int(np.floor(location / self.x_length))
                    cx = int(location % self.x_length)
                    found_dict[(location, indicator)] = not_found_dict[(location, indicator)]
                    not_found_dict.pop((location, indicator))
                    if not not_found_dict:
                        break

                perfect_heuristic[idx] = found_dict
            else:
                print("Do not define such cost")
                exit()
        return perfect_heuristic

    def get_g_val(self, prev_g_val, loc, new_loc, indicator):

        out_cost = list(prev_g_val)
        new_indicator = None

        # Calculate cost for single step
        for idx in range(self.num_objective):
            if self.cost_list[idx] == 'random':
                if new_loc != loc:
                    cy = int(np.floor(new_loc / self.x_length))
                    cx = int(new_loc % self.x_length)
                    out_cost[idx] = out_cost[idx] + self.cost_grids[idx][cy, cx]
                else:
                    out_cost[idx] = out_cost[idx] + 1
            elif self.cost_list[idx] == 'time':
                out_cost[idx] += 1
            elif self.cost_list[idx] == 'distance':
                if new_loc != loc:
                    out_cost[idx] += 1
            elif self.cost_list[idx] == 'turning':
                if new_loc != loc:
                    new_indicator = new_loc - loc
                    if new_indicator == indicator or indicator == None:
                        pass
                    elif new_indicator == -indicator:
                        out_cost[idx] += 2
                    else:
                        out_cost[idx] += 1
                else:
                    new_indicator = indicator
            else:
                print("Do not define such cost")
                exit()
        return tuple(out_cost), new_indicator


def comax(v1, v2):
    # Assumption: v1 and any v2 in V2 only contain non-negative component
    # V2 is sorted lexicographically

    return tuple(max(n1, n2) for n1, n2 in zip(v1, v2))

def ndcomax(v1, V2):
    # Assumption: v1 and any v2 in V2 only contain non-negative component
    # V2 is sorted lexicographically

    res = []
    tr_vecs = []

    for new_vec in sorted(comax(v1, v2) for v2 in V2):
#         new_vec = vec_max(v1, v2)
        if is_weakly_dominated_it(tr(new_vec), tr_vecs):
            continue
        res.append(new_vec)
        tr_vecs = update_list_it(tr_vecs, tr(new_vec))

    return res

def ndcomax_path(v1, V2):
    # Assumption: v1 and any v2 in V2 only contain non-negative component
    # V2 is sorted lexicographically
    # V2 = [(cost1, path1), (cos2, path2) .... ]


    res = []
    tr_vecs = []

    for new_vec in sorted((comax(v1, v2), path) for v2, path in V2):
#         new_vec = vec_max(v1, v2)
        if is_weakly_dominated_it(tr(new_vec[0]), tr_vecs):
            continue
        res.append(new_vec)
        tr_vecs = update_list_it(tr_vecs, tr(new_vec[0]))

    return res


def gen_splitting(lb, ub, paths):
    prev = []

    for cost, path in ndcomax_path(lb, paths):
        new_lb = cost
        new_ub = [comax(u, cost) for u in ub]
        for p in prev:
            new_ub = update_list_it(new_ub, comax(p, cost))

        print(f"{path} - {new_lb} - {new_ub}")
        prev.append(cost)