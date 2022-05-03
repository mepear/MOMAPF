from utils import DomChecker, tr

class LLNode:
    def __init__(self, loc, timestep, g_val, h_val, parent=None):
        self.loc = loc
        self.timestep = timestep
        self.g_val = g_val
        self.h_val = h_val
        self.f_val = tuple(g + h for g,h in zip(g_val, h_val))

    def __repr__(self):
        return f"ll node - loc: {self.loc} - t: {self.timestep} - g: {self.g_val}"

    def __eq__(self, other):
        return self.loc == other.loc and self.timestep == other.timestep

    def __hash__(self):
        return hash(self.loc) + 12 * hash(self.timestep)

    def __lt__(self):
        return self.f_val < other.f_val


def is_constrained(constraints, ll_node):
    pass

class LLSolver:
    def __init__(self, G, start, goal):
        # G contains grids and cost info
        self.num_objective = G.num_objective
        self.start = start
        self.goal = goal
        self.heuristic = None
        self.compute_heuristic()

    def compute_heuristic(self):
        pass

    def get_heuristic(self, loc):
        # consider moving heuristic-related stuff to a separate class
        pass


    def get_children(self, node):
        children = []

        # generate children node here

        return children

    def find_path(self, constraints):
        start = LLNode(self.start, 0, tuple(0 for _ in range(G.num_objective)), self.get_heuristic(self.start))
        open_l = [start]
        # !! DomChecker only works for 2 and 3 objectives
        closed = [DomChecker() for _ in range(G.num_v)]
        sol_dom_checker = DomChecker()
        sols = []

        num_expand = 0
        while open_l:

            curr = heappop(open_l)
            if sol_dom_checker.is_dominated(tr(curr.f_val)):
                continue

            if self.is_goal(curr):
                sols.append(curr)
                sol_dom_checker.insert(tr(curr.f_val))
                continue

            if closed[curr.state].is_dominated(tr(curr.f_val)):
                continue

            closed[curr].insert(tr(curr.f_val))

            num_expand += 1
            children = self.get_children(curr)
            for ch in children:
                if is_constrained(constraints, ch):
                    continue
                if sol_dom_checker.is_dominated(tr(ch.f_val)):
                    continue
                if closed[ch.state].is_dominated(tr(ch.f_val)):
                    continue
                heappush(open_l, ch)
        return sols
