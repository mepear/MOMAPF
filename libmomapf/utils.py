
def tr(val):
    return val[1:]


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
