import numpy as np
import math
import seaborn as sns
import matplotlib.pyplot as plt

plt.subplot()
#
# hill_map = np.zeros((32, 32), dtype=np.int)
# random = np.random.randint(-5, 5, size=(32, 32))
#
# top = [[25, 3], [16, 20], [15, 10], [5, 22]]
#
# for x_idx, y_idx in top:
#     for i in range(32):
#         for j in range(32):
#             hill_map[i, j] += int(math.pow(math.pow(x_idx - i, 4) + math.pow(y_idx - j, 4), 0.25))
#
# hill_map = (100 - hill_map + random)

hill_map = np.load('./benchmark/random-32-32-20/random-32-32-20-hill.npy')

heatmap = sns.heatmap(hill_map, cmap='Blues')

plt.show()

# np.save('./benchmark/maze-32-32-2/maze-32-32-2-hill.npy', hill_map)
