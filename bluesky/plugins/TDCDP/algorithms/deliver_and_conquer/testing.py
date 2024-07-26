import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from PACO import PACO

ST = pd.read_csv('bluesky/plugins/TDCDP/algorithms/deliver_and_conquer/tbl_truck_travel_data_PG.csv')
ST['1/time'] = ST[' time [sec]'].apply(lambda x: 1/x if x != 0 else np.nan)

cities = int(np.sqrt(ST.shape[0]))

eta = np.full((cities, cities), np.nan)
dist_mat = np.full((cities, cities), np.nan)
# Populate the matrix
for _, row in ST.iterrows():
    i = int(row['% from location i'])
    j = int(row[' to location j'])
    
    # Ensure indices are within bounds
    if 0 <= i < cities and 0 <= j < cities:
        eta[i, j] = row['1/time']
        dist_mat[i, j] = row[' time [sec]']
    else:
        print(f"Warning: Index out of bounds (i={i}, j={j})")

from python_tsp.exact import solve_tsp_branch_and_bound

xopt, fopt = solve_tsp_branch_and_bound(dist_mat)

print(xopt, fopt)

# configs = [[1,1],[1,2]]
# configs = [[1,1],[1,2],[1,3],[1,4], [1,5],[2,1],[2,2],[2,3], [2,4],[2,5]]
# configs = [[1,10],[1,15],[1,20],[2,10], [2,15], [2,20]]
# configs = [[1,10,0.1],[1,10,0.3],[1,10,0.5],[1,10,0.7]]
# best_avgs = []
# for config in configs:
#     bests = []
#     print(config)
#     for j in range(10):
#         model = PACO(config[0], config[1], config[2], 3, 10, 200, eta, 5)
#         model.simulate()
#         bests.append(model.P[0].get_tour_length(model.eta))
    
#     best_avgs.append(np.average(bests))

# # Convert configs to string labels
# config_labels = [str(config) for config in configs]

# # Plot
# plt.figure(figsize=(8, 6))
# plt.plot(config_labels, best_avgs, marker='o', linestyle='-', color='b')
# plt.xlabel('Configs')
# plt.ylabel('Best Avg Values')
# plt.title('Best Avg Values for Different Configs')
# plt.grid(True)
# plt.show()

model = PACO(1, 10, 0.3, 3, 10, 200, eta, 5)
model.simulate()
model.plot_progress()
print(model.P[0].tour)
print(model.P[0].get_tour_length(model.eta))