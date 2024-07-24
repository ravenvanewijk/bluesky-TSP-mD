import pandas as pd
import numpy as np
from PACO import PACO

ST = pd.read_csv('bluesky/plugins/TDCDP/algorithms/deliver_and_conquer/tbl_truck_travel_data_ST.csv')
ST['1/time'] = ST['time [sec]'].apply(lambda x: 1/x if x != 0 else np.nan)

eta = np.full((26, 26), np.nan)

# Populate the matrix
for _, row in ST.iterrows():
    i = int(row['% from location i'])
    j = int(row['to location j'])
    
    # Ensure indices are within bounds
    if 0 <= i <= 25 and 0 <= j <= 25:
        eta[i, j] = row['1/time']
    else:
        print(f"Warning: Index out of bounds (i={i}, j={j})")

model = PACO(5, 1, 0.3, 3, 10, 200, eta, 5)

model.simulate()
model.plot_progress()
print(model.P[0].tour)