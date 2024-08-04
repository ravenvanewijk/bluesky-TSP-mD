import gurobipy as gp
from gurobipy import GRB
import pulp

# # Example inputs
# L = [1, 2, 3]  # Launch locations
# P = [1, 2, 3]  # Pickup locations

# # Drone travel time from launch location to customer location j
# t_ij = {
#     1: 10, 2: 12, 3: 14
# }

# # Drone travel time from customer location j to pickup location
# t_jk = {
#     1: 15, 2: 18, 3: 20
# }

# # Time at which the truck passes launch locations
# T_i = {
#     1: 5, 
#     2: 10, 
#     3: 15
# }

# # Time at which the truck passes pickup locations
# T_k = {
#     1: 25, 
#     2: 30, 
#     3: 35
# }

# # Truck travel time from launch location to pickup location
# T_ik = {
#     (1, 1): 20, (2, 1): 15, (3, 1): 10,
#     (1, 2): 25, (2, 2): 20, (3, 2): 15,
#     (1, 3): 30, (2, 3): 25, (3, 3): 20
# }

# # Battery life of the drone in terms of maximum travel time
# B = 40


class LR_Optimizer:

    def __init__(self, L, P, t_ij, t_jk, T_i, T_k, T_ik, B):
        self.L          = L
        self.P          = P
        self.t_ij       = t_ij
        self.t_jk       = t_jk
        self.T_i        = T_i
        self.T_k        = T_k
        self.T_ik       = T_ik
        self.B          = B

    def create_model(self):
        # Create a new model
        self.model = gp.Model("Drone_Pickup_Optimization")

        # Decision variables
        self.x = self.model.addVars(self.L, vtype=GRB.BINARY, name="x")
        self.y = self.model.addVars(self.P, vtype=GRB.BINARY, name="y")
        self.z = self.model.addVars(self.L, self.P, vtype=GRB.BINARY, name="z")
        self.w = self.model.addVars(self.L, self.P, vtype=GRB.CONTINUOUS, name="w")

        # Objective function
        self.model.setObjective(gp.quicksum(self.w[i, k] * 10 + (self.t_ij[i] + self.t_jk[k]) * self.z[i, k]
                                    for i in self.L for k in self.P), GRB.MINIMIZE)

        # Constraints
        # Only one launch location can be chosen
        self.model.addConstr(gp.quicksum(self.x[i] for i in self.L) == 1, "launch_location")

        # Only one pickup location can be chosen
        self.model.addConstr(gp.quicksum(self.y[k] for k in self.P) == 1, "pickup_location")

        # Battery constraint for the drone
        self.model.addConstr(gp.quicksum((self.t_ij[i] + self.t_jk[k]) * self.z[i, k]
                                    for i in self.L for k in self.P) <= self.B, "battery")

        # Linking constraints
        for i in self.L:
            for k in self.P:
                self.model.addConstr(self.z[i, k] <= self.x[i], f"linking_x_{i}_{k}")
                self.model.addConstr(self.z[i, k] <= self.y[k], f"linking_y_{i}_{k}")

        # Flow constraint
        self.model.addConstr(gp.quicksum(self.z[i, k] for i in self.L for k in self.P) == 1, "flow")

        # Absolute value constraints
        for i in self.L:
            for k in self.P:
                self.model.addConstr(self.w[i, k] >= (self.T_k[k] - (self.T_i[i] + self.t_ij[i] + self.t_jk[k])) * self.z[i, k],
                                f"abs_waiting_1_{i}_{k}")
                self.model.addConstr(self.w[i, k] >= ((self.T_i[i] + self.t_ij[i] + self.t_jk[k]) - self.T_k[k]) * self.z[i, k],
                                f"abs_waiting_2_{i}_{k}")

    def solve(self):
        # Optimize model
        self.model.optimize()

        # Retrieve the results  
        if self.model.status == GRB.OPTIMAL:
            self.launch_location = [i for i in self.L if self.x[i].x > 0.5][0]
            self.pickup_location = [k for k in self.P if self.y[k].x > 0.5][0]
            
            # Calculate waiting time and traveling time
            waiting_time = self.w[self.launch_location, self.pickup_location].x
            traveling_time = (self.t_ij[self.launch_location] + \
                        self.t_jk[self.pickup_location]) * self.z[self.launch_location, self.pickup_location].x
            
            # Determine who is waiting
            truck_arrival_time = self.T_i[self.launch_location] + self.t_ij[self.launch_location]
            drone_arrival_time = self.T_k[self.pickup_location]
            waiting_entity = "Truck" if truck_arrival_time < drone_arrival_time else "Drone"

            print(f"Optimal launch location: {self.launch_location}")
            print(f"Optimal pickup location: {self.pickup_location}")
            print(f"Waiting time: {waiting_time}")
            print(f"Traveling time: {traveling_time}")
            print(f"{waiting_entity} is waiting at the pickup location.")

        else:
            print("No optimal solution found")


# lr_opt = LR_Optimizer(L, P, t_ij, t_jk, T_i, T_k, T_ik, B)
# lr_opt.create_model()
# lr_opt.solve()



class LR_PuLP:

    def __init__(self, L, P, t_ij, t_jk, T_i, T_k, T_ik, B=10**9):
        """
        Initialize PuLP model to solve the model to find optimal launch and 
        retrieval location. 

        Params: type, description
        
            - L: list, list of all wpidxs of the potential launch locations
            - P: list, list of all wpidxs of the potential retrieval locations
            - t_ij: dictionary, lookup values of drone travel time from 
            potential launch location i to customer location j
            - t_jk: dictionary, lookup values of drone travel time from 
            customer location j to potential retrieval location k
            - T_i: dictionary, lookup values of truck travel time to reach 
            potential launch location i
            - T_k: dictionary, lookup values of truck travel time to reach 
            potential retrieval location k
            - T_ik: dictionary, lookup values of truck travel time from 
            potential launch location i to potential retrieval location k
            - B: float, drone battery life [NOT CURRENTLY IMPLEMENTED]
        """
        self.L = L
        self.P = P
        self.t_ij = t_ij
        self.t_jk = t_jk
        self.T_i = T_i
        self.T_k = T_k
        self.T_ik = T_ik
        self.B = B

    def create_model(self):
        """Method to create the model. Sets up constraints, objective function,
        and ensures the variables are initiated"""
        # Create a new model
        self.model = pulp.LpProblem("Drone_Pickup_Optimization", pulp.LpMinimize)

        # Decision variables
        self.x = pulp.LpVariable.dicts("x", self.L, cat='Binary')
        self.y = pulp.LpVariable.dicts("y", self.P, cat='Binary')
        self.z = pulp.LpVariable.dicts("z", (self.L, self.P), cat='Binary')
        self.w = pulp.LpVariable.dicts("w", (self.L, self.P), lowBound=0, cat='Continuous')

        # Objective function
        self.model += pulp.lpSum(self.w[i][k] * 10 + (self.t_ij[i] + self.t_jk[k]) * self.z[i][k]
                                 for i in self.L for k in self.P)

        # Constraints
        # Only one launch location can be chosen
        self.model += pulp.lpSum(self.x[i] for i in self.L) == 1, "launch_location"

        # Only one pickup location can be chosen
        self.model += pulp.lpSum(self.y[k] for k in self.P) == 1, "pickup_location"

        # Battery constraint for the drone
        self.model += pulp.lpSum((self.t_ij[i] + self.t_jk[k]) * self.z[i][k]
                                 for i in self.L for k in self.P) <= self.B, "battery"

        # Linking constraints
        for i in self.L:
            for k in self.P:
                self.model += self.z[i][k] <= self.x[i], f"linking_x_{i}_{k}"
                self.model += self.z[i][k] <= self.y[k], f"linking_y_{i}_{k}"

        # Flow constraint
        self.model += pulp.lpSum(self.z[i][k] for i in self.L for k in self.P) == 1, "flow"

        # Absolute value constraints
        for i in self.L:
            for k in self.P:
                self.model += self.w[i][k] >= (self.T_k[k] - (self.T_i[i] + self.t_ij[i] + self.t_jk[k])) * self.z[i][k], f"abs_waiting_1_{i}_{k}"
                self.model += self.w[i][k] >= ((self.T_i[i] + self.t_ij[i] + self.t_jk[k]) - self.T_k[k]) * self.z[i][k], f"abs_waiting_2_{i}_{k}"

    def solve(self):
        """Solves the model that has been set up by the create_model method"""
        # Optimize model
        self.model.solve()

        # Retrieve the results
        if pulp.LpStatus[self.model.status] == 'Optimal':
            self.launch_location = [i for i in self.L if pulp.value(self.x[i]) > 0.5][0]
            self.pickup_location = [k for k in self.P if pulp.value(self.y[k]) > 0.5][0]

            # Calculate waiting time and traveling time
            waiting_time = pulp.value(self.w[self.launch_location][self.pickup_location])
            traveling_time = (self.t_ij[self.launch_location] + self.t_jk[self.pickup_location]) * pulp.value(self.z[self.launch_location][self.pickup_location])

            # Determine who is waiting
            truck_arrival_time = self.T_i[self.launch_location] + self.t_ij[self.launch_location]
            drone_arrival_time = self.T_k[self.pickup_location]
            waiting_entity = "Truck" if truck_arrival_time < drone_arrival_time else "Drone"

            print(f"Optimal launch location: {self.launch_location}")
            print(f"Optimal pickup location: {self.pickup_location}")
            print(f"Waiting time: {waiting_time}")
            print(f"Traveling time: {traveling_time}")
            print(f"{waiting_entity} is waiting at the pickup location.")

        else:
            print("No optimal solution found")