import gurobipy as gp
import pulp

class LR_PuLP:

    def __init__(self, L, P, t_ij, t_jk, T_i, T_k, T_ik, alpha, beta, B=10**9):
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
            - alpha: float, relative importance of excess waiting time
            - beta: float, maximum allowed waiting time of truck or drone
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
        self.alpha = alpha
        self.beta = beta

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
        self.w_p = pulp.LpVariable.dicts("p", (self.L, self.P), lowBound=0, cat='Continuous')

        # Objective function
        self.model += pulp.lpSum(self.w_p[i][k] * self.beta + (self.t_ij[i] + self.t_jk[k]) * self.z[i][k]
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

        # Waiting time soft constraint
        for i in self.L:
            for k in self.P:
                self.model += self.w[i][k] - self.alpha <= self.w_p[i][k]
        
        # Prohibit equal launch are retrieval location, algorithmic convenience
        for i in self.L:
            for k in self.P:
                if i == k:
                    self.model += self.x[i] + self.y[k] <= 1, f"not_same_{i}_{k}"

    def solve(self):
        """Solves the model that has been set up by the create_model method"""
        # Optimize model
        solver = pulp.PULP_CBC_CMD(msg=False)
        self.model.solve(solver)

        # Retrieve the results
        if pulp.LpStatus[self.model.status] == 'Optimal':
            self.launch_location = [i for i in self.L if pulp.value(self.x[i]) > 0.5][0]
            self.pickup_location = [k for k in self.P if pulp.value(self.y[k]) > 0.5][0]

            # Calculate waiting time and traveling time
            self.waiting_time = pulp.value(self.w[self.launch_location][self.pickup_location])
            d_traveling_time = (self.t_ij[self.launch_location] + self.t_jk[self.pickup_location]) * pulp.value(self.z[self.launch_location][self.pickup_location])
            t_traveling_time = self.T_k[self.pickup_location] - self.T_i[self.launch_location]

            # Determine who is waiting
            truck_arrival_time = self.T_k[self.pickup_location]
            drone_arrival_time = self.T_i[self.launch_location] + \
                                    self.t_ij[self.launch_location] + \
                                    self.t_jk[self.pickup_location]
            waiting_entity = "Truck" if truck_arrival_time < drone_arrival_time else "Drone"

            print(f"Optimal launch location: {self.launch_location}")
            print(f"Optimal pickup location: {self.pickup_location}")
            print(f"Waiting time: {self.waiting_time}")
            print(f"Drone Traveling time: {d_traveling_time}")
            print(f"Truck Travelling time: {t_traveling_time}")
            print(f"{waiting_entity} is waiting at the pickup location.")
            print(f"Arrival at launch in : {self.T_i[self.launch_location]}")

        else:
            print("No optimal solution found")