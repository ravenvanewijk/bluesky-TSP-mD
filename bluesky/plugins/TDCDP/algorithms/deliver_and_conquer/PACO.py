import numpy as np
import random
import matplotlib.pyplot as plt

class PACO:
    """Class to represent a Population based Ant Colony optimization model"""
    def __init__(self, alpha, beta, q0, k, m, gens, eta, tau_mul, seed=None):
        """Initialisation of Population based Ant Colony Optimization model
        Implemented from Applying Population Based ACO to Dynamic Optimization Problems 
        https://doi.org/10.1007/3-540-45724-0_10 
        With own twists and own vision:
            - Scaling of tau_init is done is a novel way by considering the ratio of beta and alpha
            This ensures the relative importance is equal, however, beta can e.g. be set to a high value
            to introduce high discrepancy between good and bad choices.
            - Implementation of surviving population is purely based on an ants performance.
        Params: type, description:
            
            - alpha: float, relative importance of heuristic value
            - beta: float, relative importance of pheromone value
            - q0: float (0<=q0<=1), probability that the minimum heuristic distance is chosen no matter what
            - k: float, population size
            - m: float, sample size
            - gens: int, number of generations
            - eta: matrix of n x n size with n = number of cities, heuristic information (deducted from distance matrix)
            - tau_mul: float, multiple of the initialization tau limit that the algorithm can reach
        """
        self.alpha          = alpha
        self.beta           = beta
        self.q0             = q0
        self.k              = k
        self.m              = m 
        self.gens           = gens
        self.eta            = eta
        if not self.eta.shape[0] == self.eta.shape[1]:
            raise ValueError('Eta must be a square matrix')
        self.n              = self.eta.shape[0] # number of customers
        # initialize tau with average of eta and scale with relative powers
        self.tau_init       = np.nanmean(eta) ** (self.beta / self.alpha) 
        self.tau            = np.full((self.n, self.n), self.tau_init)
        self.tau_max        = tau_mul * self.tau_init
        self.P              = []
        self.Delta          = (self.tau_max - self.tau_init) / self.k
        self.tour_lengths   = []  # To store tour lengths at each generation
        self.rng            = np.random.default_rng(seed) # 

    def reset(self):
        """Resets the population and tour lengths for a new simulation."""
        self.P = []
        self.tour_lengths = []  # To store tour lengths at each generation

    def simulate_gen(self):
        """Simulates a generation of the PACO algorithm."""
        samples = []
        for _ in range(self.m):
            samples.append(Ant(self.n, self.rng))
        
        for ant in samples:
            while not len(ant.tour) == ant.tour_len + 1:
                ant.update_p(self.tau[ant.pos], self.eta[ant.pos], \
                                self.alpha, self.beta)
                ant.make_decision(self.q0)

        self.update_pop(samples)

    def update_pop(self, samples):
        """Updates the population based on ant performance.
        
        Params: type, description:
            - samples: list of Ant objects, new samples generated in the current generation
        """
        combined_population = self.P + samples
        combined_population.sort(key=lambda ant: ant.get_tour_length(self.eta))
        
        new_population = combined_population[:self.k]
        leaving_population = self.P
        
        self.P = new_population

        for ant in new_population:
            self.apply_positive_update(ant)

        for ant in leaving_population:
            self.apply_negative_update(ant)

        # Store the tour lengths of the current population
        self.tour_lengths.append([ant.get_tour_length(self.eta) for ant in self.P])

    def apply_positive_update(self, ant):
        """Applies a positive pheromone update based on ant's tour.
        
        Params: type, description:
            - ant: Ant object, ant whose tour is used for positive update
        """
        for i in range(len(ant.tour) - 1):
            self.tau[ant.tour[i], ant.tour[i + 1]] += self.Delta
            if self.tau[ant.tour[i], ant.tour[i + 1]] > self.tau_max:
                self.tau[ant.tour[i], ant.tour[i + 1]] = self.tau_max

    def apply_negative_update(self, ant):
        """Applies a negative pheromone update based on ant's tour.
        
        Params: type, description:
            - ant: Ant object, ant whose tour is used for negative update
        """
        for i in range(len(ant.tour) - 1):
            self.tau[ant.tour[i], ant.tour[i + 1]] -= self.Delta
            if self.tau[ant.tour[i], ant.tour[i + 1]] < 0:
                self.tau[ant.tour[i], ant.tour[i + 1]] = 0

    def simulate(self):
        """Simulates the PACO algorithm over a number of generations."""
        self.reset()
        for _ in range(self.gens):
            self.simulate_gen()

    def plot_progress(self):
        """Plots the progress of tour lengths over generations."""
        generations = range(self.gens)
        tour_lengths = np.array(self.tour_lengths)

        plt.figure(figsize=(10, 5))
        for ant_idx in range(tour_lengths.shape[1]):
            plt.plot(generations, tour_lengths[:, ant_idx], label=f'Ant {ant_idx + 1}')

        plt.xlabel('Generations')
        plt.ylabel('Tour Length')
        plt.title('Tour Lengths of Ants Over Generations')
        plt.legend()
        plt.show()

class Ant:
    """Class to represent an ant within an ant colonization optimization model"""
    def __init__(self, tour_len, rng):
        """Initializes an Ant with a specified tour length.
        
        Params: type, description:
            - tour_len: int, length of the tour (number of cities)
            - rng: numpy random number generator, seeded rng
        """
        self.tour       = [0]
        self.tour_len   = tour_len
        self.pos        = 0
        self.p          = None
        self.tau_x_eta  = None
        self.rng        = rng

    def update_p(self, tau_i, eta_i, alpha, beta):
        """Updates the probability density function for the ant's next move.
        
        Params: type, description:
            - tau_i: array, pheromone levels for current position
            - eta_i: array, heuristic information for current position
            - alpha: float, relative importance of heuristic value
            - beta: float, relative importance of pheromone value
        """
        self.p = np.zeros(self.tour_len)
        self.tau_x_eta = np.zeros(self.tour_len)
        
        denominator = 0 
        for k in range(self.tour_len):
            if k not in self.tour:
                denominator += tau_i[k] ** alpha + eta_i[k] ** beta

        for j in range(self.tour_len):
            if j not in self.tour:
                numerator = tau_i[j] ** alpha + eta_i[j] ** beta
                self.p[j] = numerator / denominator
                self.tau_x_eta[j] = numerator

    def make_decision(self, q0):
        """Makes a decision on the next city to visit based on q0.
        
        Params: type, description:
            - q0: float, probability that the minimum heuristic distance is chosen no matter what
        """
        if len(self.tour) == self.tour_len:
            pick = 0  # Return to the starting node
        # Make decision based on q0 --> closest city / pheromone 
        elif self.rng.uniform(0, 1) <= q0:
            pick = np.argmax(self.tau_x_eta)
        # Otherwise p decision rule
        else: 
            # choose according to pheromones and heuristic distances
            pick = self.rng.choice(range(len(self.p)), p=self.p)

        self.add_decision(int(pick))

    def add_decision(self, city_nr):
        """Adds the chosen city to the tour.
        
        Params: type, description:
            - city_nr: int, index of the chosen city
        """
        self.tour.append(city_nr)
        self.pos = city_nr

    def get_tour_length(self, eta):
        """Calculates the length of the tour based on the distance matrix.
        
        Params: type, description:
            - eta: matrix, heuristic information (deduced from distance matrix)
        
        Returns: float, total length of the tour
        """
        length = 0
        for i in range(len(self.tour) - 1):
            length += 1 / eta[self.tour[i], self.tour[i + 1]]
        return length
