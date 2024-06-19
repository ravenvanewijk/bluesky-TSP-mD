import numpy as np
import random
# from mutate import mutate
# from funcs import euclidean_distance, sort_solution, \
#                     flatten_solution, unflatten_solution,\
#                     get_covered_elements, plot_custlocs,\
#                     calculate_centroids
from bluesky.plugins.TDCDP.algorithms.set_partitioning.mutate import mutate
from bluesky.plugins.TDCDP.algorithms.set_partitioning.funcs import \
                    euclidean_distance, sort_solution, \
                    flatten_solution, unflatten_solution,\
                    get_covered_elements, plot_custlocs,\
                    calculate_centroids

def initialize_population(population_size, customers):
    """Begin the GA with an initial set of solutions, which randomly assigns customers to a cluster.
    args: type, description
        - population_size: int, number of solutions in each generation
        - customers: list of lists/ tuples, list of coord pairs (lat, lon)"""
    population = []
    for _ in range(population_size):
        solution = [[] for _ in range(len(customers))]
        for i in range(len(customers)):
            subset = random.choice(solution)
            subset.append(i)
        population.append(sort_solution(solution))
    return population

def fitness(solution, customers):
    """Evaluate the fitness of a given solution.
    args: type, description
        - solution: list of lists, solution representing allocations of the customers
        - customers: list of lists/ tuples, list of coord pairs (lat, lon)"""
    num_clusters = len([subset for subset in solution if len(subset) > 0])
    total_distance = 0

    for idx, subset in enumerate(solution):
        if len(subset) >= 1:
            # Calculate intra-cluster distances
            if len(subset) > 1:
                intra_cluster_distances = np.sum([
                    euclidean_distance(customers[subset[i]], customers[subset[j]])**2
                    for i in range(len(subset)) for j in range(i + 1, len(subset))
                ])
                total_distance += intra_cluster_distances

    return num_clusters + np.sqrt(total_distance)

def unfitness(solution, customers, max_customers_per_subset, max_distance):
    """Evaluate the unfitness of a solution, which is based on the violation of constraints.
    Constraints are the max number of customers in a subset, and the max distance between customers in a cluster.
    args: type, description
        - solution: list of lists, solution representing allocations of the customers
        - customers: list of lists/ tuples, list of coord pairs (lat, lon)
        - max_customers_per_subset: int, max. nr. of custs per cluster
        - max_distance: int/float, max. distance between two points in the same cluster
    """
    penalty = 0
    cust_in_sol = set()
    for subset in solution:
        if len(subset) > max_customers_per_subset:
            penalty += (len(subset) - max_customers_per_subset) * 100
        for i in range(len(subset)):
            cust_in_sol.add(subset[i])
            for j in range(i + 1, len(subset)):
                if euclidean_distance(customers[subset[i]], customers[subset[j]]) > max_distance:
                    penalty += 100
    if not set(range(len(customers))) == cust_in_sol:
        penalty += 100
    return penalty

def uniform_crossover(parent1, parent2, n_custs):
    """Perform uniform crossover between two parents to produce two children. 
    The child inherits both characteristics of the first and second parent.
    args: type, description
        - parent1: list of lists, solution representing allocations of the customers
        - parent2: list of lists, solution representing allocations of the customers
        - n_custs: int, number of customers to be partitioned
    """
    # Flatten the parent solutions to work with customer assignments directly
    flat_parent1 = flatten_solution(parent1, n_custs)
    flat_parent2 = flatten_solution(parent2, n_custs)

    flat_child1 = []
    flat_child2 = []

    # Create first child
    for c1, c2 in zip(flat_parent1, flat_parent2):
        if random.random() < 0.5:
            flat_child1.append(c1)
        else:
            flat_child1.append(c2)

    # Create second child
    for c1, c2 in zip(flat_parent1, flat_parent2):
        if random.random() < 0.5:
            flat_child2.append(c1)
        else:
            flat_child2.append(c2)

    # Convert flat child assignments back into nested subsets
    child1 = unflatten_solution(flat_child1, len(parent1))
    child2 = unflatten_solution(flat_child2, len(parent1))
    
    return sort_solution(child1), sort_solution(child2)


def select_parents(population, customers, max_customers_per_subset, max_distance, tournament_size=10):
    """Select the two parents that will get children.
    args: type, description
        - population_size: int, number of solutions in each generation
        - customers: list of lists/ tuples, list of coord pairs (lat, lon)
        - max_customers_per_subset: int, max. nr. of custs per cluster
        - max_distance: int/float, max. distance between two points in the same cluster
        - tournament_size: selected number of candidates to pick from
    """
    # First parent selection by binary tournament
    parent1 = tournament_selection(population, customers, tournament_size, 
                                max_customers_per_subset, max_distance)[0]
    # Second parent selection by compatibility score
    candidate_parents = [sol for sol in population if sol != parent1]
    # parent2 = max(candidate_parents, key=lambda sol: (compatibility_score(parent1, sol), -fitness(sol)))
    parent2 = tournament_selection(population, customers, tournament_size, \
                                   max_customers_per_subset, max_distance)[0]

    return parent1, parent2

def tournament_selection(population, customers, tournament_size, \
                         max_customers_per_subset, max_distance, n_best=1):
    """Perform a tournament selection based on the performance of a solution in terms of its fitness and unfitness
    args: type, description
        - population_size: int, number of solutions in each generation
        - customers: list of lists/ tuples, list of coord pairs (lat, lon)
        - tournament_size: selected number of candidates to pick from
        - max_customers_per_subset: int, max. nr. of custs per cluster
        - max_distance: int/float, max. distance between two points in the same cluster
    """
    # Make a copy of the population to avoid modifying the original list
    population_copy = population[:]
    selected_individuals = []
    
    for i in range(n_best):
        tournament = random.sample(population_copy, tournament_size - i)
        best_individual = min(tournament, key=lambda sol: \
                            (unfitness(sol, customers, max_customers_per_subset, max_distance), \
                            fitness(sol, customers)))
        selected_individuals.append(best_individual)
        population_copy.remove(best_individual)  # Remove from the copy, not the original
    
    return selected_individuals

def compatibility_score(parent1, candidate):
    """Calculate the compatibility score between parent1 and a candidate."""
    RP1 = get_covered_elements(parent1)
    RSi = get_covered_elements(candidate)
    
    uniques = len(np.unique(np.append(RP1, RSi)))
    union_count = uniques
    intersection_count = len(np.append(RP1, RSi)) - uniques
    
    return union_count - intersection_count


def SP_GA(customers, max_customers_per_subset, max_distance,
                        generations=200, mutation_rate=0.2,\
                        population_size=100, p_elitism=0.02):
    """Run the genetic algorithm to perform set partitioning. 
    The goal of the algorithm is to divide a given set of customers into clusters of a max size, 
    with max inbetween distance. The focus is mininmization of number of clusters and intra cluster distance.
    args: type, description
        - customers: list of lists/ tuples, list of coord pairs (lat, lon)
        - max_customers_per_subset: int, max. nr. of custs per cluster
        - max_distance: int/float, max. distance between two points in the same cluster
        - generations: int, the number of generations to run the algorithm for
        - mutation_rate: float, the odds of a mutation occuring
        - population_size: int, number of solutions in each generation
        - p_elitism: float, the % of solutions from previous generation that will make next gen no matter what
    """
    population = initialize_population(population_size, customers)
    for generation in range(generations):
        new_population = []
        # implement elitism: fittest p_elitsm % reach the next stage no matter what
        elites = tournament_selection(population, customers, len(population),\
                                    max_customers_per_subset, max_distance,\
                                    n_best=int(p_elitism*len(population)))
        new_population.extend(elites)
        for i in range(population_size // 2 - int(p_elitism*len(population))):
            parent1, parent2 = select_parents(population, customers, max_customers_per_subset, max_distance, 5)
            child1, child2 = uniform_crossover(parent1, parent2, len(customers))

            if random.random() < mutation_rate:
                child1 = mutate(child1, max_customers_per_subset)
            if random.random() < mutation_rate:
                child2 = mutate(child2, max_customers_per_subset)
            new_population.extend([child1, child2])
        bestyet = min(population, key=lambda sol: \
                                (unfitness(sol, customers, max_customers_per_subset, max_distance), \
                                fitness(sol, customers))) 
        bestfitnessyet = fitness(min(population, key=lambda sol: \
                                (unfitness(sol, customers, max_customers_per_subset, max_distance), \
                                fitness(sol, customers))), customers)
        # print(f"best current: {bestyet} with a fitness score of {bestfitnessyet}")
        population = new_population
    best_solution = min(population, key=lambda sol: \
                        (unfitness(sol, customers, max_customers_per_subset, max_distance), \
                        fitness(sol, customers)))
    return flatten_solution(best_solution, len(customers)), \
            calculate_centroids(best_solution, customers), \
            fitness(best_solution, customers)




# customers = [[42.920953, -78.74451], [42.937174, -78.777122], [42.959024, -78.719749], [42.934697, -78.724611], [42.909251, -78.811514], [42.932909, -78.724395], [42.871019, -78.867715], [42.887831, -78.838013], [42.958582, -78.887663], [42.956442, -78.781567], [42.919539, -78.727003], [42.892684, -78.722783], [42.996516, -78.781718], [42.87093, -78.883244], [42.89881, -78.889832], [42.994805, -78.87042], [42.992542, -78.818319], [42.900025, -78.885804], [42.911921, -78.74319], [42.970791, -78.85052], [42.985851, -78.727206], [42.997993, -78.733863], [42.969633, -78.810666], [42.852731, -78.815075], [42.922389, -78.788938]]

# # Run the genetic algorithm
# best_solution, allocations, score = SP_GA(
#                                             customers,
#                                             4, 
#                                             100,
#                                             250, 
#                                             0.2, 
#                                             100,
#                                             0.02,
#                                             )
# print(f"Best solution found: {best_solution} with a fitness value of {score}")
# plot_custlocs(customers, best_solution)


# def profile_geneticalgo():
#     best_solution, allocations, _ = SP_GA(
#                                             customers,
#                                             4, 
#                                             100,
#                                             1, 
#                                             0.2, 
#                                             100,
#                                             0.02,
#                                             )
#     print(f"Fitness Value: {best_solution}")

# if __name__ == "__main__":
#     import cProfile
#     import pstats
#     profiler = cProfile.Profile()
#     profiler.enable()
#     profile_geneticalgo()
#     profiler.disable()
    
#     stats = pstats.Stats(profiler).strip_dirs().sort_stats('cumtime')
#     stats.print_stats()

