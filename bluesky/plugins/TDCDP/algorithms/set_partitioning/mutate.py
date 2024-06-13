import random 
# from funcs import sort_solution
from bluesky.plugins.TDCDP.algorithms.set_partitioning.funcs import sort_solution

def mutate(solution, max_customers_per_subset):
    """Initialization of a mutation. 
    Selects and commences a point, swap or merge/split mutation.
    Detailed descriptions of each mutation can be found in the respective functions.
    args: type, description
        - solution: list of lists, solution representing allocations of the customers
        - max_customers_per_subset: int, max. nr. of custs per cluster"""
    mutation_type = random.choice(['point', 'swap', 'mergesplit'])
    if mutation_type == 'point':
        return point_mutation(solution, max_customers_per_subset)
    elif mutation_type == 'swap':
        return swap_mutation(solution)
    elif mutation_type == 'mergesplit':
        return mergesplit_mutation(solution, max_customers_per_subset)

def point_mutation(solution, max_customers_per_subset):
    """Performs a point mutation on a solution.
    Picks an item from a certain cluster and places it in another cluster.
    args: type, description
        - solution: list of lists, solution representing allocations of the customers
        - max_customers_per_subset: int, max. nr. of custs per cluster"""
    # Select a random subset (cluster) that is not empty
    non_empty_clusters = sort_solution([subset for subset in solution if subset])
    if not non_empty_clusters:
        return solution

    from_subset = random.choice(non_empty_clusters)
    if len(from_subset) == 0:
        return solution

    # Select a random customer from this subset
    customer = random.choice(from_subset)
    from_subset.remove(customer)

    # Select a random destination subset or create a new one
    # the odds of adding an additional one should be as high as adding the customer to any other cluster
    # Therefore only 1 empty list (subset) should be included
    # Get first empty list of solution
    non_empty_clusters.append(solution[solution.index([])])
    to_subset = random.choice(non_empty_clusters)

    # Add the customer to the destination subset if it does not violate constraints
    if len(to_subset) < max_customers_per_subset:
        to_subset.append(customer)
    else:
        # Put the customer back if we can't move it
        from_subset.append(customer)
    return sort_solution(solution)

def swap_mutation(solution):
    """Performs a swap mutation on a solution.
    Takes two points, each from different clusters and swaps these around.
    args: type, description
        - solution: list of lists, solution representing allocations of the customers"""
    # Select two different clusters
    non_empty_clusters = [subset for subset in solution if len(subset) > 0]
    if len(non_empty_clusters) < 2:
        return solution
    
    cluster1, cluster2 = random.sample(non_empty_clusters, 2)
    
    # Select a random point from each cluster
    point1 = random.choice(cluster1)
    point2 = random.choice(cluster2)
    
    # Swap the points between the clusters
    cluster1.remove(point1)
    cluster2.remove(point2)
    
    cluster1.append(point2)
    cluster2.append(point1)
    
    return sort_solution(solution)

def mergesplit_mutation(solution, max_customers_per_subset):
    """Performs either a merge or a split mutation on a solution.
    Merging will group two clusters together, splitting will split a cluster in two
    args: type, description
        - solution: list of lists, solution representing allocations of the customers
        - max_customers_per_subset: int, max. nr. of custs per cluster"""
    mutation_type = random.choice(['merge','split'])
    if mutation_type == 'merge':
        # Select two different non-empty clusters to merge
        non_empty_clusters = [subset for subset in solution if subset]
        if len(non_empty_clusters) < 2:
            return solution
        
        cluster1, cluster2 = random.sample(non_empty_clusters, 2)
        
        # Check if the merge exceeds the maximum size constraint
        if len(cluster1) + len(cluster2) <= max_customers_per_subset:
            cluster1.extend(cluster2)
            solution.remove(cluster2)
        
        return sort_solution(solution)
    elif mutation_type == 'split':
        # Select a cluster to split
        non_empty_clusters = [subset for subset in solution if subset]
        if not non_empty_clusters:
            return solution
        
        cluster = random.choice(non_empty_clusters)
        
        if len(cluster) <= 1:
            return solution  # Cannot split a single or empty cluster
        
        split_point = len(cluster) // 2
        new_cluster = cluster[split_point:]
        cluster = cluster[:split_point]
        
        solution.append(new_cluster)
        
        return sort_solution(solution)