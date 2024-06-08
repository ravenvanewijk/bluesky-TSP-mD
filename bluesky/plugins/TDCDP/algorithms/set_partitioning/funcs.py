import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors
import numpy as np

def euclidean_distance(p1, p2):
    # return np.sqrt(np.sum((np.array(p1) - np.array(p2))**2))
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2)

def sort_solution(solution):
    # Sort the solutions
    sorted_sol = sorted(solution, key=lambda solution: (not solution, solution[0] if solution else float('inf')))
    return sorted_sol

def flatten_solution(solution, n_points):
    """Flatten the nested list of subsets into a single list of customer assignments."""
    flat_solution = [None] * n_points
    for subset_index, subset in enumerate(solution):
        for customer in subset:
            flat_solution[customer] = subset_index
    return flat_solution

def unflatten_solution(flat_solution, n_points):
    """Convert a flat list of customer assignments back into a nested list of subsets."""
    subsets = [[] for _ in range(n_points)]
    for customer, subset_index in enumerate(flat_solution):
        subsets[subset_index].append(customer)
    return subsets

def get_covered_elements(solution):
    """Extract the subsets covered by a solution as sets of elements."""
    covered_elements = [set(subset) for subset in solution if subset]
    return covered_elements

def calculate_centroids(solution, customers):
    centroids = []
    for subset in solution:
        if len(subset) > 0:
            if not isinstance(customers, np.ndarray):
                customers = np.array(customers)
            cluster_points = customers[subset]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)
        else:
            pass
    return centroids

def plot_custlocs(custlocs, clusters):
    # Extract latitudes and longitudes
    latitudes = [location[0] for location in custlocs]
    longitudes = [location[1] for location in custlocs]



    unique_clusters = np.unique(clusters)
    cmap = cm.get_cmap('tab20', len(unique_clusters))
    norm = colors.Normalize(vmin=0, vmax=len(unique_clusters) - 1)

    for cluster_id in unique_clusters:
        indices = np.where(clusters == cluster_id)[0]
        plt.scatter([longitudes[i] for i in indices], 
                    [latitudes[i] for i in indices], 
                    c=np.array([cmap(norm(cluster_id))]), 
                    marker='o', s=50, label=f'Cluster {cluster_id}')
    plt.legend()
    plt.show()