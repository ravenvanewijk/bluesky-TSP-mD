import optuna
import numpy as np
from PACO import PACO, eta

# Define the objective function to minimize
def objective(trial):
    # Sample hyperparameters from the trial
    alpha = trial.suggest_loguniform('alpha', 1e-3, 1e2)
    beta = trial.suggest_loguniform('beta', 1e-3, 1e2)
    q0 = trial.suggest_uniform('q0', 0.0, 1.0)
    k = trial.suggest_int('k', 5, 50)  # population size
    m = trial.suggest_int('m', 10, 50)  # sample size
    gens = trial.suggest_int('gens', 10, 200)  # number of generations
    tau_mul = trial.suggest_loguniform('tau_mul', 1.0, 10.0)
    
    seeds = np.random.randint(0, 10000, 5)  # Generate 5 random seeds
    tour_lengths = []

    for seed in seeds:
        # Initialize the PACO model with sampled hyperparameters and the current seed
        paco = PACO(
            alpha=alpha,
            beta=beta,
            q0=q0,
            k=k,
            m=m,
            gens=gens,
            eta=eta,
            tau_mul=tau_mul,
            seed=seed
        )

        # Run the PACO simulation
        paco.simulate()

        # Record the best tour length for this seed
        best_tour_length = min(min(lengths) for lengths in paco.tour_lengths)
        tour_lengths.append(best_tour_length)
    
    # Average the best tour lengths across all seeds
    avg_best_tour_length = np.mean(tour_lengths)
    return avg_best_tour_length

# Create an Optuna study and optimize
study = optuna.create_study(direction='minimize')  # Or 'maximize' if you need to maximize the metric
study.optimize(objective, n_trials=100)

# Print the best hyperparameters and the best score
print("Best hyperparameters:")
for key, value in study.best_params.items():
    print(f"{key}: {value}")

print(f"Best score: {study.best_value}")
"""
RUN 1:
Best hyperparameters:
alpha: 2.366302356710575
beta: 10.217606441836558
q0: 0.8904832390914627
k: 21
m: 50
gens: 62
tau_mul: 4.384013734477324
Best score: 6068.791202067266


"""
