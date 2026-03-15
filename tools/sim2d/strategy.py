"""Training strategy definition. THIS IS THE FILE THE AGENT EDITS."""

import time
import numpy as np
from multiprocessing import Pool

from sim import run_episode, set_track

_fitness_mode = "v2"
_ctrl_class = None  # set before each Pool fork


def _eval_one(genes):
    ctrl = _ctrl_class(genes)
    return run_episode(ctrl, fitness_mode=_fitness_mode)["fitness"]


def _init_worker(cls):
    global _ctrl_class
    _ctrl_class = cls


def _run_de(ctrl_class, seed, spread, pop_size, F, CR, budget):
    """Run DE optimization, return (best_genes, best_fitness, gens)."""
    num_genes = ctrl_class.NUM_GENES

    population = np.tile(seed, (pop_size, 1))
    population += np.random.randn(pop_size, num_genes) * spread
    population[0] = seed

    with Pool(8, initializer=_init_worker, initargs=(ctrl_class,)) as pool:
        fitnesses = np.array(pool.map(_eval_one, [population[i] for i in range(pop_size)]))

    best_idx = int(np.argmax(fitnesses))
    best_fitness = fitnesses[best_idx]
    best_genes = population[best_idx].copy()

    t0 = time.time()
    gen = 0
    while time.time() - t0 < budget:
        trials = np.empty_like(population)
        for i in range(pop_size):
            idxs = list(range(pop_size))
            idxs.remove(i)
            a, b, c = np.random.choice(idxs, 3, replace=False)
            mutant = population[a] + F * (population[b] - population[c])
            trial = population[i].copy()
            j_rand = np.random.randint(num_genes)
            for j in range(num_genes):
                if np.random.random() < CR or j == j_rand:
                    trial[j] = mutant[j]
            trials[i] = trial

        with Pool(8, initializer=_init_worker, initargs=(ctrl_class,)) as pool:
            trial_fitnesses = np.array(pool.map(_eval_one, [trials[i] for i in range(pop_size)]))

        for i in range(pop_size):
            if trial_fitnesses[i] > fitnesses[i]:
                population[i] = trials[i]
                fitnesses[i] = trial_fitnesses[i]
                if trial_fitnesses[i] > best_fitness:
                    best_fitness = trial_fitnesses[i]
                    best_genes = trials[i].copy()

        gen += 1

    return best_genes, best_fitness, gen


def train_controller(time_budget: float = 60.0):
    """Race Geometric vs PurePursuit with DE (40s geo, 20s pp)."""
    set_track("autocross", max_steps=2000)

    from controllers import PurePursuitController, GeometricController

    # Geometric gets 2/3 of budget (it's proven faster)
    # Seed from optimized genes: sg=1.07, lookahead=5, min_speed=6.4
    geo_seed = np.array([1.0, 0.5, 100.0, 6.0, 5.0, 0.05])
    geo_spread = np.array([0.3, 0.1, 30.0, 2.0, 2.0, 0.05])
    genes, fit, gens = _run_de(
        GeometricController, geo_seed, geo_spread, 40, 0.8, 0.9, time_budget
    )

    best_ctrl = GeometricController(genes)

    return best_ctrl, {
        "generations": gens,
        "best_fitness": fit,
        "controller_type": "geometric",
        "optimizer": "de",
    }
