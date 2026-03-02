"""Genetic algorithm & CMA-ES: evaluate, select, crossover, mutate."""

import numpy as np
from multiprocessing import Pool

from sim import run_episode, run_episode_multitrack, _active_tracks

# Module-level fitness_mode so worker processes can see it (set by GA)
_fitness_mode = "v1"
_use_multitrack = False


def _evaluate_one(args):
    """Top-level worker function (must be picklable for multiprocessing)."""
    controller_class, genes = args
    ctrl = controller_class(genes)
    if _use_multitrack:
        return run_episode_multitrack(ctrl, fitness_mode=_fitness_mode)["fitness"]
    return run_episode(ctrl, fitness_mode=_fitness_mode)["fitness"]


class GeneticAlgorithm:
    """Standard GA with tournament selection, uniform crossover, Gaussian mutation."""

    def __init__(self, controller_class, pop_size=100, elite_size=5,
                 tournament_k=5, mutation_sigma=0.1, mutation_decay=0.995,
                 sigma_min=0.005, sigma_restart_patience=30,
                 fitness_mode="v1"):
        self.controller_class = controller_class
        self.pop_size = pop_size
        self.elite_size = elite_size
        self.tournament_k = tournament_k
        self.mutation_sigma = mutation_sigma
        self.initial_sigma = mutation_sigma
        self.mutation_decay = mutation_decay
        self.sigma_min = sigma_min
        self.sigma_restart_patience = sigma_restart_patience
        self.num_genes = controller_class.NUM_GENES
        self.fitness_mode = fitness_mode

        self.population = self._init_population()
        self.fitnesses = np.zeros(pop_size)
        self.generation = 0
        self.best_fitness = -np.inf
        self.best_genes = None
        self._stagnation_counter = 0
        self._last_best = -np.inf

    # ── initialisation ────────────────────────────────────────────────

    def _init_population(self):
        if hasattr(self.controller_class, "RANGES"):
            # Geometric: sample uniformly inside gene ranges, keep defaults
            ranges = self.controller_class.RANGES
            pop = np.random.uniform(
                ranges[:, 0], ranges[:, 1],
                size=(self.pop_size, self.num_genes),
            )
            pop[0] = self.controller_class.DEFAULTS  # ensure defaults in pool
            return pop
        # Neural net: Xavier-like
        scale = 1.0 / np.sqrt(self.controller_class.INPUT_SIZE)
        return np.random.randn(self.pop_size, self.num_genes) * scale

    # ── evaluation ────────────────────────────────────────────────────

    def evaluate(self, workers=1):
        global _fitness_mode, _use_multitrack
        _fitness_mode = self.fitness_mode
        _use_multitrack = _active_tracks is not None

        args = [(self.controller_class, self.population[i])
                for i in range(self.pop_size)]

        if workers > 1:
            with Pool(workers) as pool:
                self.fitnesses = np.array(pool.map(_evaluate_one, args))
        else:
            self.fitnesses = np.array([_evaluate_one(a) for a in args])

        best_idx = int(np.argmax(self.fitnesses))
        if self.fitnesses[best_idx] > self.best_fitness:
            self.best_fitness = float(self.fitnesses[best_idx])
            self.best_genes = self.population[best_idx].copy()

    # ── selection / variation ─────────────────────────────────────────

    def _tournament(self):
        idxs = np.random.choice(self.pop_size, self.tournament_k, replace=False)
        return self.population[idxs[np.argmax(self.fitnesses[idxs])]]

    def _crossover(self, p1, p2):
        mask = np.random.random(self.num_genes) < 0.5
        return np.where(mask, p1, p2)

    def _mutate(self, ind):
        child = ind + np.random.randn(self.num_genes) * self.mutation_sigma
        if hasattr(self.controller_class, "RANGES"):
            lo = self.controller_class.RANGES[:, 0]
            hi = self.controller_class.RANGES[:, 1]
            child = np.clip(child, lo, hi)
        return child

    # ── evolution step ────────────────────────────────────────────────

    def evolve(self):
        order = np.argsort(-self.fitnesses)
        new_pop = np.empty_like(self.population)

        # Elitism
        new_pop[:self.elite_size] = self.population[order[:self.elite_size]]

        # Offspring
        for i in range(self.elite_size, self.pop_size):
            new_pop[i] = self._mutate(self._crossover(
                self._tournament(), self._tournament()))

        self.population = new_pop
        self.mutation_sigma = max(self.mutation_sigma * self.mutation_decay,
                                   self.sigma_min)

        # Sigma restart: if best fitness hasn't improved, boost sigma
        current_best = float(np.max(self.fitnesses))
        if current_best > self._last_best + 0.1:
            self._stagnation_counter = 0
            self._last_best = current_best
        else:
            self._stagnation_counter += 1
        if (self.sigma_restart_patience > 0
                and self._stagnation_counter >= self.sigma_restart_patience):
            self.mutation_sigma = self.initial_sigma * 0.5
            self._stagnation_counter = 0

        self.generation += 1

    # ── reporting ─────────────────────────────────────────────────────

    def stats(self):
        return {
            "generation": self.generation,
            "best": float(np.max(self.fitnesses)),
            "avg": float(np.mean(self.fitnesses)),
            "worst": float(np.min(self.fitnesses)),
            "all_time_best": self.best_fitness,
            "sigma": self.mutation_sigma,
        }


# ── CMA-ES optimizer ────────────────────────────────────────────────────

class CMAESOptimizer:
    """CMA-ES wrapper using the ``cma`` package.

    Much more sample-efficient than basic GA for continuous optimization
    in the 100–1000 parameter range.
    """

    def __init__(self, controller_class, pop_size=None, sigma0=0.3,
                 fitness_mode="v1", seed_genes=None):
        import cma
        self.controller_class = controller_class
        self.num_genes = controller_class.NUM_GENES
        self.fitness_mode = fitness_mode

        if seed_genes is not None:
            x0 = np.asarray(seed_genes, dtype=np.float64)
        else:
            scale = 1.0 / np.sqrt(controller_class.INPUT_SIZE)
            x0 = np.random.randn(self.num_genes) * scale

        opts = {"verb_disp": 0, "verb_log": 0}
        if pop_size is not None:
            opts["popsize"] = pop_size
        self.es = cma.CMAEvolutionStrategy(x0, sigma0, opts)
        self.pop_size = self.es.popsize
        self.generation = 0
        self.best_fitness = -np.inf
        self.best_genes = None
        self.fitnesses = np.zeros(self.pop_size)
        self._solutions = None

    def evaluate(self, workers=1):
        global _fitness_mode, _use_multitrack
        _fitness_mode = self.fitness_mode
        _use_multitrack = _active_tracks is not None

        self._solutions = self.es.ask()
        args = [(self.controller_class, np.array(s))
                for s in self._solutions]

        if workers > 1:
            with Pool(workers) as pool:
                self.fitnesses = np.array(pool.map(_evaluate_one, args))
        else:
            self.fitnesses = np.array([_evaluate_one(a) for a in args])

        best_idx = int(np.argmax(self.fitnesses))
        if self.fitnesses[best_idx] > self.best_fitness:
            self.best_fitness = float(self.fitnesses[best_idx])
            self.best_genes = np.array(self._solutions[best_idx]).copy()

    def evolve(self):
        # CMA-ES minimizes, so negate fitness
        self.es.tell(self._solutions, (-self.fitnesses).tolist())
        self.generation += 1

    def stats(self):
        return {
            "generation": self.generation,
            "best": float(np.max(self.fitnesses)),
            "avg": float(np.mean(self.fitnesses)),
            "worst": float(np.min(self.fitnesses)),
            "all_time_best": self.best_fitness,
            "sigma": self.es.sigma,
        }

    @property
    def population(self):
        """For compatibility with seeding."""
        return np.array(self._solutions) if self._solutions else np.zeros((self.pop_size, self.num_genes))
