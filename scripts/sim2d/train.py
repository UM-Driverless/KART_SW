#!/usr/bin/env python3
"""Main entry point — runs GA training for kart controllers."""

import argparse
import json
import multiprocessing
import os
import time

from controllers import GeometricController, NeuralNetController, NeuralNetV2Controller
from ga import GeneticAlgorithm
from sim import run_episode, set_track

CONTROLLER_MAP = {
    "geometric": GeometricController,
    "neural": NeuralNetController,
    "neural_v2": NeuralNetV2Controller,
}


def main():
    parser = argparse.ArgumentParser(description="Train kart controllers with GA")
    parser.add_argument("--generations", type=int, default=50)
    parser.add_argument("--pop-size", type=int, default=100)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--output-dir", type=str, default="results")
    parser.add_argument("--controllers", type=str, default="geometric,neural",
                        help="Comma-separated controller types: "
                             "geometric, neural, neural_v2")
    parser.add_argument("--fitness", type=str, default="v1",
                        choices=["v1", "v2", "v3", "v4", "v5"],
                        help="v1: distance+laps, v2: lap-time, "
                             "v3: track-keeping (nonlinear CTE penalty), "
                             "v4: boundary-aware (terminate outside cones)")
    parser.add_argument("--seed", type=str, default="",
                        help="Path to JSON weights to seed population with")
    parser.add_argument("--track", type=str, default="oval",
                        choices=["oval", "hairpin", "autocross"],
                        help="Track to train on (default: oval)")
    parser.add_argument("--max-steps", type=int, default=2000,
                        help="Max simulation steps per episode (default: 2000)")
    parser.add_argument("--sigma", type=float, default=0.1,
                        help="Initial mutation sigma (default: 0.1, use 0.01-0.02 for fine-tuning)")
    parser.add_argument("--sigma-min", type=float, default=0.005,
                        help="Minimum mutation sigma")
    args = parser.parse_args()

    # Set active track BEFORE forking workers
    set_track(args.track, max_steps=args.max_steps)

    os.makedirs(args.output_dir, exist_ok=True)

    names = [n.strip() for n in args.controllers.split(",")]
    gas = {}
    for name in names:
        cls = CONTROLLER_MAP[name]
        ga = GeneticAlgorithm(cls, pop_size=args.pop_size,
                              fitness_mode=args.fitness,
                              mutation_sigma=args.sigma,
                              sigma_min=args.sigma_min)
        if args.seed:
            import numpy as np
            with open(args.seed) as f:
                seed_data = json.load(f)
            seed_genes = np.array(seed_data["genes"], dtype=np.float64)
            # Seed entire population around the seed weights
            seed_sigma = args.sigma
            n_seed = min(args.pop_size, max(20, args.pop_size // 2))
            ga.population[0] = seed_genes.copy()
            for i in range(1, n_seed):
                ga.population[i] = seed_genes + np.random.randn(len(seed_genes)) * seed_sigma
            print(f"  Seeded {name} from {args.seed} (fitness={seed_data.get('fitness', '?')})")
            print(f"  {n_seed}/{args.pop_size} slots seeded, σ={seed_sigma}")
        gas[name] = ga

    print(f"Training {list(gas.keys())} on '{args.track}' track | "
          f"generations={args.generations}  pop={args.pop_size}  "
          f"workers={args.workers}  fitness={args.fitness}\n")

    for gen in range(args.generations):
        t0 = time.time()
        for name, ga in gas.items():
            ga.evaluate(workers=args.workers)
            s = ga.stats()
            dt = time.time() - t0
            print(f"[{name:>10}] gen {gen:3d} | "
                  f"best={s['best']:8.1f}  avg={s['avg']:8.1f} | "
                  f"all-time={s['all_time_best']:8.1f} | "
                  f"σ={s['sigma']:.4f} | {dt:.1f}s")
            ga.evolve()
        print()

    # Save best controllers
    for name, ga in gas.items():
        ctrl = ga.controller_class(ga.best_genes)
        result = run_episode(ctrl, fitness_mode=args.fitness)

        payload = {
            "controller_type": name,
            "track": args.track,
            "genes": ga.best_genes.tolist(),
            "fitness": ga.best_fitness,
            "fitness_mode": args.fitness,
            "result": result,
            "generations": args.generations,
            "pop_size": args.pop_size,
        }

        path = os.path.join(args.output_dir, f"best_{name}.json")
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)

        print(f"Saved {name} → {path}")
        print(f"  fitness={ga.best_fitness:.1f}  "
              f"dist={result['distance']:.1f}m  laps={result['laps']}  "
              f"avg_cte={result['avg_cte']:.2f}m  max_cte={result['max_cte']:.2f}m  "
              f"avg_speed={result['avg_speed']:.1f}m/s  "
              f"time={result['time']:.1f}s"
              + (f"  cte_pen={result['cte_penalty']:.1f}"
                 if result.get('cte_penalty') else ""))
        if name == "geometric":
            g = ga.best_genes
            print(f"  genes: gain={g[0]:.3f} max_steer={g[1]:.3f} "
                  f"max_speed={g[2]:.3f} min_speed={g[3]:.3f} "
                  f"lookahead={g[4]:.3f} curve_factor={g[5]:.3f}")
        print()


if __name__ == "__main__":
    multiprocessing.set_start_method("fork")
    main()
