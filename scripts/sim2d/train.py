#!/usr/bin/env python3
"""Main entry point — runs GA training for kart controllers."""

import argparse
import json
import multiprocessing
import os
import time

from controllers import (GeometricController, NeuralNetController,
                         NeuralNetV2Controller, NeuralNetV3Controller)
from ga import GeneticAlgorithm, CMAESOptimizer
from sim import run_episode, run_episode_multitrack, set_track
from track import track_to_json, get_track as _get_track

CONTROLLER_MAP = {
    "geometric": GeometricController,
    "neural": NeuralNetController,
    "neural_v2": NeuralNetV2Controller,
    "neural_v3": NeuralNetV3Controller,
}


def main():
    parser = argparse.ArgumentParser(description="Train kart controllers with GA")
    parser.add_argument("--generations", type=int, default=50)
    parser.add_argument("--pop-size", type=int, default=100)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--output-dir", type=str, default="results")
    parser.add_argument("--controllers", type=str, default="geometric,neural",
                        help="Comma-separated controller types: "
                             "geometric, neural, neural_v2, neural_v3")
    parser.add_argument("--fitness", type=str, default="v1",
                        choices=["v1", "v2", "v3", "v4", "v5", "v6"],
                        help="v1: distance+laps, v2: lap-time, "
                             "v3: track-keeping (nonlinear CTE penalty), "
                             "v4: boundary-aware (terminate outside cones)")
    parser.add_argument("--seed", type=str, default="",
                        help="Path to JSON weights to seed population with")
    parser.add_argument("--track", type=str, default="oval",
                        help="Track: built-in name (oval, hairpin, autocross), "
                             "JSON path, or random spec (random:seed=42). "
                             "Comma-separated for multi-track.")
    parser.add_argument("--max-steps", type=int, default=2000,
                        help="Max simulation steps per episode (default: 2000)")
    parser.add_argument("--sigma", type=float, default=0.1,
                        help="Initial mutation sigma (default: 0.1, use 0.01-0.02 for fine-tuning)")
    parser.add_argument("--sigma-min", type=float, default=0.005,
                        help="Minimum mutation sigma")
    parser.add_argument("--optimizer", type=str, default="ga",
                        choices=["ga", "cma"],
                        help="Optimizer: ga (genetic algorithm) or cma (CMA-ES)")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="Perception noise std in metres (default: 0, try 0.1-0.3)")
    parser.add_argument("--dropout", type=float, default=0.0,
                        help="Cone dropout probability (default: 0, try 0.05-0.15)")
    args = parser.parse_args()

    # Set active track BEFORE forking workers
    set_track(args.track, max_steps=args.max_steps,
              noise_std=args.noise, dropout=args.dropout)

    os.makedirs(args.output_dir, exist_ok=True)

    # Auto-save random tracks for reproducibility
    for track_spec in args.track.split(","):
        track_spec = track_spec.strip()
        if track_spec == "random" or track_spec.startswith("random:"):
            trk = _get_track(track_spec)
            save_path = os.path.join(args.output_dir, f"track_{trk.name}.json")
            track_to_json(trk, save_path)
            print(f"Saved random track → {save_path}")

    names = [n.strip() for n in args.controllers.split(",")]
    gas = {}
    for name in names:
        cls = CONTROLLER_MAP[name]

        # Load seed genes if provided
        seed_genes = None
        if args.seed:
            import numpy as np
            with open(args.seed) as f:
                seed_data = json.load(f)
            seed_genes = np.array(seed_data["genes"], dtype=np.float64)

        if args.optimizer == "cma":
            opt = CMAESOptimizer(cls, pop_size=args.pop_size, sigma0=args.sigma,
                                 fitness_mode=args.fitness,
                                 seed_genes=seed_genes)
            if seed_genes is not None:
                print(f"  CMA-ES seeded {name} from {args.seed} (fitness={seed_data.get('fitness', '?')})")
        else:
            opt = GeneticAlgorithm(cls, pop_size=args.pop_size,
                                   fitness_mode=args.fitness,
                                   mutation_sigma=args.sigma,
                                   sigma_min=args.sigma_min)
            if seed_genes is not None:
                seed_sigma = args.sigma
                n_seed = min(args.pop_size, max(20, args.pop_size // 2))
                opt.population[0] = seed_genes.copy()
                for i in range(1, n_seed):
                    opt.population[i] = seed_genes + np.random.randn(len(seed_genes)) * seed_sigma
                print(f"  GA seeded {name} from {args.seed} (fitness={seed_data.get('fitness', '?')})")
                print(f"  {n_seed}/{args.pop_size} slots seeded, σ={seed_sigma}")
        gas[name] = opt

    extra = ""
    if args.noise > 0:
        extra += f"  noise={args.noise}m"
    if args.dropout > 0:
        extra += f"  dropout={args.dropout}"
    print(f"Training {list(gas.keys())} on '{args.track}' track | "
          f"optimizer={args.optimizer}  generations={args.generations}  "
          f"pop={args.pop_size}  workers={args.workers}  "
          f"fitness={args.fitness}{extra}\n")

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
            "optimizer": args.optimizer,
            "noise_std": args.noise,
            "dropout": args.dropout,
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
