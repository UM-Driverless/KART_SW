# kart autoresearch

Train an autonomous kart controller to complete the autocross track as fast as possible without going off track.

Inspired by [karpathy/autoresearch](https://github.com/karpathy/autoresearch).

## Setup

To set up a new experiment, work with the user to:

1. **Agree on a run tag**: propose a tag based on today's date (e.g. `mar15`). The branch `autoresearch/<tag>` must not already exist.
2. **Create the branch**: `git checkout -b autoresearch/<tag>` from current main.
3. **Read the in-scope files**:
   - `evaluate.py` — fixed evaluation script. Do not modify.
   - `strategy.py` — the file you modify. Training strategy definition.
   - `controllers.py` — reference: existing controller architectures (geometric, neural v1/v2/v3). Read-only but you can import and use anything.
   - `sim.py` — reference: simulation loop, fitness functions (v1-v6). Read-only.
   - `track.py` — reference: track definitions, boundary checking. Read-only.
   - `ga.py` — reference: GA and CMA-ES optimizers. Read-only.
   - `kart_model.py` — reference: bicycle kinematics model. Read-only.
   - `perception.py` — reference: cone visibility simulation. Read-only.
4. **Run baseline**: `python evaluate.py > run.log 2>&1` and record results.
5. **Initialize results.tsv** with header row and baseline.
6. **Confirm and go**.

## The problem

- A kart drives around the "autocross" track (~250m lap, mixed left/right turns).
- The kart sees cones (blue=left, yellow=right) in its field of view (±35°, 0.5-15m range).
- A controller receives cone positions and outputs steering angle + speed.
- The kart must stay on the track (between the cone boundaries) at all times.
- Goal: minimize lap time.

## The metric

The evaluation trains a controller within a **fixed 60-second time budget**, then evaluates it. The key output is `loss` (lower is better):

- If **1+ laps completed**: loss ≈ lap_time + small centering penalty
- If **0 laps**: loss = 100 - 50 × (distance / track_length) — distance proxy
- **Off-track penalty**: +50.0 if the kart ever leaves the track boundaries

Best possible loss: fast lap time (~20-30s range) with good centering.

## What you CAN do

Modify `strategy.py` — this is the only file you edit. Everything is fair game:

- **Controller architecture**: use existing controllers (geometric, neural v1/v2/v3) or define entirely new ones inline. New architectures must have `.control(visible_cones)` or `.control(visible_cones, current_speed=...)` method returning `(steer, speed)`.
- **Optimizer**: CMA-ES, GA, or implement your own (differential evolution, random search, etc.)
- **Hyperparameters**: population size, sigma, mutation rates, elite fraction
- **Fitness function**: which mode (v1-v6) to use during training
- **Training sim settings**: max_steps, noise, dropout for robustness
- **Seeding**: load previous best weights and fine-tune
- **Multi-track training**: train on multiple tracks for generalization
- **Anything else** that fits in `strategy.py`

## What you CANNOT do

- Modify `evaluate.py`. It is read-only.
- Modify any other file (`controllers.py`, `sim.py`, `track.py`, `ga.py`, `kart_model.py`, `perception.py`). They are read-only. You can import and call anything from them.
- Exceed the 60-second training time budget.
- Install new packages.

## Physical constraints (important for controller design)

- **Steering**: -0.5 to +0.5 rad
- **Speed**: 0 to 10 m/s (but realistic max ~5 m/s for control)
- **Wheelbase**: 1.05m
- **FOV**: ±35° from forward, 0.5m to 15m range
- **Track width**: 3m (1.5m half-width)
- **Acceleration**: 2 m/s² accel, 3 m/s² decel (enforced by ackermann_to_vel)
- **No-cone timeout**: controller gets zero cones if none visible — handle gracefully
- **Off-track = termination**: episode ends immediately if kart crosses boundary

## Running an experiment

```bash
cd tools/sim2d
python evaluate.py > run.log 2>&1
```

Extract results:
```bash
grep "^loss:\|^laps:\|^lap_time:\|^avg_speed:" run.log
```

Each run takes ~70 seconds (60s training + 10s evaluation).

## Logging results

Log every experiment to `results.tsv` (tab-separated):

```
commit	loss	laps	lap_time	status	description
```

- commit: short git hash (7 chars)
- loss: the combined loss value (lower is better)
- laps: number of complete laps
- lap_time: seconds per lap (999.0 if no laps)
- status: `keep`, `discard`, or `crash`
- description: what you tried

## The experiment loop

LOOP FOREVER:

1. Look at current state
2. Edit `strategy.py` with an experimental idea
3. git commit
4. Run: `python evaluate.py > run.log 2>&1`
5. Read results: `grep "^loss:\|^laps:\|^lap_time:" run.log`
6. If empty/crash: `tail -n 50 run.log`, attempt fix or skip
7. Record in results.tsv
8. If loss improved (lower): keep the commit
9. If loss is equal or worse: `git reset --hard HEAD~1`

**NEVER STOP.** The user might be away. Keep experimenting indefinitely until manually stopped.

## Ideas to explore

- Different controller architectures (deeper nets, recurrent/LSTM, attention over cones)
- Different optimizers (differential evolution, particle swarm, simulated annealing)
- Different fitness functions (v3 for track-keeping, v6 for speed × centering)
- Perception noise/dropout during training for robustness
- Curriculum learning: train on easy track first, then autocross
- Seeding from previous best and fine-tuning with smaller sigma
- Custom controller that combines geometric heuristics with neural adjustment
- Larger populations with more workers
- Multi-track training (oval + hairpin + autocross)
