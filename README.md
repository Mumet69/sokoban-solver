[![sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Mumet69/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Mumet69/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Sokoban Solver

A push-optimal Sokoban solver for the [make.fun bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121): one program answers 5 levels in as few pushes as possible.

## How it works

The generator's levels are a hallway with 1–4 rooms off it, each room 3–4 boxes, and a box never leaves its room — so the level optimum is the sum of the rooms' optima. The solver detects the rooms (cutting the hallway's dead squares out of the floor), then solves each room independently with a push-optimal A*:

- **State**: the boxes plus the player's reachable region (named by its lowest square).
- **Pruning**: dead squares (a box that can never reach a goal, found by pulling back from the goals) and frozen 2×2 blocks.
- **Heuristic**: min-cost matching of boxes to goals by shortest-path distance — admissible, so the first goal popped is push-optimal. An impossible matching prunes a dead state outright.
- **Deterministic**: no clock, no randomness; the budget is counted in states, so two runs write the same bytes.

## Results

- Practice set: **1,000,000 / 1,000,000** — every level solved in exactly P* pushes (13, 16, 29, 47, 60), in ~0.4 s for the whole set.
- House baseline (plain-astar 1.0.0) on the same practice set: 200,000 (solves only the first level).
- Also push-optimal (1,000,000) on three freshly drawn sets from the kit's own generator, so it generalises to the scored set.

## Run

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` is the whole solver: no dependencies, no network, writes only to `MAKEFUN_OUTPUT_DIR`.
