
# csce420-fall24: PA1 — Blocksworld (A* GraphSearch)

## Summary
This program solves Blocksworld problems using A* search with selectable heuristics.  
Moves: pick the **top** block from any stack and place it on top of another stack (unit cost).

## How to Run (Python implementation)
From this `PA1/` folder:
```bash
# Option A: quick reminder via make
make
# Option B: run directly
python3 blocksworld.py tests/probA03.bwp -H H2 --trace -MAX_ITERS 1000000
```

## Command Line Arguments
python3 blocksworld.py <problem.bwp> [-H H0|H1|H2] [-MAX_ITERS N] [--trace | --no-trace]

-H:
1. H0: 0 everywhere, simulates BFS.
2. H1: counts blocks not in a correct bottom prefix on their correct stack.
3. H2 (default): +1 for each block on the wrong stack, else (right stack) +1 if any block below it deviates from the goal’s bottom prefix (stronger than H1).

-MAX_ITERS N: hard cap on A* iterations before giving up (default is 1,000,000). If reached, the program reports planlen FAILED.

--trace / --no-trace: print or suppress the full sequence of states from start to goal.

## Input Format (.bwp)
S B M
>>>>>>>>>>
<initial stack 1>
...
<initial stack S>
>>>>>>>>>>
<goal stack 1>
...
<goal stack S>
>>>>>>>>>>

- Each stack line lists blocks bottom to top with no spaces.
- A blank line denotes an empty stack.

## Output Format
At termination the program prints one summary line:

statistics: <filename> method Astar planlen <N or FAILED> iters <count> maxq <max-frontier>

- planlen: length of the solution (number of moves) or FAILED.
- iters: count of main loop iterations.
- maxq: maximum frontier (priority queue) size observed.