import sys
import argparse
import heapq
from collections import deque

class State:
    # A state is a list of stacks, each stack is a string 'bottom...top'.
    # We store as tuple of strings for immutability and hashing.

    __slots__ = ("stacks",)

    def __init__(self, stacks):
        # normalize to tuple[str]
        self.stacks = tuple(stacks)

    def encode(self) -> str:
        # unique key for visited map
        # stack ["D", "", "CA", "BE"] becomes "S|D||CA|BE|"
        return "S|" + "|".join(self.stacks) + "|"

    def __eq__(self, other):
        return isinstance(other, State) and self.stacks == other.stacks

    def __hash__(self):
        return hash(self.stacks)
    
class Node:
    # Node wraps a State with g, h, f and parent pointer.
    __slots__ = ("state", "g", "h", "f", "parent")
    def __init__(self, state, g=0, h=0, parent=None):
        self.state = state
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
    def __lt__(self, other):
        # for heapq tie breaking, lowest f, then lowest h, then highest g
        if self.f != other.f: 
            return self.f < other.f
        if self.h != other.h: 
            return self.h < other.h
        return self.g > other.g

class Problem:
    __slots__ = ("initial", "goal", "S", "B", "M")
    def __init__(self, initial, goal, S, B, M):
        self.initial = initial
        self.goal = goal
        self.S = S
        self.B = B
        self.M = M

def _trim(s): return s.strip()

def _is_sep(line: str) -> bool:
    return line.count(">") >= 8

def _read_stacks(lines, S):
    stacks = []
    for _ in range(S):
        try:
            raw = _trim(next(lines))
        except StopIteration:
            raw = ""
        # keep only letters A-Z, empty line is empty stack
        filtered = "".join(ch for ch in raw if ch.isalpha())
        stacks.append(filtered)
    return stacks

def parse_bwp(path: str) -> Problem:
    with open(path, "r", encoding="utf-8") as f:
        lines_iter = iter(f.readlines())

    # header - S B M
    line = _trim(next(lines_iter))
    while not line:
        line = _trim(next(lines_iter))
    parts = line.split()
    if len(parts) < 2:
        raise ValueError("Bad header (need S B M)")
    S = int(parts[0]); B = int(parts[1]); M = int(parts[2]) if len(parts) >= 3 else -1

    # separator
    line = _trim(next(lines_iter))
    while line and not _is_sep(line):
        line = _trim(next(lines_iter))

    # initial stacks (S lines)
    init_stacks = _read_stacks(lines_iter, S)

    # separator
    line = _trim(next(lines_iter))
    while line and not _is_sep(line):
        line = _trim(next(lines_iter))

    # goal stacks (S lines)
    goal_stacks = _read_stacks(lines_iter, S)

    initial = State(init_stacks)
    goal = State(goal_stacks)
    return Problem(initial, goal, S, B, M)

# heuristics
def build_goal_index(goal_state: State):
    # goal_pos: dict[block_char] -> (stack_index, depth_from_bottom)
    goal_pos = {}
    for s_idx, col in enumerate(goal_state.stacks):
        for d_idx, ch in enumerate(col):
            goal_pos[ch] = (s_idx, d_idx)
    return goal_pos

def H0(_cur: State, _goal_index, _goal: State) -> int:
    return 0

def H1(cur: State, goal_index, _goal: State) -> int:
    # Count blocks NOT in a correct bottom prefix on their correct stack.
    # h = total_blocks - sum(correct_bottom_prefix_lengths).
    total = sum(len(col) for col in cur.stacks)
    correct = 0
    for s_idx, col in enumerate(cur.stacks):
        for d_idx, ch in enumerate(col):
            pos = goal_index.get(ch)
            if pos is None:       # unknown, ignore
                break
            gs, gd = pos
            if gs == s_idx and gd == d_idx:
                correct += 1
            else:
                break
    return total - correct

def H2(cur: State, goal_index, goal: State) -> int:
    # +1 per block if on wrong stack, OR on right stack but anything below it differs from the goal's below sequence.
    # Admissible under unit moves, more informative than H1.
    h = 0
    for s_idx, col in enumerate(cur.stacks):
        for d_idx, ch in enumerate(col):
            pos = goal_index.get(ch)
            if pos is None:
                continue
            gs, gd = pos
            if s_idx != gs:
                h += 1
            else:
                # right stack - check bottom..d_idx matches goal's bottom..d_idx
                goal_col = goal.stacks[gs]
                ok = True
                if d_idx >= len(goal_col):
                    ok = False
                else:
                    for k in range(d_idx + 1):
                        if k >= len(goal_col) or cur.stacks[s_idx][k] != goal_col[k]:
                            ok = False; break
                if not ok:
                    h += 1
    return h

def compute_h(cur: State, goal: State, gi, name: str) -> int:
    if name == "H0": return H0(cur, gi, goal)
    if name == "H1": return H1(cur, gi, goal)
    return H2(cur, gi, goal)  # default H2

# successors
def successors(state: State):
    S = len(state.stacks)
    for i in range(S):
        col_i = state.stacks[i]
        if not col_i:
            continue
        top = col_i[-1]
        for j in range(S):
            if i == j:
                continue
            # build next stacks (copy-on-write)
            new_cols = list(state.stacks)
            new_cols[i] = col_i[:-1]
            new_cols[j] = state.stacks[j] + top
            yield State(new_cols)

def print_state(state: State):
    for col in state.stacks:
        print(col)
    print(">>>>>>>>>>")

def print_solution_path(goal_node: Node, trace_states: bool):
    path = deque()
    p = goal_node
    while p is not None:
        path.appendleft(p.state)
        p = p.parent
    if trace_states:
        for k, st in enumerate(path):
            print(f"move {k}")
            print_state(st)

# A* Search
def astar(problem: Problem, heuristic_name: str, max_iters: int):
    gi = build_goal_index(problem.goal)

    # min heap frontier
    open_heap = []
    start = Node(problem.initial, g=0,
                 h=compute_h(problem.initial, problem.goal, gi, heuristic_name),
                 parent=None)
    heapq.heappush(open_heap, start)

    # best g seen for a state
    best_g = {problem.initial.encode(): 0}

    iters = 0
    maxq = len(open_heap)

    while open_heap:
        if iters >= max_iters:
            return False, -1, iters, maxq, None

        cur = heapq.heappop(open_heap)
        iters += 1

        if cur.state == problem.goal:
            return True, cur.g, iters, maxq, cur

        for nxt in successors(cur.state):
            g2 = cur.g + 1
            key = nxt.encode()
            if key in best_g and g2 >= best_g[key]:
                continue
            best_g[key] = g2
            h2 = compute_h(nxt, problem.goal, gi, heuristic_name)
            heapq.heappush(open_heap, Node(nxt, g=g2, h=h2, parent=cur))

        if len(open_heap) > maxq:
            maxq = len(open_heap)

    # no solution
    return False, -1, iters, maxq, None

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("filename", help="Path to .bwp problem file")
    p.add_argument("-H", dest="heuristic", default="H2", choices=["H0", "H1", "H2"],
                   help="Heuristic to use (default: H2)")
    p.add_argument("-MAX_ITERS", dest="max_iters", type=int, default=1_000_000,
                   help="Maximum A* loop iterations before giving up")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--trace", dest="trace", action="store_true", default=True,
                   help="Print full solution path states (default)")
    g.add_argument("--no-trace", dest="trace", action="store_false",
                   help="Do not print path states")
    return p.parse_args()

def main():
    args = parse_args()
    try:
        prob = parse_bwp(args.filename)
    except Exception as e:
        print(f"ERROR parsing {args.filename}: {e}", file=sys.stderr)
        sys.exit(2)

    ok, planlen, iters, maxq, goal_node = astar(prob, args.heuristic, args.max_iters)

    if ok:
        if args.trace:
            print_solution_path(goal_node, trace_states=True)
        print(f"statistics: {args.filename} method Astar planlen {planlen} iters {iters} maxq {maxq}")
    else:
        print(f"statistics: {args.filename} method Astar planlen FAILED iters {iters} maxq {maxq}")

if __name__ == "__main__":
    main()