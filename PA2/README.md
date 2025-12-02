# PA2 DPLL Solver

This program implements the DPLL algorithm described in class. It recursively simplifies clauses under a model, applies unit clause propagation, and branches on unassigned symbols. Clause simplification and literal evaluation are implemented manually through helper functions without external libraries. Extra command line literals are treated as unit clauses to support generating alternative models.

---

Files included:
1. **dpll.py**: My full DPLL SAT solver implementation
    - Parses CNF files  
    - Applies unit propagation
    - Simplifies clauses manually 
    - Recursively branches on unassigned symbols
    - Tracks number of DPLL calls 
2. **mapcolor.cnf**: Australia map-coloring problem
3. **6queens.cnf**: 6-Queens puzzle encoded in CNF
4. **wumpus.cnf**: Wumpus World inference rules (breeze/stench biconditionals, safety rules )
5. **Output transcripts**:
    - transcript.mapcolor.A.txt
    - transcript.mapcolor.B.txt
    - transcript.6queens.A.txt
    - transcript.6queens.B.txt
    - transcript.wumpus.txt
6. **RESULTS.txt** - summarizes commands, SAT results, important model values, and DPLL call counts

---

# How to run:
  1. `python3 DPLL.py <cnf_file> <optional unit literals>`
  2. `make`
    - This will automatically run the solver on all CNF files, apply the required forced literal tests, and generate all transcript files. You can also clean transcripts with: `make clean`
    
    - Transcript files produced:
        1. `transcript.mapcolor.txt`
        2. `transcript.mapcolor2.txt`
        3. `transcript.6queens.A.txt`
        4. `transcript.6queens.B.txt`
        5. `transcript.wumpus.txt`

---

# Example commands:
  `python3 DPLL.py mapcolor.cnf`
  `python3 DPLL.py 6queens.cnf -Q36`
  `python3 DPLL.py wumpus.cnf -B12 -S12 B21 -S21`

---

# Notes:
1. Extra command line literals are seen as unit clauses, and these override the KB
2. DPLL call counter is printed at the end.
3. Output prints the model in sorted symbol order and the total number of DPLL calls.
4. All CNF files follow plain ASCII format (one clause per line, literals separated by spaces, `-` for negation, and `#` for comments)
5. No external SAT or logic libraries are used. Only Python’s built-in modules (`sys`) are used for handling command line arguments.

