import sys

# helper functions for parsing the CNF file and command line arguments
def parse_cnf(filename):
    clauses = []
    with open(filename, "r") as f:
        for line in f:
            # remove whitespace
            line = line.strip()
            # skip blank lines
            if line == "":
                continue
            # skip comment lines
            if line.startswith("#"):
                continue
            # split line into literals
            literals = line.split()
            # add clause to the list
            clauses.append(literals)
    return clauses

def add_arg_unit_clauses(clauses, extra_lits):
    # for each extra literal given on the command line, append it to the clause list as a unit clause
    for lit in extra_lits:
        if lit.strip() == "":
            continue
        # otherwise each literal becomes a unit clause
        clauses.append([lit])

def get_symbols(clauses):
    symbols = set()
    for clause in clauses:
        for lit in clause:
            if lit.startswith("-"):
                sym = lit[1:]
            else:
                sym = lit
            symbols.add(sym)
    # sort for deterministic behavior
    return sorted(symbols)

# helper functions to evaluate model and clause
def lit_value(literal, model):
    # return 1 if literal is true, -1 if false, and 0 if unknown
    if literal.startswith("-"):
        sym = literal[1:]
        sign = -1
    else:
        sym = literal
        sign = 1
    
    val = model.get(sym, 0)

    if val == 0:   # unknown literal
        return 0
    
    if val == sign:
        return 1
    else:
        return -1
    
def simplify_clauses(clauses, model):
    # new list of simplified clauses
    new_clauses = []

    for clause in clauses:
        clause_satisfied = False
        new_clause = []

        for lit in clause:
            lit_val = lit_value(lit, model)

            if lit_val == 1:
                # clause is satisfied, can ignore
                clause_satisfied = True
                break
            elif lit_val == 0:
                # literal unknown, keep in clause
                new_clause.append(lit)
            else:
                # literal is false (-1) so we drop it
                continue

        if clause_satisfied:
            continue

        # if not satsified
        new_clauses.append(new_clause)
    return new_clauses

def find_uc(clauses):
    # look for a unit clause (has only 1 literal)
    for clause in clauses:
        if len(clause) == 1:
            return clause[0]  # return the only literal
    return None  # no unit clause found
        
def unassigned_symbol(symbols, model):
    for sym in symbols:
        if model.get(sym, 0) == 0:
            return sym
    return None

def assign_lit(literal, model):
    new_model = dict(model)

    if literal.startswith("-"):
        sym = literal[1:]
        new_model[sym] = -1
    else:
        sym = literal
        new_model[sym] = 1

    return new_model

def assign_symbol(model, symbol, value):
    # return a new model with symbol set to a given value
    new_model = dict(model)
    new_model[symbol] = value
    return new_model


# DPLL algorithm

# num of calls to dpll 
dpll_calls = 0

def DPLL(clauses, symbols, model):
    global dpll_calls
    dpll_calls += 1

    # simplify clause set under current model
    simplified = simplify_clauses(clauses, model)

    for clause in simplified:
        if len(clause) == 0:
            return None  # fail
        
    if len(simplified) == 0:
        return model  # success
    
    # unit clause heuristic
    unit_lit = find_uc(simplified)
    if unit_lit is not None:
        new_model = assign_lit(unit_lit, model)
        return DPLL(clauses, symbols, new_model)
    
    u = unassigned_symbol(symbols, model)
    if u is None:
        return None
    
    # try u = true
    model_true = assign_symbol(model, u, 1)
    result = DPLL(clauses, symbols, model_true)
    if result is not None:
        return result  # found satisfying model
    
    # otherwise backtrack, try u = false
    model_false = assign_symbol(model, u, -1)
    return DPLL(clauses, symbols, model_false)


def print_model(model):
    print("solution:\n")
    for sym in sorted(model.keys()):
        val = model[sym]
        if val != 0:
            print(f"{sym}: {val}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 DPLL.py <KB_filename> <literal>*")
        sys.exit(1)

    kb_filename = sys.argv[1]
    extra_lits = sys.argv[2:] # extra literals become unit clauses

    # parse cnf file
    clauses = parse_cnf(kb_filename)

    # add unit clauses from command line literals
    add_arg_unit_clauses(clauses, extra_lits)

    # extract propositional symbols
    symbols = get_symbols(clauses)

    # initialize model with all symbols as 0
    model = {sym: 0 for sym in symbols}

    # run dpll
    global dpll_calls
    dpll_calls = 0
    result_model = DPLL(clauses, symbols, model)

    # print results
    if result_model is None:
        print("unsatisfiable")
    else:
        print_model(result_model)

    print(f"\ntotal DPLL calls: {dpll_calls}")

if __name__ == "__main__":
    main()


