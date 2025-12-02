# Makefile for PA2 DPLL Solver (Python)

PYTHON = python3
SCRIPT = DPLL.py

# this runs the solver by default on all CNF files automatically
# 'make' produces all the transcripts.

all: test_mapcolor test_6queens test_wumpus
	@echo "All tests completed."

# run mapcolor.cnf
test_mapcolor:
	$(PYTHON) $(SCRIPT) mapcolor.cnf > transcript.mapcolor.txt
	$(PYTHON) $(SCRIPT) mapcolor.cnf SAB > transcript.mapcolor2.txt
	@echo "MapColor tests done."

# run 6queens.cnf
test_6queens:
	$(PYTHON) $(SCRIPT) 6queens.cnf > transcript.6queens.A.txt
	$(PYTHON) $(SCRIPT) 6queens.cnf -Q36 > transcript.6queens.B.txt
	@echo "6Queens tests done."

# run wumpus.cnf
test_wumpus:
	$(PYTHON) $(SCRIPT) wumpus.cnf -B12 -S12 B21 -S21 > transcript.wumpus.txt
	@echo "Wumpus test done."

# to clean transcripts
clean:
	rm -f transcript*.txt
	@echo "Cleaned transcripts."
