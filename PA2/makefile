# Makefile for PA2 - DPLL 

PYTHON = python3
SCRIPT = DPLL.py

# run all tests and show output in console
all: test_mapcolor test_6queens test_wumpus
	@echo "All tests completed"

# mapcolor tests
test_mapcolor:
	@echo " Running mapcolor test 1"
	$(PYTHON) $(SCRIPT) cnf_files/mapcolor.cnf
	@echo ""
	@echo " Running mapcolor test 2 (force SAB)"
	$(PYTHON) $(SCRIPT) cnf_files/mapcolor.cnf SAB
	@echo "Mapcolor tests done"
	@echo ""

# 6Queens tests
test_6queens:
	@echo " Running 6-Queens test 1"
	$(PYTHON) $(SCRIPT) cnf_files/6queens.cnf
	@echo ""
	@echo " Running 6-Queens Test 2 (force -Q36)"
	$(PYTHON) $(SCRIPT) cnf_files/6queens.cnf -Q36
	@echo "6-Queens tests done"
	@echo ""

# Wumpus tests
test_wumpus:
	@echo " Running wumpus test"
	$(PYTHON) $(SCRIPT) cnf_files/wumpus.cnf -B12 -S12 B21 -S21
	@echo "Wumpus test done"
	@echo ""

# clean compiled Python cache
clean:
	rm -rf __pycache__
	@echo "Cleaned Python cache"
