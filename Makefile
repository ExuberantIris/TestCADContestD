CC      = gcc
CXX     = g++
CFLAGS  = -std=c11 -Wall -Wextra -O2 -Iinclude
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lm
TESTCASE_DIR ?= testcase
RESULT_DIR ?= result
SA_PHASE_TIME_LIMIT ?= 0.11
GREEDY_TIME_LIMIT ?= 540.0
GREEDY_ITERATIONS ?= 10
TESTCASE ?= all
TC ?= $(TESTCASE)
TESTCASES := $(notdir $(wildcard $(TESTCASE_DIR)/testcase*))

PD_DIR  = .
PD_SRCS = $(PD_DIR)/src/pd_parser.c \
          $(PD_DIR)/src/pd_util.c \
          $(PD_DIR)/src/pd_clock.c \
          $(PD_DIR)/src/pd_timing.c \
          $(PD_DIR)/src/pd_output.c

SA_SRCS = src/main.cpp \
          src/lp_branch.cpp \
          src/lp_score.cpp \
          src/lp_buffer_dp.cpp \
          src/lp_mo_init.cpp \
          src/sa_eval.cpp \
          src/sa_path_solve.cpp \
		  src/sa_apply.cpp \
		  src/greedy_postlp.cpp

PD_OBJS = $(PD_SRCS:.c=.lp.o)
SA_OBJS = $(SA_SRCS:.cpp=.o)

UNITTEST_SRCS = unittest/test_buffer_chain_dp.cpp
UNITTEST_OBJS = $(UNITTEST_SRCS:.cpp=.o)

PRINT_LP_SRCS = tools/print_lp_input.cpp src/lp_print_input.cpp
PRINT_LP_OBJS = tools/print_lp_input.o src/lp_print_input.o

.PHONY: all build run run-all clean unittest print_lp_input

all: run

build: sa_solver

run: sa_solver
	@if [ "$(TC)" = "all" ]; then \
		$(MAKE) run-all; \
	else \
		tc="$(TC)"; \
		tc="$${tc#$(TESTCASE_DIR)/}"; \
		if [ ! -d "$(TESTCASE_DIR)/$$tc" ]; then \
			echo "ERROR: $(TESTCASE_DIR)/$$tc not found"; \
			echo "Available testcases: $(TESTCASES)"; \
			exit 1; \
		fi; \
		mkdir -p "$(RESULT_DIR)/$$tc"; \
		echo "==> Running $$tc -> $(RESULT_DIR)/$$tc (SA_PHASE_TIME_LIMIT=$(SA_PHASE_TIME_LIMIT) GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT) GREEDY_ITERATIONS=$(GREEDY_ITERATIONS))"; \
		SA_PHASE_TIME_LIMIT=$(SA_PHASE_TIME_LIMIT) GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT) GREEDY_ITERATIONS=$(GREEDY_ITERATIONS) ./sa_solver "$(TESTCASE_DIR)/$$tc" "$(RESULT_DIR)/$$tc"; \
	fi

run-all: sa_solver
	@set -e; \
	echo "Using SA_PHASE_TIME_LIMIT=$(SA_PHASE_TIME_LIMIT) GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT) GREEDY_ITERATIONS=$(GREEDY_ITERATIONS)"; \
	for tc in $(TESTCASES); do \
		mkdir -p "$(RESULT_DIR)/$$tc"; \
		echo "==> Running $$tc -> $(RESULT_DIR)/$$tc"; \
		SA_PHASE_TIME_LIMIT=$(SA_PHASE_TIME_LIMIT) GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT) GREEDY_ITERATIONS=$(GREEDY_ITERATIONS) ./sa_solver "$(TESTCASE_DIR)/$$tc" "$(RESULT_DIR)/$$tc"; \
	done

print_lp_input: $(PD_OBJS) $(PRINT_LP_OBJS) src/lp_branch.o src/lp_score.o src/lp_buffer_dp.o src/sa_eval.o
	$(CXX) $(CXXFLAGS) -o $@ $(PD_OBJS) $(PRINT_LP_OBJS) src/lp_branch.o src/lp_score.o src/lp_buffer_dp.o src/sa_eval.o $(LDFLAGS)

sa_solver: $(PD_OBJS) $(SA_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(PD_OBJS) $(SA_OBJS) $(LDFLAGS)

unittest: sa_solver $(UNITTEST_OBJS)
	$(CXX) $(CXXFLAGS) -o test_buffer_chain_dp $(PD_OBJS) $(UNITTEST_OBJS) src/lp_branch.o src/lp_buffer_dp.o $(LDFLAGS)

$(PD_DIR)/src/%.lp.o: $(PD_DIR)/src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

tools/%.o: tools/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

unittest/%.o: unittest/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(PD_OBJS) $(SA_OBJS) $(UNITTEST_OBJS) $(PRINT_LP_OBJS) sa_solver test_buffer_chain_dp print_lp_input
