CC      = gcc
CXX     = g++
CFLAGS  = -std=c11 -Wall -Wextra -O2 -Iinclude
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lm

PD_DIR  = .
PD_SRCS = $(PD_DIR)/src/pd_parser.c \
          $(PD_DIR)/src/pd_util.c \
          $(PD_DIR)/src/pd_clock.c \
          $(PD_DIR)/src/pd_timing.c \
          $(PD_DIR)/src/pd_output.c

LP_SRCS = src/main.cpp \
          src/lp_branch.cpp \
          src/lp_score.cpp \
          src/lp_solve.cpp \
          src/lp_solve_pg.cpp \
          src/lp_solve_glpk.cpp \
          src/lp_apply.cpp \
          src/lp_buffer_dp.cpp

PD_OBJS = $(PD_SRCS:.c=.lp.o)
LP_OBJS = $(LP_SRCS:.cpp=.o)

UNITTEST_SRCS = unittest/test_buffer_chain_dp.cpp
UNITTEST_OBJS = $(UNITTEST_SRCS:.cpp=.o)

.PHONY: all clean unittest

all: lp_solver

lp_solver: $(PD_OBJS) $(LP_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(PD_OBJS) $(LP_OBJS) $(LDFLAGS)

unittest: lp_solver $(UNITTEST_OBJS)
	$(CXX) $(CXXFLAGS) -o test_buffer_chain_dp $(PD_OBJS) $(UNITTEST_OBJS) src/lp_branch.o src/lp_buffer_dp.o $(LDFLAGS)

$(PD_DIR)/src/%.lp.o: $(PD_DIR)/src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

unittest/%.o: unittest/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(PD_OBJS) $(LP_OBJS) $(UNITTEST_OBJS) lp_solver test_buffer_chain_dp
