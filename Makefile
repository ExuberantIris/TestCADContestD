CC      = gcc
CXX     = g++
CFLAGS  = -std=c11 -Wall -Wextra -O2 -Iinclude -I../ProblemD/include
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude -I../ProblemD/include
LDFLAGS = -lm

PD_DIR  = ../ProblemD
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
          src/lp_apply.cpp

PD_OBJS = $(PD_SRCS:.c=.lp.o)
LP_OBJS = $(LP_SRCS:.cpp=.o)

.PHONY: all clean

all: lp_solver

lp_solver: $(PD_OBJS) $(LP_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(PD_OBJS) $(LP_OBJS) $(LDFLAGS)

$(PD_DIR)/src/%.lp.o: $(PD_DIR)/src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(PD_OBJS) $(LP_OBJS) lp_solver
