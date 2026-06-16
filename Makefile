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

SA_SRCS = src/main.cpp \
          src/lp_branch.cpp \
          src/lp_score.cpp \
          src/lp_buffer_dp.cpp \
          src/sa_eval.cpp \
          src/sa_params.cpp \
          src/setup_lp_solve.cpp \
          src/sa_apply.cpp

SA_EMBED = src/sa_params_embed.cpp

PD_OBJS = $(PD_SRCS:.c=.lp.o)
SA_OBJS = $(SA_SRCS:.cpp=.o) $(SA_EMBED:.cpp=.o)

UNITTEST_SRCS = unittest/test_buffer_chain_dp.cpp
UNITTEST_OBJS = $(UNITTEST_SRCS:.cpp=.o)

.PHONY: all clean unittest verify_metrics

all: sa_solver

verify_metrics: $(PD_OBJS) verify/verify_metrics.o src/lp_score.o
	$(CXX) $(CXXFLAGS) -o verify/verify_metrics $(PD_OBJS) verify/verify_metrics.o src/lp_score.o $(LDFLAGS)

sa_solver: $(PD_OBJS) $(SA_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(PD_OBJS) $(SA_OBJS) $(LDFLAGS)

unittest: sa_solver $(UNITTEST_OBJS)
	$(CXX) $(CXXFLAGS) -o test_buffer_chain_dp $(PD_OBJS) $(UNITTEST_OBJS) src/lp_branch.o src/lp_buffer_dp.o $(LDFLAGS)

$(PD_DIR)/src/%.lp.o: $(PD_DIR)/src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

src/sa_params_embed.cpp: src/sa_params.txt
	xxd -i $< > $@

tools/%.o: tools/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

unittest/%.o: unittest/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

verify/%.o: verify/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(PD_OBJS) $(SA_OBJS) $(UNITTEST_OBJS) verify/verify_metrics.o
	rm -f sa_solver test_buffer_chain_dp verify/verify_metrics
	rm -f src/sa_params_embed.cpp
	rm -f src/*.o src/*.lp.o verify/*.o unittest/*.o
