CC      = gcc
CXX     = g++
CFLAGS  = -std=c11 -Wall -Wextra -O2 -Iinclude
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lm

TESTCASE_DIR ?= testcase
RESULT_DIR ?= result
GREEDY_TIME_LIMIT ?= 555.0
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
          src/sa_apply.cpp \
          src/greedy_postlp.cpp

PD_OBJS = $(PD_SRCS:.c=.lp.o)
SA_OBJS = $(SA_SRCS:.cpp=.o)

UNITTEST_SRCS = unittest/test_buffer_chain_dp.cpp
UNITTEST_OBJS = $(UNITTEST_SRCS:.cpp=.o)

PRINT_LP_SRCS = tools/print_lp_input.cpp src/lp_print_input.cpp
PRINT_LP_OBJS = tools/print_lp_input.o src/lp_print_input.o

# 將 report 加入 PHONY 清單
.PHONY: all build run run-all clean unittest print_lp_input report

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
		echo "==> Running $$tc -> $(RESULT_DIR)/$$tc (GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT))"; \
		GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT) ./sa_solver "$(TESTCASE_DIR)/$$tc" "$(RESULT_DIR)/$$tc"; \
	fi

run-all: sa_solver
	@set -e; \
	echo "Using GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT)"; \
	for tc in $(TESTCASES); do \
		mkdir -p "$(RESULT_DIR)/$$tc"; \
		echo "==> Running $$tc -> $(RESULT_DIR)/$$tc"; \
		GREEDY_TIME_LIMIT=$(GREEDY_TIME_LIMIT) ./sa_solver "$(TESTCASE_DIR)/$$tc" "$(RESULT_DIR)/$$tc"; \
	done
	@echo "==> 所有測資執行完畢，開始產生總結報告..."
	@$(MAKE) report

# 獨立抓取腳本，只抓 testcase0 ~ testcase4 的最終數據
report:
	@echo "Testcase,Final_Score,Final_Area,SS_WNS,SS_TNS,FF_WNS,FF_TNS" > result_summary.csv
	@for tc in testcase0 testcase1 testcase2 testcase3 testcase4; do \
		FILE="$(RESULT_DIR)/$$tc/result.txt"; \
		if [ -f "$$FILE" ]; then \
			score=$$(grep "Score" "$$FILE" | tail -n 1 | awk -F': ' '{print $$2}'); \
			area=$$(grep "Total area" "$$FILE" | tail -n 1 | awk -F': ' '{print $$2}'); \
			ss_wns=$$(grep "SS setup WNS" "$$FILE" | tail -n 1 | awk '{print $$5}'); \
			ss_tns=$$(grep "SS setup WNS" "$$FILE" | tail -n 1 | awk '{print $$8}'); \
			ff_wns=$$(grep "FF hold" "$$FILE" | tail -n 1 | awk '{print $$5}'); \
			ff_tns=$$(grep "FF hold" "$$FILE" | tail -n 1 | awk '{print $$8}'); \
			echo "$$tc,$$score,$$area,$$ss_wns,$$ss_tns,$$ff_wns,$$ff_tns" >> result_summary.csv; \
			echo "  [OK] 已將 $$tc 數據寫入 result_summary.csv"; \
		else \
			echo "  [SKIP] 找不到 $$FILE，略過該筆資料"; \
		fi \
	done
	@echo "==> 報告生成完畢！請查看 result_summary.csv"

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