#pragma once

#include <cstddef>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "pd_c_api.hpp"

enum class LpBufferDpCorner { SS, FF };

struct LpBufferChainEntry {
    double delay = 0.0;
    double area = 0.0;
    /** Cell indices in drive order: optional fanout-1 prefix, then final fanout-n buffer. */
    std::vector<int> cell_indices;
    bool reachable = false;
};

/** Minimum-area buffer chain: (0+ fanout-1 cells) then one fanout-n cell, total delay = d. */
class LpBufferChainDp {
  public:
    static constexpr double kDelayStep = 1e-4;
    static constexpr double kMaxDelay = 0.5;

    int build(const PdDesign *d, LpBufferDpCorner corner);

    int max_fanout() const { return max_fanout_; }

    const LpBufferChainEntry &lookup(int fanout_n, double delay) const;

    const std::vector<double> &delays_for_fanout(int fanout_n) const;

    void print_table(std::ostream &out, LpBufferDpCorner corner) const;

  private:
    using Key = long long;

    static Key delay_key(double d);
    static double key_to_delay(Key k);

    double cell_delay(const PdCell *c, int fanout, LpBufferDpCorner corner) const;
    double cell_area(const PdCell *c) const;

    const PdDesign *design_ = nullptr;
    int max_fanout_ = 1;
    std::unordered_map<Key, LpBufferChainEntry> prefix_;
    std::vector<std::unordered_map<Key, LpBufferChainEntry>> table_;
    std::vector<std::vector<double>> delay_grid_;
    LpBufferChainEntry unreachable_{};
};
