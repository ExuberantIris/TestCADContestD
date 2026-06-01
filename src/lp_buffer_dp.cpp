#include "lp_buffer_dp.hpp"

#include "lp_types.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
LpBufferChainDp::Key LpBufferChainDp::delay_key(double d)
{
    if (d < 0.0)
        d = 0.0;
    return static_cast<Key>(std::llround(d / kDelayStep));
}

double LpBufferChainDp::key_to_delay(Key k)
{
    return static_cast<double>(k) * kDelayStep;
}

double LpBufferChainDp::cell_delay(const PdCell *c, int fanout, LpBufferDpCorner corner) const
{
    if (corner == LpBufferDpCorner::SS)
        return lp_eval_branch_delay_ss(design_, c, fanout);
    return lp_eval_branch_delay_ff(design_, c, fanout);
}

double LpBufferChainDp::cell_area(const PdCell *c) const
{
    return c->width * c->height;
}

int LpBufferChainDp::build(const PdDesign *d, LpBufferDpCorner corner)
{
    design_ = d;
    max_fanout_ = 1;
    for (int i = 0; i < d->n_cells; i++)
        max_fanout_ = std::max(max_fanout_, d->cells[i].max_fanout);

    prefix_.clear();
    prefix_[0] = LpBufferChainEntry{0.0, 0.0, {}, true};

    const Key max_k = delay_key(kMaxDelay);
    bool changed = true;
    while (changed) {
        changed = false;
        const auto snapshot = prefix_;
        for (const auto &kv : snapshot) {
            const Key kd = kv.first;
            const LpBufferChainEntry &cur = kv.second;
            if (!cur.reachable)
                continue;

            for (int ci = 0; ci < d->n_cells; ci++) {
                const PdCell *c = &d->cells[ci];
                const double step = cell_delay(c, 1, corner);
                if (step <= 0.0)
                    continue;

                const Key nk = kd + delay_key(step);
                if (nk > max_k)
                    continue;

                const double narea = cur.area + cell_area(c);
                auto it = prefix_.find(nk);
                if (it == prefix_.end() || narea < it->second.area - 1e-12) {
                    LpBufferChainEntry ent;
                    ent.delay = key_to_delay(nk);
                    ent.area = narea;
                    ent.cell_indices = cur.cell_indices;
                    ent.cell_indices.push_back(ci);
                    ent.reachable = true;
                    prefix_[nk] = std::move(ent);
                    changed = true;
                }
            }
        }
    }

    table_.assign(static_cast<std::size_t>(max_fanout_ + 1), {});
    delay_grid_.assign(static_cast<std::size_t>(max_fanout_ + 1), {});

    unreachable_.reachable = false;
    unreachable_.area = std::numeric_limits<double>::infinity();

    for (int n = 1; n <= max_fanout_; n++) {
        auto &row = table_[static_cast<std::size_t>(n)];
        std::vector<double> delays;

        for (int ci = 0; ci < d->n_cells; ci++) {
            const PdCell *c = &d->cells[ci];
            if (n > c->max_fanout)
                continue;

            const double final_d = cell_delay(c, n, corner);
            const Key final_k = delay_key(final_d);
            const double final_a = cell_area(c);

            for (const auto &pkv : prefix_) {
                const Key pk = pkv.first;
                const LpBufferChainEntry &pre = pkv.second;
                if (!pre.reachable)
                    continue;

                const Key total_k = pk + final_k;
                if (total_k > max_k)
                    continue;

                const double total_a = pre.area + final_a;
                auto it = row.find(total_k);
                if (it == row.end() || total_a < it->second.area - 1e-12) {
                    LpBufferChainEntry ent;
                    ent.delay = key_to_delay(total_k);
                    ent.area = total_a;
                    ent.cell_indices = pre.cell_indices;
                    ent.cell_indices.push_back(ci);
                    ent.reachable = true;
                    row[total_k] = std::move(ent);
                }
            }
        }

        delays.reserve(row.size());
        for (const auto &kv : row)
            delays.push_back(kv.second.delay);
        std::sort(delays.begin(), delays.end());
        delays.erase(std::unique(delays.begin(), delays.end(),
                                 [](double a, double b) {
                                     return std::fabs(a - b) < kDelayStep * 0.5;
                                 }),
                     delays.end());
        delay_grid_[static_cast<std::size_t>(n)] = std::move(delays);
    }

    return 0;
}

const LpBufferChainEntry &LpBufferChainDp::lookup(int fanout_n, double delay) const
{
    if (fanout_n < 1 || fanout_n > max_fanout_)
        return unreachable_;

    const auto &row = table_[static_cast<std::size_t>(fanout_n)];
    const auto it = row.find(delay_key(delay));
    if (it == row.end())
        return unreachable_;
    return it->second;
}

const std::vector<double> &LpBufferChainDp::delays_for_fanout(int fanout_n) const
{
    static const std::vector<double> empty;
    if (fanout_n < 1 || fanout_n > max_fanout_)
        return empty;
    return delay_grid_[static_cast<std::size_t>(fanout_n)];
}

void LpBufferChainDp::print_table(std::ostream &out, LpBufferDpCorner corner) const
{
    const char *corner_name = corner == LpBufferDpCorner::SS ? "SS" : "FF";
    out << std::fixed << std::setprecision(4);
    out << "=== Buffer chain DP (" << corner_name
        << ", step=" << kDelayStep << ", max_d=" << kMaxDelay << ") ===\n";
    out << "Structure: (0+ fanout-1 buffers) -> (1 fanout-n buffer)\n\n";

    for (int n = 1; n <= max_fanout_; n++) {
        out << "--- fanout n = " << n << " ---\n";
        const auto &delays = delays_for_fanout(n);
        if (delays.empty()) {
            out << "  (no entries)\n\n";
            continue;
        }

        out << "  delay\tarea\tbuffers\n";
        for (double d : delays) {
            const LpBufferChainEntry &e = lookup(n, d);
            if (!e.reachable)
                continue;
            out << "  " << d << '\t' << e.area << '\t';
            for (std::size_t i = 0; i < e.cell_indices.size(); i++) {
                if (i > 0)
                    out << " -> ";
                out << design_->cells[e.cell_indices[i]].name;
            }
            out << '\n';
        }
        out << '\n';
    }
}
