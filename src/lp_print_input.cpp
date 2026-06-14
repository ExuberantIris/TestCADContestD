#include "lp_print_input.hpp"

#include "lp_score.hpp"
#include "sa_eval.hpp"

#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <string>
#include <vector>

namespace {

int write_file(const char *out_dir, const char *name, const char *text)
{
    char path[1024];
    if (pd_join_path(path, sizeof(path), out_dir, name) != 0)
        return -1;
    FILE *fp = std::fopen(path, "w");
    if (!fp)
        return -1;
    std::fputs(text, fp);
    std::fclose(fp);
    return 0;
}

const char *branch_kind_str(LpBranchKind k)
{
    return k == LpBranchKind::ExistingBuf ? "ExistingBuf" : "Insertable";
}

const char *node_name(const PdDesign *d, int id)
{
    if (id < 0 || id >= d->n_nodes)
        return "?";
    return d->nodes[id].name;
}

void eval_slacks_at_x(const LpProblem *pb, const PdDesign *d, const SaPgCtx &ctx,
                      const std::vector<double> &d_ss, const std::vector<double> &d_ff,
                      SaPgCtx *out)
{
    *out = ctx;
    const int n_ff = static_cast<int>(pb->ff_node_ids.size());
    const int n_paths = static_cast<int>(pb->path_ids.size());

    for (int f = 0; f < n_ff; f++) {
        out->T_ss[static_cast<std::size_t>(f)] = 0.0;
        out->T_ff[static_cast<std::size_t>(f)] = 0.0;
        for (int k = out->ff_path_off[static_cast<std::size_t>(f)];
             k < out->ff_path_off[static_cast<std::size_t>(f + 1)]; k++) {
            const int b = out->ff_path_br[static_cast<std::size_t>(k)];
            out->T_ss[static_cast<std::size_t>(f)] += d_ss[static_cast<std::size_t>(b)];
            out->T_ff[static_cast<std::size_t>(f)] += d_ff[static_cast<std::size_t>(b)];
        }
    }

    out->wns_ss = 1e30;
    out->wns_ff = 1e30;
    out->tns_ss = 0.0;
    out->tns_ff = 0.0;

    for (int p = 0; p < n_paths; p++) {
        const int li = out->launch_ff[static_cast<std::size_t>(p)];
        const int ci = out->capture_ff[static_cast<std::size_t>(p)];
        double t_ss_l = 0.0, t_ss_c = 0.0, t_ff_l = 0.0, t_ff_c = 0.0;
        if (li >= 0) {
            t_ss_l = out->T_ss[static_cast<std::size_t>(li)];
            t_ff_l = out->T_ff[static_cast<std::size_t>(li)];
        }
        if (ci >= 0) {
            t_ss_c = out->T_ss[static_cast<std::size_t>(ci)];
            t_ff_c = out->T_ff[static_cast<std::size_t>(ci)];
        }
        out->slack_ss[static_cast<std::size_t>(p)] =
            d->clock_period - d->t_setup -
            d->paths[pb->path_ids[static_cast<std::size_t>(p)]].data_ss + (t_ss_c - t_ss_l);
        out->slack_ff[static_cast<std::size_t>(p)] =
            d->paths[pb->path_ids[static_cast<std::size_t>(p)]].data_ff - d->t_hold -
            (t_ff_c - t_ff_l);

        if (out->slack_ss[static_cast<std::size_t>(p)] < out->wns_ss)
            out->wns_ss = out->slack_ss[static_cast<std::size_t>(p)];
        if (out->slack_ss[static_cast<std::size_t>(p)] < 0.0)
            out->tns_ss += out->slack_ss[static_cast<std::size_t>(p)];
        if (out->slack_ff[static_cast<std::size_t>(p)] < out->wns_ff)
            out->wns_ff = out->slack_ff[static_cast<std::size_t>(p)];
        if (out->slack_ff[static_cast<std::size_t>(p)] < 0.0)
            out->tns_ff += out->slack_ff[static_cast<std::size_t>(p)];
    }
    if (out->wns_ss > 1e29)
        out->wns_ss = 0.0;
    if (out->wns_ff > 1e29)
        out->wns_ff = 0.0;

    out->area = 0.0;
    for (const LpBranch &br : pb->branches) {
        if (br.kind == LpBranchKind::ExistingBuf)
            out->area += br.area_per_ss;
    }
    out->score = lp_compute_score(out->wns_ss, out->tns_ss, out->wns_ff, out->tns_ff, out->area,
                                  pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori,
                                  pb->area_ori);
}

void init_delays_from_design(const LpProblem *pb, const PdDesign *d, std::vector<double> *d_ss,
                             std::vector<double> *d_ff)
{
    const int n = static_cast<int>(pb->branches.size());
    d_ss->assign(static_cast<std::size_t>(n), 0.0);
    d_ff->assign(static_cast<std::size_t>(n), 0.0);
    for (int b = 0; b < n; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind == LpBranchKind::ExistingBuf && br.cell_idx >= 0) {
            const PdCell *c = &d->cells[br.cell_idx];
            (*d_ss)[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ss(d, c, br.fanout);
            (*d_ff)[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ff(d, c, br.fanout);
        }
    }
}

} // namespace

int lp_write_mo_input_dump(const LpProblem *pb, const PdDesign *d, const char *out_dir,
                           char *err, std::size_t err_sz)
{
    if (!pb || !d || !out_dir) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null argument");
        return -1;
    }

    mkdir(out_dir, 0755);

    {
        std::string s;
        s += "# mo_projected_gradient LP input (ProblemD_SA_prime)\n";
        s += "# Variables: per branch b — d_ss[b], d_ff[b]\n";
        s += "# Constraints: box bounds per branch (project each iter)\n";
        s += "# Objective: minimize violations of WNS/TNS for SS setup and FF hold\n\n";

        char buf[256];
        std::snprintf(buf, sizeof(buf), "n_branches: %zu\n", pb->branches.size());
        s += buf;
        std::snprintf(buf, sizeof(buf), "n_ff: %zu\n", pb->ff_node_ids.size());
        s += buf;
        std::snprintf(buf, sizeof(buf), "n_paths: %zu\n", pb->path_ids.size());
        s += buf;
        std::snprintf(buf, sizeof(buf), "clock_period: %.9f\n", d->clock_period);
        s += buf;
        std::snprintf(buf, sizeof(buf), "t_setup: %.9f\n", d->t_setup);
        s += buf;
        std::snprintf(buf, sizeof(buf), "t_hold: %.9f\n", d->t_hold);
        s += buf;
        std::snprintf(buf, sizeof(buf), "wns_ss_ori: %.9f\n", pb->wns_ss_ori);
        s += buf;
        std::snprintf(buf, sizeof(buf), "tns_ss_ori: %.9f\n", pb->tns_ss_ori);
        s += buf;
        std::snprintf(buf, sizeof(buf), "wns_ff_ori: %.9f\n", pb->wns_ff_ori);
        s += buf;
        std::snprintf(buf, sizeof(buf), "tns_ff_ori: %.9f\n", pb->tns_ff_ori);
        s += buf;
        std::snprintf(buf, sizeof(buf), "area_ori: %.9f\n", pb->area_ori);
        s += buf;
        s += "solver: mo_projected_gradient\n";
        s += "step: 0.004\n";
        s += "eps: 1e-6\n";
        s += "default_lp_init_time_limit_sec: 30\n";

        if (write_file(out_dir, "summary.txt", s.c_str()) != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "cannot write summary.txt");
            return -1;
        }
    }

    {
        FILE *fp = nullptr;
        char path[1024];
        if (pd_join_path(path, sizeof(path), out_dir, "branches.tsv") != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "path too long");
            return -1;
        }
        fp = std::fopen(path, "w");
        if (!fp) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "cannot open branches.tsv");
            return -1;
        }

        std::fprintf(fp,
                     "branch_idx\tkind\tparent_node\tparent_name\tchild_node\tchild_name\t"
                     "fanout\tcell_idx\tcell_name\t"
                     "d_ss_min\td_ss_max\td_ff_min\td_ff_max\tarea_per_ss\t"
                     "x0_d_ss\tx0_d_ff\n");

        std::vector<double> x0_ss, x0_ff;
        init_delays_from_design(pb, d, &x0_ss, &x0_ff);

        for (std::size_t b = 0; b < pb->branches.size(); b++) {
            const LpBranch &br = pb->branches[b];
            const char *cname =
                (br.cell_idx >= 0 && br.cell_idx < d->n_cells) ? d->cells[br.cell_idx].name : "";
            std::fprintf(fp, "%zu\t%s\t%d\t%s\t%d\t%s\t%d\t%d\t%s\t"
                             "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
                         b, branch_kind_str(br.kind), br.parent_node,
                         node_name(d, br.parent_node), br.child_node, node_name(d, br.child_node),
                         br.fanout, br.cell_idx, cname, br.d_ss_min, br.d_ss_max, br.d_ff_min,
                         br.d_ff_max, br.area_per_ss, x0_ss[b], x0_ff[b]);
        }
        std::fclose(fp);
    }

    {
        FILE *fp = nullptr;
        char path[1024];
        if (pd_join_path(path, sizeof(path), out_dir, "paths.tsv") != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "path too long");
            return -1;
        }
        fp = std::fopen(path, "w");
        if (!fp) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "cannot open paths.tsv");
            return -1;
        }

        SaPgCtx ctx;
        if (!sa_build_ctx(pb, d, &ctx)) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "sa_build_ctx failed");
            return -1;
        }

        std::vector<double> x0_ss, x0_ff;
        init_delays_from_design(pb, d, &x0_ss, &x0_ff);
        SaPgCtx eval;
        eval_slacks_at_x(pb, d, ctx, x0_ss, x0_ff, &eval);

        std::fprintf(fp,
                     "path_lp_idx\tpath_design_idx\tlaunch_name\tcapture_name\t"
                     "launch_ff_idx\tcapture_ff_idx\tdata_ss\tdata_ff\t"
                     "slack_ss_x0\tslack_ff_x0\n");

        for (int p = 0; p < static_cast<int>(pb->path_ids.size()); p++) {
            const int pid = pb->path_ids[static_cast<std::size_t>(p)];
            const PdPath *path = &d->paths[pid];
            const int li = ctx.launch_ff[static_cast<std::size_t>(p)];
            const int ci = ctx.capture_ff[static_cast<std::size_t>(p)];
            std::fprintf(fp, "%d\t%d\t%s\t%s\t%d\t%d\t%.9f\t%.9f\t%.9f\t%.9f\n", p, pid,
                         path->launch, path->capture, li, ci, path->data_ss, path->data_ff,
                         eval.slack_ss[static_cast<std::size_t>(p)],
                         eval.slack_ff[static_cast<std::size_t>(p)]);
        }
        std::fclose(fp);
    }

    {
        FILE *fp = nullptr;
        char path[1024];
        if (pd_join_path(path, sizeof(path), out_dir, "ff_branch_paths.tsv") != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "path too long");
            return -1;
        }
        fp = std::fopen(path, "w");
        if (!fp) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "cannot open ff_branch_paths.tsv");
            return -1;
        }

        SaPgCtx ctx;
        sa_build_ctx(pb, d, &ctx);

        std::fprintf(fp, "ff_lp_idx\tff_node_id\tff_name\tstep\tbranch_idx\tparent\tchild\n");
        for (int f = 0; f < static_cast<int>(pb->ff_node_ids.size()); f++) {
            const int nid = pb->ff_node_ids[static_cast<std::size_t>(f)];
            int step = 0;
            for (int k = ctx.ff_path_off[static_cast<std::size_t>(f)];
                 k < ctx.ff_path_off[static_cast<std::size_t>(f + 1)]; k++, step++) {
                const int b = ctx.ff_path_br[static_cast<std::size_t>(k)];
                const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
                std::fprintf(fp, "%d\t%d\t%s\t%d\t%d\t%s\t%s\n", f, nid, node_name(d, nid), step,
                             b, node_name(d, br.parent_node), node_name(d, br.child_node));
            }
        }
        std::fclose(fp);
    }

    {
        std::string s;
        s += "# Timing at x0 (design delays, before LP iterations)\n\n";
        std::vector<double> x0_ss, x0_ff;
        init_delays_from_design(pb, d, &x0_ss, &x0_ff);
        SaPgCtx base, eval;
        sa_build_ctx(pb, d, &base);
        eval_slacks_at_x(pb, d, base, x0_ss, x0_ff, &eval);

        char buf[128];
        std::snprintf(buf, sizeof(buf), "wns_ss_x0: %.9f\n", eval.wns_ss);
        s += buf;
        std::snprintf(buf, sizeof(buf), "tns_ss_x0: %.9f\n", eval.tns_ss);
        s += buf;
        std::snprintf(buf, sizeof(buf), "wns_ff_x0: %.9f\n", eval.wns_ff);
        s += buf;
        std::snprintf(buf, sizeof(buf), "tns_ff_x0: %.9f\n", eval.tns_ff);
        s += buf;
        std::snprintf(buf, sizeof(buf), "area_proxy_x0: %.9f\n", eval.area);
        s += buf;
        std::snprintf(buf, sizeof(buf), "score_x0: %.9f\n", eval.score);
        s += buf;

        int n_viol_ss = 0, n_viol_ff = 0;
        for (int p = 0; p < static_cast<int>(pb->path_ids.size()); p++) {
            if (eval.slack_ss[static_cast<std::size_t>(p)] < 0.0)
                n_viol_ss++;
            if (eval.slack_ff[static_cast<std::size_t>(p)] < 0.0)
                n_viol_ff++;
        }
        std::snprintf(buf, sizeof(buf), "violating_paths_ss: %d\n", n_viol_ss);
        s += buf;
        std::snprintf(buf, sizeof(buf), "violating_paths_ff: %d\n", n_viol_ff);
        s += buf;

        if (write_file(out_dir, "timing_x0.txt", s.c_str()) != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "cannot write timing_x0.txt");
            return -1;
        }
    }

    {
        FILE *fp = nullptr;
        char path[1024];
        if (pd_join_path(path, sizeof(path), out_dir, "violating_paths_x0.tsv") != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "path too long");
            return -1;
        }
        fp = std::fopen(path, "w");
        if (!fp) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "cannot open violating_paths_x0.tsv");
            return -1;
        }

        std::vector<double> x0_ss, x0_ff;
        init_delays_from_design(pb, d, &x0_ss, &x0_ff);
        SaPgCtx base, eval;
        sa_build_ctx(pb, d, &base);
        eval_slacks_at_x(pb, d, base, x0_ss, x0_ff, &eval);

        std::fprintf(fp, "path_lp_idx\tlaunch\tcapture\tslack_ss\tslack_ff\tviolation_weight\n");
        for (int p = 0; p < static_cast<int>(pb->path_ids.size()); p++) {
            const double ss = eval.slack_ss[static_cast<std::size_t>(p)];
            const double ff = eval.slack_ff[static_cast<std::size_t>(p)];
            if (ss >= 0.0 && ff >= 0.0)
                continue;
            const int pid = pb->path_ids[static_cast<std::size_t>(p)];
            const PdPath *path = &d->paths[pid];
            const double w =
                (ss < 0.0 ? -ss : 0.0) + (ff < 0.0 ? -ff : 0.0);
            std::fprintf(fp, "%d\t%s\t%s\t%.9f\t%.9f\t%.9f\n", p, path->launch, path->capture, ss,
                         ff, w);
        }
        std::fclose(fp);
    }

    {
        const char *readme =
            "LP input dump (testcase1 example)\n"
            "================================\n"
            "summary.txt           — counts, clock/timing constants, ori metrics, solver params\n"
            "branches.tsv          — decision variables: bounds + x0 (initial delays from tree)\n"
            "paths.tsv             — all timing paths with slack at x0\n"
            "ff_branch_paths.tsv   — FF -> root branch list (gradient topology)\n"
            "timing_x0.txt         — WNS/TNS/area/score at initial point\n"
            "violating_paths_x0.tsv — paths with negative slack at x0 (drive LP gradients)\n"
            "\n"
            "Slack formulas (path p, launch L, capture C):\n"
            "  slack_ss = clock_period - t_setup - data_ss + (T_ss[C] - T_ss[L])\n"
            "  slack_ff = data_ff - t_hold - (T_ff[C] - T_ff[L])\n"
            "  T_ss[f] = sum of d_ss on branches from FF f to root\n"
            "\n"
            "LP update: projected gradient on d_ss/d_ff with step=0.004;\n"
            "  capture branches += gradient for setup violations;\n"
            "  launch branches -= gradient for hold violations.\n";
        if (write_file(out_dir, "README.txt", readme) != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "cannot write README.txt");
            return -1;
        }
    }

    return 0;
}
