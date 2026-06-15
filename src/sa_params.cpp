#include "sa_params.hpp"

#include <cstdio>
#include <cstring>

namespace {

void trim(char *s)
{
    if (!s)
        return;
    char *start = s;
    while (*start == ' ' || *start == '\t')
        start++;
    if (start != s)
        std::memmove(s, start, std::strlen(start) + 1);
    const std::size_t n = std::strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r' || s[n - 1] == '\n'))
        s[n - 1] = '\0';
}

bool parse_key_value(const char *line, char *key, std::size_t key_sz, char *val, std::size_t val_sz)
{
    const char *eq = std::strchr(line, '=');
    if (!eq)
        return false;
    std::snprintf(key, key_sz, "%.*s", static_cast<int>(eq - line), line);
    std::snprintf(val, val_sz, "%s", eq + 1);
    trim(key);
    trim(val);
    return key[0] != '\0';
}

void set_double(const char *key, const char *val, const char *name, double *out)
{
    if (std::strcmp(key, name) == 0)
        *out = std::atof(val);
}

void set_int_pos(const char *key, const char *val, const char *name, int *out)
{
    if (std::strcmp(key, name) == 0) {
        const int v = static_cast<int>(std::atof(val));
        if (v > 0)
            *out = v;
    }
}

void set_int_nonneg(const char *key, const char *val, const char *name, int *out)
{
    if (std::strcmp(key, name) == 0) {
        const int v = static_cast<int>(std::atof(val));
        if (v >= 0)
            *out = v;
    }
}

} // namespace

void sa_params_load(SaParams *out, const char *path)
{
    if (!out)
        return;

    if (!path || !path[0])
        path = kSaParamsPath;

    FILE *fp = std::fopen(path, "r");
    if (!fp)
        return;

    char line[512];
    while (std::fgets(line, sizeof(line), fp)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        char key[128];
        char val[384];
        if (!parse_key_value(line, key, sizeof(key), val, sizeof(val)))
            continue;

        set_double(key, val, "total_time_limit_sec", &out->total_time_limit_sec);
        set_double(key, val, "sa_time_limit_sec", &out->sa_time_limit_sec);
        set_int_pos(key, val, "no_improve_limit", &out->no_improve_limit);
        set_int_nonneg(key, val, "sa_batch_size", &out->sa_batch_size);
        set_int_pos(key, val, "sa_leaf_ff_pick_count", &out->sa_leaf_ff_pick_count);
        set_int_pos(key, val, "sa_branch_no_improve_limit", &out->sa_branch_no_improve_limit);
        set_double(key, val, "sa_temperature_init", &out->sa_temperature_init);
        set_double(key, val, "sa_temperature_decay", &out->sa_temperature_decay);
        set_double(key, val, "sa_temperature_floor", &out->sa_temperature_floor);
        set_double(key, val, "phase2_path_move_prob", &out->phase2_path_move_prob);
        set_int_pos(key, val, "phase2_top_path_pool", &out->phase2_top_path_pool);
        set_int_pos(key, val, "phase2_area_branch_pool", &out->phase2_area_branch_pool);
        set_double(key, val, "dp_delay_margin", &out->dp_delay_margin);
        if (std::strcmp(key, "score_a") == 0)
            out->score_weights.a = std::atof(val);
        else if (std::strcmp(key, "score_b") == 0)
            out->score_weights.b = std::atof(val);
        else if (std::strcmp(key, "score_g") == 0)
            out->score_weights.g = std::atof(val);
    }

    std::fclose(fp);
}
