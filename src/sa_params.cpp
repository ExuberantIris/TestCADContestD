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

} // namespace

void sa_params_load(SaParams *out, const char *path)
{
    if (!out)
        return;

    out->no_improve_limit = 10000;
    out->sa_batch_size = 20;
    out->score_weights.a = 0.6;
    out->score_weights.b = 0.2;
    out->score_weights.g = 0.2;

    if (!path || !path[0])
        return;

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

        if (std::strcmp(key, "no_improve_limit") == 0) {
            const int v = static_cast<int>(std::atof(val));
            if (v > 0)
                out->no_improve_limit = v;
        } else if (std::strcmp(key, "sa_batch_size") == 0) {
            const int v = static_cast<int>(std::atof(val));
            if (v >= 0)
                out->sa_batch_size = v;
        } else if (std::strcmp(key, "score_a") == 0) {
            out->score_weights.a = std::atof(val);
        } else if (std::strcmp(key, "score_b") == 0) {
            out->score_weights.b = std::atof(val);
        } else if (std::strcmp(key, "score_g") == 0) {
            out->score_weights.g = std::atof(val);
        }
    }

    std::fclose(fp);
}
