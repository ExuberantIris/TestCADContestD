#include "pd_parser.h"

#include "pd_util.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fail(char *err, size_t err_sz, const char *msg)
{
    if (err && err_sz > 0)
        snprintf(err, err_sz, "%s", msg);
    return -1;
}

/* Small open-addressing string->int hash map, used only to speed up the name/path lookups
 * during parsing. Without it, resolving each path's launch/capture node is an O(n_nodes) scan
 * (O(n_paths*n_nodes) total) and matching FF_delay.rpt rows back to SS_delay.rpt paths is an
 * O(n_paths) scan per row (O(n_paths^2) total) - on the largest testcases (tens of thousands
 * of nodes, over 100k paths) that alone can burn minutes of the 10-minute run budget before
 * any optimization even starts. */
typedef struct {
    char key[2 * PD_MAX_NAME];
    int value;
    int used;
} PdHashSlot;

typedef struct {
    PdHashSlot *slots;
    unsigned int cap; /* power of 2 */
} PdHashMap;

static unsigned long pd_hash_str(const char *s)
{
    unsigned long h = 5381;
    while (*s)
        h = h * 33u + (unsigned char)(*s++);
    return h;
}

static int pd_hashmap_init(PdHashMap *m, int min_entries)
{
    unsigned int cap = 16;
    while (cap < (unsigned int)(min_entries < 1 ? 1 : min_entries) * 2u)
        cap *= 2;
    m->slots = calloc(cap, sizeof(PdHashSlot));
    if (!m->slots)
        return -1;
    m->cap = cap;
    return 0;
}

static void pd_hashmap_free(PdHashMap *m)
{
    free(m->slots);
    m->slots = NULL;
    m->cap = 0;
}

static void pd_hashmap_put(PdHashMap *m, const char *key, int value)
{
    unsigned long h = pd_hash_str(key) & (m->cap - 1);
    while (m->slots[h].used) {
        if (strcmp(m->slots[h].key, key) == 0) {
            m->slots[h].value = value;
            return;
        }
        h = (h + 1) & (m->cap - 1);
    }
    strncpy(m->slots[h].key, key, sizeof(m->slots[h].key) - 1);
    m->slots[h].value = value;
    m->slots[h].used = 1;
}

static int pd_hashmap_get(const PdHashMap *m, const char *key)
{
    unsigned long h = pd_hash_str(key) & (m->cap - 1);
    while (m->slots[h].used) {
        if (strcmp(m->slots[h].key, key) == 0)
            return m->slots[h].value;
        h = (h + 1) & (m->cap - 1);
    }
    return -1;
}

static int ensure_nodes(PdDesign *d, char *err, size_t err_sz)
{
    if (d->n_nodes < d->cap_nodes)
        return 0;
    int new_cap = d->cap_nodes ? d->cap_nodes * 2 : PD_INIT_NODES;
    PdNode *p = realloc(d->nodes, (size_t)new_cap * sizeof(PdNode));
    if (!p)
        return fail(err, err_sz, "out of memory (nodes)");
    d->nodes = p;
    d->cap_nodes = new_cap;
    return 0;
}

static int ensure_paths(PdDesign *d, char *err, size_t err_sz)
{
    if (d->n_paths < d->cap_paths)
        return 0;
    int new_cap = d->cap_paths ? d->cap_paths * 2 : PD_INIT_PATHS;
    PdPath *p = realloc(d->paths, (size_t)new_cap * sizeof(PdPath));
    if (!p)
        return fail(err, err_sz, "out of memory (paths)");
    d->paths = p;
    d->cap_paths = new_cap;
    return 0;
}

void pd_free_design(PdDesign *d)
{
    int i;

    if (!d)
        return;
    for (i = 0; i < d->n_nodes; i++)
        free(d->nodes[i].children);
    free(d->nodes);
    free(d->paths);
    memset(d, 0, sizeof(*d));
}

static int find_cell(const PdDesign *d, const char *name)
{
    int i;

    for (i = 0; i < d->n_cells; i++) {
        if (pd_streq(d->cells[i].name, name))
            return i;
    }
    return -1;
}

static int parse_buf_lib(const char *path, PdDesign *d, char *err, size_t err_sz)
{
    FILE *fp = fopen(path, "r");
    char line[PD_MAX_LINE];
    PdCell *cur = NULL;
    int in_cell = 0;

    if (!fp)
        return fail(err, err_sz, "cannot open buf.lib");

    while (fgets(line, sizeof(line), fp)) {
        char *p = pd_trim(line);
        char name[PD_MAX_NAME];
        double w, h;
        int n;

        if (*p == 0 || *p == '#')
            continue;

        if (sscanf(p, "cell ( %127[^)] )", name) == 1 ||
            sscanf(p, "cell (%127[^)])", name) == 1 ||
            sscanf(p, "cell( %127[^)] )", name) == 1 ||
            sscanf(p, "cell(%127[^)])", name) == 1) {
            if (d->n_cells >= PD_MAX_CELLS)
                goto bad;
            cur = &d->cells[d->n_cells++];
            memset(cur, 0, sizeof(*cur));
            strncpy(cur->name, name, PD_MAX_NAME - 1);
            in_cell = 1;
            continue;
        }

        if (!in_cell || !cur)
            continue;

        if (strncmp(p, "SIZE", 4) == 0) {
            if (sscanf(p, "SIZE %lf BY %lf", &w, &h) == 2) {
                cur->width = w;
                cur->height = h;
            }
            continue;
        }

        if (strncmp(p, "SS_DELAY", 8) == 0) {
            const char *vals = p + 8;
            int consumed;

            n = 0;
            while (n < PD_MAX_FANOUT_TBL) {
                while (*vals == ' ' || *vals == '\t')
                    vals++;
                if (sscanf(vals, "%lf%n", &cur->ss_delay[n], &consumed) != 1)
                    break;
                n++;
                vals += consumed;
            }
            if (n > cur->max_fanout)
                cur->max_fanout = n;
            continue;
        }

        if (strncmp(p, "FF_DELAY", 8) == 0) {
            const char *vals = p + 8;
            int fn = 0;
            int consumed;

            while (fn < PD_MAX_FANOUT_TBL) {
                while (*vals == ' ' || *vals == '\t')
                    vals++;
                if (sscanf(vals, "%lf%n", &cur->ff_delay[fn], &consumed) != 1)
                    break;
                fn++;
                vals += consumed;
            }
            if (fn > cur->max_fanout)
                cur->max_fanout = fn;
            continue;
        }

        if (*p == '}')
            in_cell = 0;
    }

    fclose(fp);
    return 0;
bad:
    fclose(fp);
    return fail(err, err_sz, "too many cells in buf.lib");
}

static int parse_structure(const char *path, PdDesign *d, char *err, size_t err_sz)
{
    FILE *fp = fopen(path, "r");
    char line[PD_MAX_LINE];
    int *stack = NULL;
    int stack_top = -1;
    int stack_cap = 0;
    int root_done = 0;

    if (!fp)
        return fail(err, err_sz, "cannot open clk_tree.structure");

    d->n_nodes = 0;

    while (fgets(line, sizeof(line), fp)) {
        char *p = pd_trim(line);
        int level;
        char name[PD_MAX_NAME];
        char cell[PD_MAX_NAME];
        char sink_tag[PD_MAX_NAME];
        PdNode *node;

        if (*p == 0 || *p == '#')
            continue;

        if (strncmp(p, "Root:", 5) == 0) {
            const char *rname = pd_trim(p + 5);
            strncpy(d->root_name, rname, PD_MAX_NAME - 1);
            if (ensure_nodes(d, err, err_sz) != 0) {
                fclose(fp);
                free(stack);
                return -1;
            }
            node = &d->nodes[d->n_nodes];
            memset(node, 0, sizeof(*node));
            strncpy(node->name, rname, PD_MAX_NAME - 1);
            node->kind = PD_NODE_ROOT;
            node->level = 0;
            node->parent = -1;
            node->id = d->n_nodes;
            d->n_nodes++;
            if (stack_top + 1 >= stack_cap) {
                int nc = stack_cap ? stack_cap * 2 : PD_INIT_NODES;
                int *ns = realloc(stack, (size_t)nc * sizeof(int));
                if (!ns) {
                    fclose(fp);
                    free(stack);
                    return fail(err, err_sz, "out of memory");
                }
                stack = ns;
                stack_cap = nc;
            }
            stack[++stack_top] = 0;
            root_done = 1;
            continue;
        }

        if (!root_done)
            goto bad;

        sink_tag[0] = '\0';
        if (sscanf(p, "[%d] %127s (%127[^)]) (%127[^)])", &level, name, cell, sink_tag) >= 3 ||
            sscanf(p, "[%d] %127s (%127[^)])", &level, name, cell) >= 3) {
            int parent_id;
            int is_sink = (sink_tag[0] && pd_streq(sink_tag, "SINK"));

            while (stack_top >= 0 && d->nodes[stack[stack_top]].level >= level)
                stack_top--;
            if (stack_top < 0)
                goto bad;

            parent_id = stack[stack_top];
            if (ensure_nodes(d, err, err_sz) != 0) {
                fclose(fp);
                free(stack);
                return -1;
            }

            node = &d->nodes[d->n_nodes];
            memset(node, 0, sizeof(*node));
            node->id = d->n_nodes;
            strncpy(node->name, name, PD_MAX_NAME - 1);
            strncpy(node->cell, cell, PD_MAX_NAME - 1);
            node->level = level;
            node->parent = parent_id;
            node->is_sink = is_sink;

            if (strncmp(name, "FF_", 3) == 0) {
                node->kind = PD_NODE_FF;
            } else {
                node->kind = PD_NODE_BUF;
                node->cell_idx = find_cell(d, cell);
                if (node->cell_idx < 0) {
                    fclose(fp);
                    snprintf(err, err_sz, "unknown cell type: %s", cell);
                    return -1;
                }
            }

            {
                PdNode *parent = &d->nodes[parent_id];
                int *new_ch;
                new_ch = realloc(parent->children, (size_t)(parent->nchildren + 1) * sizeof(int));
                if (!new_ch) {
                    fclose(fp);
                    return fail(err, err_sz, "out of memory");
                }
                parent->children = new_ch;
                parent->children[parent->nchildren++] = node->id;
            }

            d->n_nodes++;
            if (stack_top + 1 >= stack_cap) {
                int nc = stack_cap ? stack_cap * 2 : PD_INIT_NODES;
                int *ns = realloc(stack, (size_t)nc * sizeof(int));
                if (!ns) {
                    fclose(fp);
                    free(stack);
                    return fail(err, err_sz, "out of memory");
                }
                stack = ns;
                stack_cap = nc;
            }
            stack[++stack_top] = node->id;
        }
    }

    fclose(fp);
    free(stack);
    if (!root_done)
        return fail(err, err_sz, "missing Root line in clk_tree.structure");
    return 0;
bad:
    fclose(fp);
    free(stack);
    return fail(err, err_sz, "invalid clk_tree.structure");
}

static int parse_delay_rpt(const char *path, int is_ss, PdDesign *d, const PdHashMap *node_map,
                           const PdHashMap *path_map, char *err, size_t err_sz)
{
    FILE *fp = fopen(path, "r");
    char line[PD_MAX_LINE];
    int in_paths = 0;

    if (!fp)
        return fail(err, err_sz, is_ss ? "cannot open SS_delay.rpt" : "cannot open FF_delay.rpt");

    while (fgets(line, sizeof(line), fp)) {
        char *p = pd_trim(line);
        char launch[PD_MAX_NAME];
        char capture[PD_MAX_NAME];
        double delay;
        PdPath *path_row;

        if (strncmp(p, "Clock Period", 12) == 0) {
            if (sscanf(p, "Clock Period : %lf", &d->clock_period) != 1)
                sscanf(p, "Clock Period: %lf", &d->clock_period);
            continue;
        }

        if (strncmp(p, "#Path", 5) == 0) {
            in_paths = 1;
            continue;
        }

        if (!in_paths || *p == '-' || *p == 0)
            continue;

        {
            const char *colon = strchr(p, ':');
            if (!colon)
                continue;
            if (sscanf(colon + 1, " %127s -> %127s %lf", launch, capture, &delay) != 3)
                continue;
        }
        {
            if (is_ss) {
                if (ensure_paths(d, err, err_sz) != 0)
                    goto bad;
                path_row = &d->paths[d->n_paths];
                memset(path_row, 0, sizeof(*path_row));
                strncpy(path_row->launch, launch, PD_MAX_NAME - 1);
                strncpy(path_row->capture, capture, PD_MAX_NAME - 1);
                path_row->data_ss = delay;
                path_row->launch_id = pd_hashmap_get(node_map, launch);
                path_row->capture_id = pd_hashmap_get(node_map, capture);
                d->n_paths++;
            } else {
                char composite[2 * PD_MAX_NAME];
                int idx;

                snprintf(composite, sizeof(composite), "%s\x01%s", launch, capture);
                idx = pd_hashmap_get(path_map, composite);
                if (idx < 0) {
                    fclose(fp);
                    snprintf(err, err_sz, "FF path not in SS report: %s -> %s", launch, capture);
                    return -1;
                }
                d->paths[idx].data_ff = delay;
            }
        }
    }

    fclose(fp);
    return 0;
bad:
    fclose(fp);
    return fail(err, err_sz, "too many timing paths");
}

int pd_load_design(const char *testcase_dir, PdDesign *d, char *err, size_t err_sz)
{
    char path[1024];
    PdHashMap node_map = {0};
    PdHashMap path_map = {0};
    int i, rc;

    pd_free_design(d);

    if (pd_join_path(path, sizeof(path), testcase_dir, "buf.lib") != 0)
        return fail(err, err_sz, "path too long");
    if (parse_buf_lib(path, d, err, err_sz) != 0)
        return -1;

    if (pd_join_path(path, sizeof(path), testcase_dir, "clk_tree.structure") != 0)
        return fail(err, err_sz, "path too long");
    if (parse_structure(path, d, err, err_sz) != 0)
        return -1;

    if (pd_hashmap_init(&node_map, d->n_nodes) != 0)
        return fail(err, err_sz, "out of memory (node map)");
    for (i = 0; i < d->n_nodes; i++)
        pd_hashmap_put(&node_map, d->nodes[i].name, i);

    if (pd_join_path(path, sizeof(path), testcase_dir, "SS_delay.rpt") != 0) {
        pd_hashmap_free(&node_map);
        return fail(err, err_sz, "path too long");
    }
    rc = parse_delay_rpt(path, 1, d, &node_map, NULL, err, err_sz);
    pd_hashmap_free(&node_map);
    if (rc != 0)
        return -1;

    if (pd_hashmap_init(&path_map, d->n_paths) != 0)
        return fail(err, err_sz, "out of memory (path map)");
    for (i = 0; i < d->n_paths; i++) {
        char composite[2 * PD_MAX_NAME];
        snprintf(composite, sizeof(composite), "%s\x01%s", d->paths[i].launch, d->paths[i].capture);
        pd_hashmap_put(&path_map, composite, i);
    }

    if (pd_join_path(path, sizeof(path), testcase_dir, "FF_delay.rpt") != 0) {
        pd_hashmap_free(&path_map);
        return fail(err, err_sz, "path too long");
    }
    rc = parse_delay_rpt(path, 0, d, NULL, &path_map, err, err_sz);
    pd_hashmap_free(&path_map);
    if (rc != 0)
        return -1;

    d->t_setup = PD_SETUP_RATIO * d->clock_period;
    d->t_hold = PD_HOLD_RATIO * d->clock_period;
    return 0;
}
