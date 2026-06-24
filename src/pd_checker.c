#include "pd_checker.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Small open-addressing string->int hash map, self-contained here (mirrors the one in
 * pd_parser.c used to speed up parsing - duplicated rather than shared so this checker stays
 * independent and low-risk to touch). */
typedef struct {
    char key[PD_MAX_NAME];
    int value;
    int used;
} CheckHashSlot;

typedef struct {
    CheckHashSlot *slots;
    unsigned int cap;
} CheckHashMap;

static unsigned long check_hash_str(const char *s)
{
    unsigned long h = 5381;
    while (*s)
        h = h * 33u + (unsigned char)(*s++);
    return h;
}

static int check_hashmap_init(CheckHashMap *m, int min_entries)
{
    unsigned int cap = 16;
    while (cap < (unsigned int)(min_entries < 1 ? 1 : min_entries) * 2u)
        cap *= 2;
    m->slots = (CheckHashSlot *)calloc(cap, sizeof(CheckHashSlot));
    if (!m->slots)
        return -1;
    m->cap = cap;
    return 0;
}

static void check_hashmap_free(CheckHashMap *m)
{
    free(m->slots);
    m->slots = NULL;
    m->cap = 0;
}

static void check_hashmap_put(CheckHashMap *m, const char *key, int value)
{
    unsigned long h = check_hash_str(key) & (m->cap - 1);
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

/* Returns the stored value, or -1 if key not present. */
static int check_hashmap_get(const CheckHashMap *m, const char *key)
{
    unsigned long h = check_hash_str(key) & (m->cap - 1);
    while (m->slots[h].used) {
        if (strcmp(m->slots[h].key, key) == 0)
            return m->slots[h].value;
        h = (h + 1) & (m->cap - 1);
    }
    return -1;
}

static int failf(char *err, size_t err_sz, const char *fmt, ...)
{
    if (err && err_sz > 0) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(err, err_sz, fmt, ap);
        va_end(ap);
    }
    return -1;
}

/* "NEW_BUF_<non-negative integer>" with nothing else trailing. */
static int is_new_buf_name(const char *name)
{
    int idx;
    int consumed;

    if (strncmp(name, "NEW_BUF_", 8) != 0)
        return 0;
    if (sscanf(name + 8, "%d%n", &idx, &consumed) != 1)
        return 0;
    if (idx < 0)
        return 0;
    return name[8 + consumed] == '\0';
}

int pd_check_legality(const PdDesign *orig, const PdDesign *mod, char *err, size_t err_sz)
{
    int i, j;
    CheckHashMap mod_names = {0};
    CheckHashMap orig_names = {0};
    int rc = 0;

    if (!orig || !mod)
        return failf(err, err_sz, "pd_check_legality: null design");
    if (mod->n_nodes <= 0)
        return failf(err, err_sz, "pd_check_legality: modified design is empty");

    /* --- Check 1: structural self-consistency of `mod` ----------------------------------- */
    for (i = 0; i < mod->n_nodes; i++) {
        const PdNode *n = &mod->nodes[i];

        if (i == 0) {
            if (n->kind != PD_NODE_ROOT || n->parent != -1)
                return failf(err, err_sz, "pd_check_legality: node 0 is not a valid root");
        } else {
            if (n->kind == PD_NODE_ROOT)
                return failf(err, err_sz, "'%s': only node 0 may be the clock root", n->name);
            if (n->parent < 0 || n->parent >= mod->n_nodes)
                return failf(err, err_sz, "'%s': parent index %d out of range", n->name, n->parent);
        }

        if (n->kind == PD_NODE_FF) {
            if (n->nchildren != 0)
                return failf(err, err_sz, "FF '%s' drives %d children (FF must be a sink)",
                            n->name, n->nchildren);
        } else if (n->nchildren < 1) {
            return failf(err, err_sz, "'%s' has no children (dangling output pin)", n->name);
        }

        if (n->kind == PD_NODE_BUF) {
            if (n->cell_idx < 0 || n->cell_idx >= mod->n_cells)
                return failf(err, err_sz, "buffer '%s' has invalid cell index %d", n->name,
                            n->cell_idx);
            if (n->nchildren > mod->cells[n->cell_idx].max_fanout)
                return failf(err, err_sz, "buffer '%s' drives %d > max_fanout %d of cell '%s'",
                            n->name, n->nchildren, mod->cells[n->cell_idx].max_fanout,
                            mod->cells[n->cell_idx].name);
        }

        for (j = 0; j < n->nchildren; j++) {
            const int cid = n->children[j];
            if (cid < 0 || cid >= mod->n_nodes)
                return failf(err, err_sz, "'%s': child index %d out of range", n->name, cid);
            if (mod->nodes[cid].parent != i)
                return failf(err, err_sz,
                            "'%s' lists '%s' as a child, but its parent pointer disagrees",
                            n->name, mod->nodes[cid].name);
        }
    }

    /* --- Check 2: globally unique names ---------------------------------------------------- */
    if (check_hashmap_init(&mod_names, mod->n_nodes) != 0)
        return failf(err, err_sz, "pd_check_legality: out of memory");
    for (i = 0; i < mod->n_nodes; i++) {
        if (check_hashmap_get(&mod_names, mod->nodes[i].name) >= 0) {
            rc = failf(err, err_sz, "duplicate component name: '%s'", mod->nodes[i].name);
            goto done;
        }
        check_hashmap_put(&mod_names, mod->nodes[i].name, i);
    }

    /* --- Check 3: existing components preserved; new ones follow naming rule -------------- */
    if (check_hashmap_init(&orig_names, orig->n_nodes) != 0) {
        rc = failf(err, err_sz, "pd_check_legality: out of memory");
        goto done;
    }
    for (i = 0; i < orig->n_nodes; i++)
        check_hashmap_put(&orig_names, orig->nodes[i].name, i);

    for (i = 0; i < orig->n_nodes; i++) {
        const int mi = check_hashmap_get(&mod_names, orig->nodes[i].name);
        if (mi < 0) {
            rc = failf(err, err_sz, "existing component '%s' was deleted or renamed",
                      orig->nodes[i].name);
            goto done;
        }
        if (mod->nodes[mi].kind != orig->nodes[i].kind) {
            rc = failf(err, err_sz, "component '%s' changed kind", orig->nodes[i].name);
            goto done;
        }
    }
    for (i = 0; i < mod->n_nodes; i++) {
        if (check_hashmap_get(&orig_names, mod->nodes[i].name) >= 0)
            continue; /* pre-existing component, already checked above */
        if (mod->nodes[i].kind != PD_NODE_BUF || !is_new_buf_name(mod->nodes[i].name)) {
            rc = failf(err, err_sz,
                      "unrecognized component '%s' (new components must be buffers named NEW_BUF_X)",
                      mod->nodes[i].name);
            goto done;
        }
    }

    /* --- Check 4: relative order of existing components unchanged ------------------------- */
    for (i = 0; i < orig->n_nodes; i++) {
        int oc, mc;

        if (orig->nodes[i].kind == PD_NODE_ROOT)
            continue;

        oc = orig->nodes[i].parent;
        mc = mod->nodes[check_hashmap_get(&mod_names, orig->nodes[i].name)].parent;

        while (oc >= 0) {
            /* Skip past any newly-inserted buffers to the next *original* ancestor. */
            while (mc >= 0 && check_hashmap_get(&orig_names, mod->nodes[mc].name) < 0)
                mc = mod->nodes[mc].parent;

            if (mc < 0) {
                rc = failf(err, err_sz, "'%s': ancestor chain shortened relative to original",
                          orig->nodes[i].name);
                goto done;
            }
            if (strcmp(orig->nodes[oc].name, mod->nodes[mc].name) != 0) {
                rc = failf(err, err_sz,
                          "'%s': existing component order changed (expected ancestor '%s', found '%s')",
                          orig->nodes[i].name, orig->nodes[oc].name, mod->nodes[mc].name);
                goto done;
            }

            oc = orig->nodes[oc].parent;
            mc = mod->nodes[mc].parent;
        }
    }

done:
    check_hashmap_free(&mod_names);
    check_hashmap_free(&orig_names);
    return rc;
}
