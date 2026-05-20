#include "pd_util.h"

#include "pd_types.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

char *pd_trim(char *s)
{
    char *end;

    while (*s && isspace((unsigned char)*s))
        s++;
    if (*s == 0)
        return s;
    end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end))
        *end-- = '\0';
    return s;
}

int pd_streq(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
}

int pd_join_path(char *out, size_t out_sz, const char *dir, const char *file)
{
    size_t dlen = strlen(dir);
    int need_slash = (dlen > 0 && dir[dlen - 1] != '/');

    if (snprintf(out, out_sz, need_slash ? "%s/%s" : "%s%s", dir, file) >= (int)out_sz)
        return -1;
    return 0;
}

int pd_find_node_by_name(const PdDesign *d, const char *name)
{
    int i;

    for (i = 0; i < d->n_nodes; i++) {
        if (pd_streq(d->nodes[i].name, name))
            return i;
    }
    return -1;
}
