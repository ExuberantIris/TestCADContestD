#ifndef PD_TYPES_H
#define PD_TYPES_H

#include "pd_config.h"

typedef enum {
    PD_NODE_ROOT = 0,
    PD_NODE_BUF  = 1,
    PD_NODE_FF   = 2
} PdNodeKind;

typedef struct {
    char name[PD_MAX_NAME];
    double width;
    double height;
    double ss_delay[PD_MAX_FANOUT_TBL];
    double ff_delay[PD_MAX_FANOUT_TBL];
    int max_fanout;
} PdCell;

typedef struct {
    int id;
    char name[PD_MAX_NAME];
    char cell[PD_MAX_NAME];
    PdNodeKind kind;
    int level;
    int is_sink;
    int parent;
    int *children;
    int nchildren;
    int cell_idx;
    int fanout;
    double d_clk_ss;
    double d_clk_ff;
    double area;
} PdNode;

typedef struct {
    char launch[PD_MAX_NAME];
    char capture[PD_MAX_NAME];
    int launch_id;
    int capture_id;
    double data_ss;
    double data_ff;
    double slack_setup_ss;
    double slack_hold_ff;
} PdPath;

typedef struct {
    PdNode *nodes;
    int n_nodes;
    int cap_nodes;
    char root_name[PD_MAX_NAME];

    PdCell cells[PD_MAX_CELLS];
    int n_cells;

    PdPath *paths;
    int n_paths;
    int cap_paths;

    double clock_period;
    double t_setup;
    double t_hold;

    double wns_setup_ss;
    double tns_setup_ss;
    double wns_hold_ff;
    double tns_hold_ff;
    double total_area;
} PdDesign;

#endif
