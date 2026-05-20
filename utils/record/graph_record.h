#pragma once

#include <graaflib/edge.h>
#include <graaflib/graph.h>

#include <string>

#include "buffer_record.h"
#include "ff_record.h"
#include "graphs/clock_vertex.h"
#include "graphs/clock_edge.h"

using ClockTreeGraph = graaf::directed_graph<ClockVertex, ClockEdge>;

class GraphRecord {
public:
    GraphRecord() = default;

    /* Parse clk_tree.structure, build clock_tree, populate ff_list via ff_record_. */
    bool load_clk_tree(const std::string& structure_path, const BufferRecord& buffer_record);

    [[nodiscard]] const ClockTreeGraph& get_clock_tree() const;
    [[nodiscard]] const FFRecord& get_ff_record() const;
    [[nodiscard]] FFRecord& get_ff_record();

private:
    ClockTreeGraph clock_tree_;
    FFRecord ff_record_;
};
