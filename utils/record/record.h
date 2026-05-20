#pragma once

#include <string>

#include "buffer_record.h"
#include "graph_record.h"
#include "graphs/clock_edge.h"
#include "path_record.h"

/* Owns all parsed design data; no global record state. */
class Record {
public:
    Record();

    /* Load buf.lib, clk_tree.structure, FF_delay.rpt, SS_delay.rpt from folder. */
    void load_files(const std::string& folder);

    [[nodiscard]] bool loaded() const noexcept { return loaded_; }

    /* All BufferEdge weights use this mode (default FF). */
    void set_delay_kind(DelayKind kind) noexcept;
    [[nodiscard]] DelayKind delay_kind() const noexcept { return delay_kind_; }

    [[nodiscard]] const BufferRecord& buffers() const { return buffers_; }
    [[nodiscard]] const GraphRecord& graph() const { return graph_; }
    [[nodiscard]] const PathRecord& paths() const { return paths_; }

private:
    bool loaded_ = false;
    DelayKind delay_kind_ = DelayKind::FF;
    BufferRecord buffers_;
    GraphRecord graph_;
    PathRecord paths_;
};
