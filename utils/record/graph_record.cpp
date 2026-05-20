#include "graph_record.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <variant>
#include <vector>

static bool parse_ff_id(const char* name, int* id_out)
{
    return name && id_out && std::sscanf(name, "FF_%d", id_out) == 1;
}

static bool is_buffer_name(const char* name)
{
    return name && (std::strncmp(name, "BUF_", 4) == 0 || std::strncmp(name, "NEW_BUF_", 8) == 0);
}

static bool parse_structure_line(const char* line, int* level, char* name, char* cell, int* is_sink)
{
    char sink_tag[64];

    if (!line || !level || !name || !cell || !is_sink)
        return false;

    sink_tag[0] = '\0';
    *is_sink = 0;

    if (std::sscanf(line, "[%d] %127s (%127[^)]) (%127[^)])", level, name, cell, sink_tag) >= 3 ||
        std::sscanf(line, "[%d] %127s (%127[^)])", level, name, cell) >= 3) {
        if (sink_tag[0] != '\0')
            *is_sink = 1;
        return true;
    }
    return false;
}

static void annotate_buffer_edge_delays(ClockTreeGraph& graph)
{
    for (const auto& [vid, _] : graph.get_vertices()) {
        ClockVertex& vertex = graph.get_vertex(vid);
        const auto& neighbors = graph.get_neighbors(vid);
        int fanout = static_cast<int>(neighbors.size());
        float edge_ff_delay = 0.f;
        float edge_ss_delay = 0.f;

        if (auto* buf_node = std::get_if<BufferNode>(&vertex)) {
            if (fanout < 1)
                fanout = 1;
            const Buffer* buf = buf_node->buffers.empty() ? nullptr : buf_node->buffers[0];
            if (buf) {
                buf_node->ff_delay = buf->get_ff_delay(fanout);
                buf_node->ss_delay = buf->get_ss_delay(fanout);
                edge_ff_delay = buf_node->ff_delay;
                edge_ss_delay = buf_node->ss_delay;
            }
        }

        for (graaf::vertex_id_t child : neighbors) {
            if (!graph.has_edge(vid, child))
                continue;
            ClockEdge& edge = graph.get_edge(vid, child);
            if (std::holds_alternative<BufferEdge>(edge))
                std::get<BufferEdge>(edge) = BufferEdge{edge_ff_delay, edge_ss_delay};
        }
    }
}

bool GraphRecord::load_clk_tree(const std::string& structure_path,
                                const BufferRecord& buffer_record)
{
    FILE* fp = std::fopen(structure_path.c_str(), "r");
    if (!fp)
        return false;

    clock_tree_ = ClockTreeGraph{};

    char line[4096];
    std::vector<graaf::vertex_id_t> stack;
    int root_done = 0;

    while (std::fgets(line, static_cast<int>(sizeof(line)), fp)) {
        char* p = line;
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
            ++p;
        if (*p == '\0' || *p == '#')
            continue;

        if (std::strncmp(p, "Root:", 5) == 0) {
            const graaf::vertex_id_t root_id = clock_tree_.add_vertex(Nothing{});
            stack.push_back(root_id);
            root_done = 1;
            continue;
        }

        if (!root_done)
            continue;

        int level = 0;
        char name[128];
        char cell[128];
        int is_sink = 0;
        int ff_id = -1;

        if (!parse_structure_line(p, &level, name, cell, &is_sink))
            continue;

        while (static_cast<int>(stack.size()) > level)
            stack.pop_back();
        if (stack.empty())
            continue;

        const graaf::vertex_id_t parent_id = stack.back();
        graaf::vertex_id_t node_id;

        if (parse_ff_id(name, &ff_id)) {
            FlipFlop* ff = ff_record_.ensure(ff_id);
            node_id = clock_tree_.add_vertex(FlipFlopNode{ff_id, name});
            if (ff) {
                auto& node = std::get<FlipFlopNode>(clock_tree_.get_vertex(node_id));
                ff->set_graph_node(&node);
            }
            clock_tree_.add_edge(parent_id, node_id, BufferEdge{});
        } else if (is_buffer_name(name)) {
            std::vector<Buffer*> bufs;
            if (const Buffer* buf = buffer_record.find_by_name(cell))
                bufs.push_back(const_cast<Buffer*>(buf));
            node_id = clock_tree_.add_vertex(BufferNode{std::move(bufs)});
            clock_tree_.add_edge(parent_id, node_id, BufferEdge{});
        } else {
            node_id = clock_tree_.add_vertex(Nothing{});
            clock_tree_.add_edge(parent_id, node_id, BufferEdge{});
        }

        stack.push_back(node_id);
    }

    std::fclose(fp);

    if (!root_done || clock_tree_.vertex_count() == 0)
        return false;

    annotate_buffer_edge_delays(clock_tree_);
    return true;
}

const ClockTreeGraph& GraphRecord::get_clock_tree() const
{
    return clock_tree_;
}

const FFRecord& GraphRecord::get_ff_record() const
{
    return ff_record_;
}

FFRecord& GraphRecord::get_ff_record()
{
    return ff_record_;
}
