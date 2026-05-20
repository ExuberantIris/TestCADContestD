#include "graph_record.h"

#include "ff_and_path_record.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

BufferNode::BufferNode(std::vector<Buffer*> buffers)
    : buffers(std::move(buffers)), ff_delay(0.f), ss_delay(0.f)
{
}

float ClockTreeEdge::get_weight() const noexcept
{
    return 0.f;
}

BufferEdge::BufferEdge(float delay) : delay_(delay) {}

float BufferEdge::get_weight() const noexcept
{
    return delay_;
}

PathEdge::PathEdge(float delay) : delay_(delay) {}

float PathEdge::get_weight() const noexcept
{
    return delay_;
}

FlipFlopNode::FlipFlopNode() : id_(-1) {}

FlipFlopNode::FlipFlopNode(int id, std::string name) : id_(id), name_(std::move(name)) {}

int FlipFlopNode::id() const
{
    return id_;
}

const std::string& FlipFlopNode::name() const
{
    return name_;
}

static void ensure_ff_index(int id)
{
    if (id < 0)
        return;

    if (static_cast<int>(ff_list.size()) <= id)
        ff_list.resize(static_cast<size_t>(id + 1), nullptr);

    for (int i = 0; i <= id; ++i) {
        if (!ff_list[static_cast<size_t>(i)])
            ff_list[static_cast<size_t>(i)] = new FlipFlop(i);
    }
}

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

bool GraphRecord::load_clk_tree(const std::string& structure_path)
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
            const graaf::vertex_id_t root_id = clock_tree_.add_vertex(ClockVertex{Nothing{}});
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
            ensure_ff_index(ff_id);
            node_id = clock_tree_.add_vertex(ClockVertex{FlipFlopNode{ff_id, name}});
            clock_tree_.add_edge(parent_id, node_id, BufferEdge{0.f});
        } else if (is_buffer_name(name)) {
            node_id = clock_tree_.add_vertex(ClockVertex{BufferNode{std::vector<Buffer*>{}}});
            clock_tree_.add_edge(parent_id, node_id, BufferEdge{0.f});
        } else {
            node_id = clock_tree_.add_vertex(ClockVertex{Nothing{}});
            clock_tree_.add_edge(parent_id, node_id, BufferEdge{0.f});
        }

        stack.push_back(node_id);
    }

    std::fclose(fp);
    return root_done && clock_tree_.vertex_count() > 0;
}

const ClockTreeGraph& GraphRecord::get_clock_tree() const
{
    return clock_tree_;
}
