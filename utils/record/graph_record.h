#pragma once

#include <graaflib/edge.h>
#include <graaflib/graph.h>

#include <string>
#include <vector>
#include <variant>

#include "../struct/buffer.h"
#include "../struct/flipflop.h"

class ClockTreeVertex {};
class InternalNode : public ClockTreeVertex {};
class Nothing : public InternalNode {};
class BufferNode : public InternalNode {
    public:
        std::vector<Buffer*> buffers;
        float ff_delay;
        float ss_delay;
        BufferNode(std::vector<Buffer*> buffers);
        ~BufferNode() = default;
};
/* Leaf FF node in clock tree graph (distinct from ff_and_path_record's FlipFlop typedef). */
class FlipFlopNode : public ClockTreeVertex {
public:
    FlipFlopNode();
    FlipFlopNode(int id, std::string name);

    [[nodiscard]] int id() const;
    [[nodiscard]] const std::string& name() const;

private:
    int id_;
    std::string name_;
};

class ClockTreeEdge : public graaf::weighted_edge<float> {
    [[nodiscard]] float get_weight() const noexcept override;
};
class BufferEdge : public ClockTreeEdge {
    public:
        explicit BufferEdge(float delay = 0.f);
        [[nodiscard]] float get_weight() const noexcept override;
    
    private:
        float delay_;
    };

class PathEdge : public ClockTreeEdge {
public:
    explicit PathEdge(float delay = 0.f);
    [[nodiscard]] float get_weight() const noexcept override;

private:
    float delay_;
};

using ClockVertex = std::variant<Nothing, BufferNode, FlipFlopNode>;
using ClockTreeGraph = graaf::directed_graph<ClockVertex, BufferEdge>;

class GraphRecord {
public:
    GraphRecord() = default;

    /* Parse clk_tree.structure, build clock_tree, populate ff_list. */
    bool load_clk_tree(const std::string& structure_path);

    [[nodiscard]] const ClockTreeGraph& get_clock_tree() const;

private:
    ClockTreeGraph clock_tree_;
};
