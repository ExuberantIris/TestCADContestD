#pragma once

#include <string>
#include <variant>
#include <vector>

#include "../struct/buffer.h"

class ClockTreeVertex {
    public:
        virtual std::string show() const = 0;
};
class InternalNode : public ClockTreeVertex {};
class Nothing : public InternalNode {
    public:
        std::string show() const override;
};
class BufferNode : public InternalNode {
    public:
        std::vector<Buffer*> buffers;
        float ff_delay;
        float ss_delay;
        BufferNode(std::vector<Buffer*> buffers);
        ~BufferNode() = default;
        std::string show() const override;
};
/* Leaf FF node in clock tree graph (distinct from ff_and_path_record's FlipFlop typedef). */
class FlipFlopNode : public ClockTreeVertex {
public:
    FlipFlopNode();
    FlipFlopNode(int id, std::string name);

    [[nodiscard]] int id() const;
    [[nodiscard]] const std::string& name() const;
    std::string show() const override;

private:
    int id_;
    std::string name_;
};

/* Concrete storage type for graaf (cannot store abstract ClockTreeVertex). */
using ClockVertex = std::variant<Nothing, BufferNode, FlipFlopNode>;