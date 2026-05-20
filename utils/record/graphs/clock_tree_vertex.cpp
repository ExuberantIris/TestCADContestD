#include "clock_tree_vertex.h"

#include <ranges>

std::string Nothing::show() const
{
    return "Nothing";
}

BufferNode::BufferNode(std::vector<Buffer*> buffers)
    : buffers(std::move(buffers)), ff_delay(0.f), ss_delay(0.f)
{
}

std::string BufferNode::show() const
{
    std::string label = "BufferNode <br /> ";
    std::ranges::reverse_view reverse_buffers(this->buffers);
    std::string arrow = "";
    for (const Buffer* buffer : reverse_buffers) {
        label += arrow + buffer->name;
        arrow = "->";
    }
    return label;
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

std::string FlipFlopNode::show() const
{
    return name_;
}