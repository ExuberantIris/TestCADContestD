#include "flipflop.h"

FlipFlop::FlipFlop(int id) : id_(id) {}

void FlipFlop::add_path_from(Path* path)
{
    if (path)
        paths_.first.push_back(path);
}

void FlipFlop::add_path_to(Path* path)
{
    if (path)
        paths_.second.push_back(path);
}

void FlipFlop::clear_paths() noexcept
{
    paths_.first.clear();
    paths_.second.clear();
}
