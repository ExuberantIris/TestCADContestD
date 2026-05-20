#pragma once

#include <utility>
#include <vector>

class Path;
class FlipFlopNode;

/* FF_0..FF_N; graph node and path lists filled when building graph / paths. */
class FlipFlop {
public:
    /* first: paths launching from this FF; second: paths capturing at this FF */
    using PathLists = std::pair<std::vector<Path*>, std::vector<Path*>>;

    explicit FlipFlop(int id);

    [[nodiscard]] int id() const noexcept { return id_; }

    [[nodiscard]] FlipFlopNode* graph_node() const noexcept { return graph_node_; }
    void set_graph_node(FlipFlopNode* node) noexcept { graph_node_ = node; }

    [[nodiscard]] PathLists& paths() noexcept { return paths_; }
    [[nodiscard]] const PathLists& paths() const noexcept { return paths_; }

    void add_path_from(Path* path);
    void add_path_to(Path* path);
    void clear_paths() noexcept;

private:
    int id_;
    FlipFlopNode* graph_node_ = nullptr;
    PathLists paths_;
};
