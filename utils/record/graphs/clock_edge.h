#pragma once

#include <graaflib/edge.h>

#include <string>
#include <variant>

enum class DelayKind { FF, SS };

class ClockTreeEdge : public graaf::weighted_edge<float> {
public:
    static void set_active_delay_kind(DelayKind kind) noexcept;
    [[nodiscard]] static DelayKind active_delay_kind() noexcept;

    [[nodiscard]] float get_weight() const noexcept override;
    [[nodiscard]] virtual std::string show() const = 0;

protected:
    [[nodiscard]] static float pick_delay(float ff_delay, float ss_delay) noexcept;

private:
    static DelayKind active_delay_kind_;
};

class BufferEdge : public ClockTreeEdge {
public:
    BufferEdge(float ff_delay = 0.f, float ss_delay = 0.f);

    [[nodiscard]] float ff_delay() const noexcept { return ff_delay_; }
    [[nodiscard]] float ss_delay() const noexcept { return ss_delay_; }
    [[nodiscard]] float get_weight() const noexcept override;
    [[nodiscard]] std::string show() const override;

private:
    float ff_delay_;
    float ss_delay_;
};

class PathEdge : public ClockTreeEdge {
public:
    explicit PathEdge(float delay = 0.f);
    [[nodiscard]] float get_weight() const noexcept override;
    [[nodiscard]] std::string show() const override;

private:
    float delay_;
};

/* Concrete storage type for graaf (clock tree uses BufferEdge; PathEdge reserved). */
using ClockEdge = std::variant<BufferEdge, PathEdge>;

[[nodiscard]] inline std::string clock_edge_dot_attrs(const ClockEdge& edge)
{
    const std::string label = std::visit([](const auto& e) { return e.show(); }, edge);
    return "label=\"" + label + "\"";
}
