#include "clock_edge.h"

#include <cstdio>
#include <string>

DelayKind ClockTreeEdge::active_delay_kind_ = DelayKind::FF;

void ClockTreeEdge::set_active_delay_kind(DelayKind kind) noexcept
{
    active_delay_kind_ = kind;
}

DelayKind ClockTreeEdge::active_delay_kind() noexcept
{
    return active_delay_kind_;
}

float ClockTreeEdge::pick_delay(float ff_delay, float ss_delay) noexcept
{
    return active_delay_kind_ == DelayKind::SS ? ss_delay : ff_delay;
}

float ClockTreeEdge::get_weight() const noexcept
{
    return 0.f;
}

BufferEdge::BufferEdge(float ff_delay, float ss_delay) : ff_delay_(ff_delay), ss_delay_(ss_delay) {}

float BufferEdge::get_weight() const noexcept
{
    return pick_delay(ff_delay_, ss_delay_);
}

std::string BufferEdge::show() const
{
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%f/%f", ff_delay_,
                  ss_delay_);
    return buf;
}

PathEdge::PathEdge(float delay) : delay_(delay) {}

float PathEdge::get_weight() const noexcept
{
    return delay_;
}

std::string PathEdge::show() const
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "delay=%g", static_cast<double>(delay_));
    return buf;
}
