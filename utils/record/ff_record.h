#pragma once

#include <vector>

#include "../struct/flipflop.h"

/* Owns FF_0..FF_N as consecutive FlipFlop pointers (index == id()). */
class FFRecord {
public:
    FFRecord() = default;
    ~FFRecord();

    FFRecord(const FFRecord&) = delete;
    FFRecord& operator=(const FFRecord&) = delete;

    /* Create ff_list_[0..id] if missing; used while building clock tree. */
    FlipFlop* ensure(int id);

    [[nodiscard]] FlipFlop* lookup(int id) const;

    [[nodiscard]] const std::vector<FlipFlop*>& get_ff_list() const { return ff_list_; }

private:
    std::vector<FlipFlop*> ff_list_;
};
