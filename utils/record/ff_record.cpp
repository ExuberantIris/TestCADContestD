#include "ff_record.h"

FFRecord::~FFRecord()
{
    for (FlipFlop* ff : ff_list_) {
        delete ff;
    }
}

FlipFlop* FFRecord::ensure(int id)
{
    if (id < 0)
        return nullptr;

    if (static_cast<int>(ff_list_.size()) <= id)
        ff_list_.resize(static_cast<size_t>(id + 1), nullptr);

    for (int i = 0; i <= id; ++i) {
        if (!ff_list_[static_cast<size_t>(i)])
            ff_list_[static_cast<size_t>(i)] = new FlipFlop(i); /* id == index */
    }

    return ff_list_[static_cast<size_t>(id)];
}

FlipFlop* FFRecord::lookup(int id) const
{
    if (id < 0 || static_cast<size_t>(id) >= ff_list_.size())
        return nullptr;
    return ff_list_[static_cast<size_t>(id)];
}
