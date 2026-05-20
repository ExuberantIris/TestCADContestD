#pragma once

#include <string>
#include <utility>
#include <vector>

#include "../struct/flipflop.h"

class FFRecord;

class Path {
public:
    std::pair<FlipFlop*, FlipFlop*> endpoints;
    float ff_delay;
    float ss_delay;

    Path(FlipFlop* launch, FlipFlop* capture, float ff_d, float ss_d = 0.0f);
};

class PathRecord {
public:
    explicit PathRecord(FFRecord& ff_record);

    PathRecord() = delete;

    /* Parse FF_delay.rpt; endpoints reference ff_list built by GraphRecord. */
    bool load_ff_delay_rpt(const std::string& rpt_path);
    /* Parse SS_delay.rpt and update ss_delay for existing/generated paths. */
    bool load_ss_delay_rpt(const std::string& rpt_path);
    /* Convenience API to parse both reports. */
    bool load_delay_reports(const std::string& ff_rpt_path, const std::string& ss_rpt_path);

    [[nodiscard]] const std::vector<Path>& get_path_list() const;

private:
    FFRecord& ff_record_;
    std::vector<Path> path_list_;
};