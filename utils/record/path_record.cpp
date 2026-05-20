#include "path_record.h"

#include "ff_record.h"

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <unordered_map>

Path::Path(FlipFlop* launch, FlipFlop* capture, float ff_d, float ss_d)
    : endpoints(launch, capture), ff_delay(ff_d), ss_delay(ss_d)
{
}

PathRecord::PathRecord(FFRecord& ff_record) : ff_record_(ff_record) {}

static bool parse_path_line(const char* line, int* launch_id, int* capture_id, float* delay)
{
    char launch_name[64];
    char capture_name[64];
    int l_id;
    int c_id;
    float d;

    if (!line || !launch_id || !capture_id || !delay)
        return false;

    if (std::sscanf(line, "Path%*d : %63s -> %63s %f", launch_name, capture_name, &d) == 3 &&
        std::sscanf(launch_name, "FF_%d", &l_id) == 1 &&
        std::sscanf(capture_name, "FF_%d", &c_id) == 1) {
        *launch_id = l_id;
        *capture_id = c_id;
        *delay = d;
        return true;
    }
    return false;
}

static std::uint64_t make_key(int launch_id, int capture_id)
{
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(launch_id)) << 32) |
           static_cast<std::uint32_t>(capture_id);
}

static void clear_ff_paths(FFRecord& ff_record)
{
    for (FlipFlop* ff : ff_record.get_ff_list()) {
        if (ff)
            ff->clear_paths();
    }
}

static void register_path_on_ffs(Path& path)
{
    if (path.endpoints.first)
        path.endpoints.first->add_path_from(&path);
    if (path.endpoints.second)
        path.endpoints.second->add_path_to(&path);
}

bool PathRecord::load_ff_delay_rpt(const std::string& rpt_path)
{
    FILE* fp = std::fopen(rpt_path.c_str(), "r");
    char line[4096];

    if (!fp)
        return false;

    clear_ff_paths(ff_record_);
    path_list_.clear();

    while (std::fgets(line, static_cast<int>(sizeof(line)), fp)) {
        int launch_id;
        int capture_id;
        float delay;

        if (parse_path_line(line, &launch_id, &capture_id, &delay)) {
            FlipFlop* launch_ff = ff_record_.lookup(launch_id);
            FlipFlop* capture_ff = ff_record_.lookup(capture_id);
            if (!launch_ff || !capture_ff)
                continue;
            path_list_.emplace_back(launch_ff, capture_ff, delay, 0.0f);
        }
    }

    std::fclose(fp);
    return !path_list_.empty();
}

const std::vector<Path>& PathRecord::get_path_list() const
{
    return path_list_;
}

bool PathRecord::load_ss_delay_rpt(const std::string& rpt_path)
{
    FILE* fp = std::fopen(rpt_path.c_str(), "r");
    char line[4096];
    std::unordered_map<std::uint64_t, size_t> path_idx;

    if (!fp)
        return false;

    for (size_t i = 0; i < path_list_.size(); ++i) {
        const Path& p = path_list_[i];
        if (!p.endpoints.first || !p.endpoints.second)
            continue;
        path_idx[make_key(p.endpoints.first->id(), p.endpoints.second->id())] = i;
    }

    while (std::fgets(line, static_cast<int>(sizeof(line)), fp)) {
        int launch_id;
        int capture_id;
        float delay;

        if (!parse_path_line(line, &launch_id, &capture_id, &delay))
            continue;

        FlipFlop* launch_ff = ff_record_.lookup(launch_id);
        FlipFlop* capture_ff = ff_record_.lookup(capture_id);
        if (!launch_ff || !capture_ff)
            continue;
        const auto key = make_key(launch_id, capture_id);
        const auto it = path_idx.find(key);

        if (it != path_idx.end()) {
            path_list_[it->second].ss_delay = delay;
        } else {
            path_list_.emplace_back(launch_ff, capture_ff, 0.0f, delay);
            path_idx[key] = path_list_.size() - 1;
        }
    }

    std::fclose(fp);
    return !path_list_.empty();
}

static void register_all_paths(std::vector<Path>& path_list)
{
    for (Path& path : path_list)
        register_path_on_ffs(path);
}

bool PathRecord::load_delay_reports(const std::string& ff_rpt_path, const std::string& ss_rpt_path)
{
    if (!load_ff_delay_rpt(ff_rpt_path))
        return false;
    if (!load_ss_delay_rpt(ss_rpt_path))
        return false;
    register_all_paths(path_list_);
    return true;
}
