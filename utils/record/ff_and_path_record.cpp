#include "ff_and_path_record.h"

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <unordered_map>

std::vector<FlipFlop*> ff_list;
std::vector<Path> path_list;

Path::Path(FlipFlop* launch, FlipFlop* capture, float ff_d, float ss_d)
    : endpoints(launch, capture), ff_delay(ff_d), ss_delay(ss_d)
{
}

static FlipFlop* find_or_create_ff(int id)
{
    if (id < 0)
        return nullptr;

    if (static_cast<int>(ff_list.size()) <= id)
        ff_list.resize(static_cast<size_t>(id + 1), nullptr);

    for (int i = 0; i <= id; ++i) {
        if (!ff_list[static_cast<size_t>(i)])
            ff_list[static_cast<size_t>(i)] = new FlipFlop(i);
    }

    return ff_list[static_cast<size_t>(id)];
}

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

bool FFAndPathRecord::load_ff_delay_rpt(const std::string& rpt_path)
{
    FILE* fp = std::fopen(rpt_path.c_str(), "r");
    char line[4096];

    if (!fp)
        return false;

    path_list.clear();

    while (std::fgets(line, static_cast<int>(sizeof(line)), fp)) {
        int launch_id;
        int capture_id;
        float delay;

        if (parse_path_line(line, &launch_id, &capture_id, &delay)) {
            FlipFlop* launch_ff = find_or_create_ff(launch_id);
            FlipFlop* capture_ff = find_or_create_ff(capture_id);
            path_list.emplace_back(launch_ff, capture_ff, delay, 0.0f);
        }
    }

    std::fclose(fp);
    return !path_list.empty();
}

const std::vector<Path>& FFAndPathRecord::get_path_list() const
{
    return path_list;
}

bool FFAndPathRecord::load_ss_delay_rpt(const std::string& rpt_path)
{
    FILE* fp = std::fopen(rpt_path.c_str(), "r");
    char line[4096];
    std::unordered_map<std::uint64_t, size_t> path_idx;

    if (!fp)
        return false;

    for (size_t i = 0; i < path_list.size(); ++i) {
        const Path& p = path_list[i];
        if (!p.endpoints.first || !p.endpoints.second)
            continue;
        path_idx[make_key(*p.endpoints.first, *p.endpoints.second)] = i;
    }

    while (std::fgets(line, static_cast<int>(sizeof(line)), fp)) {
        int launch_id;
        int capture_id;
        float delay;

        if (!parse_path_line(line, &launch_id, &capture_id, &delay))
            continue;

        FlipFlop* launch_ff = find_or_create_ff(launch_id);
        FlipFlop* capture_ff = find_or_create_ff(capture_id);
        const auto key = make_key(launch_id, capture_id);
        const auto it = path_idx.find(key);

        if (it != path_idx.end()) {
            path_list[it->second].ss_delay = delay;
        } else {
            path_list.emplace_back(launch_ff, capture_ff, 0.0f, delay);
            path_idx[key] = path_list.size() - 1;
        }
    }

    std::fclose(fp);
    return !path_list.empty();
}

bool FFAndPathRecord::load_delay_reports(const std::string& ff_rpt_path, const std::string& ss_rpt_path)
{
    if (!load_ff_delay_rpt(ff_rpt_path))
        return false;
    return load_ss_delay_rpt(ss_rpt_path);
}
