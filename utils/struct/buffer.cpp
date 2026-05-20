#include "buffer.h"

#include <cctype>
#include <cstdio>
#include <sstream>

namespace {

/** Read one line from fp into line (no trailing '\n'). Returns false at EOF with nothing read. */
bool read_line(FILE* fp, std::string& line)
{
    line.clear();
    int c = 0;

    while ((c = std::fgetc(fp)) != EOF) {
        if (c == '\n')
            break;
        line.push_back(static_cast<char>(c));
    }
    return !line.empty() || c != EOF;
}

std::string& trim(std::string& s)
{
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
        s.pop_back();
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
        ++start;
    if (start > 0)
        s.erase(0, start);
    return s;
}

bool parse_cell_header(const std::string& raw, std::string& name_out)
{
    std::string p = raw;
    trim(p);

    if (p.size() < 6 || p.compare(0, 4, "cell") != 0)
        return false;

    size_t i = 4;
    while (i < p.size() && std::isspace(static_cast<unsigned char>(p[i])))
        ++i;
    if (i >= p.size() || p[i] != '(')
        return false;
    ++i;

    name_out.clear();
    while (i < p.size() && p[i] != ')') {
        if (!std::isspace(static_cast<unsigned char>(p[i])))
            name_out.push_back(p[i]);
        ++i;
    }
    if (i >= p.size() || p[i] != ')')
        return false;
    ++i;
    while (i < p.size() && std::isspace(static_cast<unsigned char>(p[i])))
        ++i;
    return i < p.size() && p[i] == '{';
}

void parse_float_line_tail(const std::string& tail, std::vector<float>& out)
{
    std::istringstream iss(tail);
    float v;

    out.clear();
    while (iss >> v)
        out.push_back(v);
}

bool parse_size_line(const std::string& p, float& w, float& h)
{
    /* "SIZE 0.356 BY 0.356" after optional trim */
    if (p.size() < 5 || p.compare(0, 4, "SIZE") != 0)
        return false;
    std::istringstream iss(p.substr(4));
    std::string by_token;

    iss >> w >> by_token >> h;
    if (iss.fail())
        return false;
    for (auto& ch : by_token)
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    return by_token == "BY";
}

} /* namespace */

Buffer::Buffer(std::string cell_name, std::vector<float> ss, std::vector<float> ff,
               float cell_area)
    : name(std::move(cell_name))
    , ss_delays(std::move(ss))
    , ff_delays(std::move(ff))
    , area(cell_area)
{
    const int nss = static_cast<int>(ss_delays.size());
    const int nff = static_cast<int>(ff_delays.size());
    max_fanout = nss > nff ? nss : nff;
    if (max_fanout < 1)
        max_fanout = 1;
}

int Buffer::clamp_fanout_index(int fanout, int max_f) const
{
    if (fanout < 1)
        fanout = 1;
    if (fanout > max_f)
        fanout = max_f;
    return fanout - 1;
}

float Buffer::get_ss_delay(int fanout) const
{
    if (ss_delays.empty())
        return 0.f;
    return ss_delays[static_cast<size_t>(clamp_fanout_index(fanout, max_fanout))];
}

float Buffer::get_ff_delay(int fanout) const
{
    if (ff_delays.empty())
        return 0.f;
    return ff_delays[static_cast<size_t>(clamp_fanout_index(fanout, max_fanout))];
}

Buffer* Buffer::parse_buffer(FILE* fp)
{
    std::string line;
    std::string name;
    float w = 0.f;
    float h = 0.f;
    std::vector<float> ss;
    std::vector<float> ff;
    int in_cell = 0;

    if (!fp)
        return nullptr;

    while (read_line(fp, line)) {
        trim(line);
        if (line.empty() || line[0] == '#')
            continue;

        if (!in_cell) {
            if (parse_cell_header(line, name)) {
                in_cell = 1;
                ss.clear();
                ff.clear();
                w = h = 0.f;
            }
            continue;
        }

        /* inside cell { ... } */
        if (parse_size_line(line, w, h))
            continue;
        if (line.size() >= 8 && line.compare(0, 8, "SS_DELAY") == 0) {
            parse_float_line_tail(line.substr(8), ss);
            continue;
        }
        if (line.size() >= 8 && line.compare(0, 8, "FF_DELAY") == 0) {
            parse_float_line_tail(line.substr(8), ff);
            continue;
        }
        if (line[0] == '}') {
            const float cell_area = w * h;
            return new Buffer(std::move(name), std::move(ss), std::move(ff), cell_area);
        }
    }

    return nullptr;
}
