#ifndef BUFFER_H
#define BUFFER_H

#include <cstdio>
#include <string>
#include <vector>

class Buffer {
public:
    std::string name;
    std::vector<float> ss_delays;
    std::vector<float> ff_delays;
    float area;
    int max_fanout;

    Buffer(std::string cell_name, std::vector<float> ss, std::vector<float> ff, float cell_area);
    ~Buffer() = default;

    float get_ss_delay(int fanout) const;
    float get_ff_delay(int fanout) const;

    /** Read one `cell (...) { ... }` block from fp; returns nullptr at EOF or on incomplete block. */
    static Buffer* parse_buffer(FILE* fp);

private:
    int clamp_fanout_index(int fanout, int max_f) const;
};

#endif
