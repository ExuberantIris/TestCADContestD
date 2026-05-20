#include "buffer_record.h"

#include <cstdio>
#include <utility>

BufferRecord::~BufferRecord()
{
    clear();
}

BufferRecord::BufferRecord(BufferRecord&& other) noexcept
    : buffer_list_(std::move(other.buffer_list_))
{
    other.buffer_list_.clear();
}

BufferRecord& BufferRecord::operator=(BufferRecord&& other) noexcept
{
    if (this != &other) {
        clear();
        buffer_list_ = std::move(other.buffer_list_);
        other.buffer_list_.clear();
    }
    return *this;
}

bool BufferRecord::load_from_file(const std::string& buf_lib_path)
{
    FILE* fp = std::fopen(buf_lib_path.c_str(), "r");
    if (!fp)
        return false;

    clear();
    while (true) {
        Buffer* b = Buffer::parse_buffer(fp);
        if (!b)
            break;
        buffer_list_.push_back(b);
    }

    std::fclose(fp);
    return !buffer_list_.empty();
}

const std::vector<Buffer*>& BufferRecord::get_buffer_list() const
{
    return buffer_list_;
}

const Buffer* BufferRecord::find_by_name(const std::string& cell_name) const
{
    for (const Buffer* b : buffer_list_) {
        if (b && b->name == cell_name)
            return b;
    }
    return nullptr;
}

void BufferRecord::clear()
{
    for (Buffer* b : buffer_list_)
        delete b;
    buffer_list_.clear();
}
