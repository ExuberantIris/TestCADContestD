#ifndef BUFFER_RECORD_H
#define BUFFER_RECORD_H

#include <string>
#include <vector>

#include "../struct/buffer.h"

class BufferRecord {
public:
    BufferRecord() = default;
    ~BufferRecord();

    BufferRecord(const BufferRecord&) = delete;
    BufferRecord& operator=(const BufferRecord&) = delete;

    BufferRecord(BufferRecord&& other) noexcept;
    BufferRecord& operator=(BufferRecord&& other) noexcept;

    /* Parse all `cell (...) { ... }` blocks in buf.lib */
    bool load_from_file(const std::string& buf_lib_path);

    const std::vector<Buffer*>& get_buffer_list() const;
    const Buffer* find_by_name(const std::string& cell_name) const;
    void clear();

private:
    std::vector<Buffer*> buffer_list_;
};

#endif
