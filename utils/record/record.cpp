#include "record.h"

#include <filesystem>

Record::Record() : paths_(graph_.get_ff_record())
{
    set_delay_kind(DelayKind::FF);
}

void Record::set_delay_kind(DelayKind kind) noexcept
{
    delay_kind_ = kind;
    ClockTreeEdge::set_active_delay_kind(kind);
}

void Record::load_files(const std::string& folder)
{
    loaded_ = false;

    const std::filesystem::path dir(folder);
    if (!std::filesystem::is_directory(dir))
        return;

    const std::string buf_path = (dir / "buf.lib").string();
    const std::string structure_path = (dir / "clk_tree.structure").string();
    const std::string ff_rpt_path = (dir / "FF_delay.rpt").string();
    const std::string ss_rpt_path = (dir / "SS_delay.rpt").string();

    if (!buffers_.load_from_file(buf_path))
        return;
    if (!graph_.load_clk_tree(structure_path, buffers_))
        return;
    if (!paths_.load_delay_reports(ff_rpt_path, ss_rpt_path))
        return;

    loaded_ = true;
}
