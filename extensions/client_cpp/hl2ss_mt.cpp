
#include "hl2ss_mt.h"

namespace hl2ss
{
namespace mt
{
void source::capture()
{
    std::shared_ptr<hl2ss::packet> data = m_rx->get_next_packet();
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    m_buffer.insert(data);
}

void source::run()
{
    m_rx->open();
    while (m_enable) { capture(); }
    m_rx->close();
}

source::source(uint64_t buffer_size, std::unique_ptr<hl2ss::rx> rx) : m_buffer(buffer_size), m_rx(std::move(rx)), m_enable(false)
{
}

source::~source()
{
    stop();
}

void source::start()
{
    m_enable = true;
    m_thread = std::thread(&source::run, this);
}

void source::stop()
{
    m_enable = false;
    if (m_thread.joinable()) { m_thread.join(); }
}

hl2ss::rx const* source::get_rx()
{
    return m_rx.get();
}

std::shared_ptr<hl2ss::packet> source::get_packet(int64_t& frame_stamp, int32_t& status)
{
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_buffer.get(frame_stamp, status);
}
}
}
