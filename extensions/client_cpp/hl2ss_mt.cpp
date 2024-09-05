
#include "hl2ss_mt.h"

namespace hl2ss
{
namespace mt
{
void source::capture()
{
    std::shared_ptr<hl2ss::packet> data = m_rx->get_next_packet();
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    m_buffer.insert(data, data->timestamp);
}

void source::run()
{
    try
    {
    m_rx->open();
    while (m_enable) { capture(); }
    }
    catch(const std::exception& e)
    {
    m_error = e;
    m_enable = false;
    }
    m_rx->close();
}

source::source(int64_t buffer_size, std::unique_ptr<hl2ss::rx> rx) : m_buffer(buffer_size), m_rx(std::move(rx)), m_enable(false)
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

bool source::status(std::exception& error)
{
    bool enable = m_enable;
    if (!enable) { error = m_error; }
    return enable;
}

void source::stop()
{
    m_enable = false;
    if (m_thread.joinable()) { m_thread.join(); }
}

std::shared_ptr<hl2ss::packet> source::get_packet(int64_t& frame_stamp, int32_t& status)
{
    uint64_t timestamp;
    std::shared_ptr<hl2ss::packet> data;
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    status = m_buffer.get(frame_stamp, data, timestamp, frame_stamp);
    return data;
}

std::shared_ptr<hl2ss::packet> source::get_packet(uint64_t timestamp, int32_t time_preference, bool tiebreak_right, int64_t& frame_stamp, int32_t& status)
{
    std::shared_ptr<hl2ss::packet> data;
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    status = m_buffer.get(timestamp, time_preference, tiebreak_right, data, timestamp, frame_stamp);
    return data;
}
}
}
