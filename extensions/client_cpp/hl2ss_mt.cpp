
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

bool source::status(std::exception& error)
{
    if (!m_enable) { error = m_error; }
    return m_enable;
}

void source::stop()
{
    m_enable = false;
    if (m_thread.joinable()) { m_thread.join(); }
}

std::shared_ptr<hl2ss::packet> source::get_packet(int64_t& frame_index, int32_t& status)
{
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_buffer.get(frame_index, status);
}

std::shared_ptr<hl2ss::packet> source::get_packet(uint64_t timestamp, int32_t mode, int64_t& frame_index, int32_t& status)
{
    std::shared_lock<std::shared_mutex> lock(m_mutex);

    int64_t begin = m_buffer.index_begin();
    int64_t end   = m_buffer.index_end();
    int32_t _;

    if (end < begin) { frame_index = -1; status = 1; return std::shared_ptr<hl2ss::packet>(); }

    std::shared_ptr<hl2ss::packet> data_begin = m_buffer.get(begin, _);
    std::shared_ptr<hl2ss::packet> data_end   = m_buffer.get(end,   _);

    if (data_begin->timestamp >= timestamp) { frame_index = begin; status = ((mode < 0) && (data_begin->timestamp > timestamp)) ? -1 : 0; return data_begin; }
    if (data_end->timestamp   <= timestamp) { frame_index = end;   status = ((mode > 0) && (data_end->timestamp   < timestamp)) ?  1 : 0; return data_end; }

    status = 0;

    while ((end - begin) > 1)
    {
    int64_t i = (end + begin) / 2;
    std::shared_ptr<hl2ss::packet> data_i = m_buffer.get(i, _);

    if      (data_i->timestamp < timestamp) { begin       = i; data_begin = data_i; }
    else if (data_i->timestamp > timestamp) { end         = i; data_end   = data_i; }
    else                                    { frame_index = i; return       data_i; }
    }
    
    if (mode < 0) { frame_index = begin; return data_begin; }
    if (mode > 0) { frame_index = end;   return data_end; }

    if ((timestamp - data_begin->timestamp) <= (data_end->timestamp - timestamp)) { frame_index = begin; return data_begin; }
    else                                                                          { frame_index = end;   return data_end; }
}
}
}
