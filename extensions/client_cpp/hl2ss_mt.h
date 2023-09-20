
#include <shared_mutex>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include "hl2ss.h"

namespace hl2ss
{
namespace mt
{
//------------------------------------------------------------------------------
// Buffer
//------------------------------------------------------------------------------

template <typename T>
class ring_buffer
{
private:
    static int64_t const TOO_OLD = -2;
    static int64_t const TOO_NEW = -1;

    int64_t m_frame_index;
    int64_t m_write_index;
    std::vector<T> m_data;

    int64_t to_index(int64_t frame_index)
    {
        if (frame_index < 0) { return ((frame_index + m_data.size() - 1 - m_frame_index) >= 0) ? TOO_NEW : TOO_OLD; }
        int64_t d = m_frame_index - frame_index;
        if (d < 0) { return TOO_NEW; }
        int64_t x = m_write_index - d;
        if (x >= 0) {return x; }
        x += m_data.size();
        if (x > m_write_index) { return x; }
        return TOO_OLD;
    }

public:
    ring_buffer(uint64_t buffer_size)
    {
        m_data.resize(buffer_size);
        m_frame_index = -1;
        m_write_index = -1;
    }

    void insert(T const& data)
    {
        m_write_index = (m_write_index + 1) % m_data.size();
        m_frame_index++;
        m_data[m_write_index] = data;
    }

    T get(int64_t& frame_index, int32_t& status)
    {
        if (frame_index < 0) { frame_index += (m_frame_index + 1); }
        int64_t index = to_index(frame_index);
        switch (index)
        {
        case TOO_NEW: status =  1; return T();
        case TOO_OLD: status = -1; return T();
        default:      status =  0; return m_data[index];
        }
    }

    int64_t index_begin()
    {
        return std::max(0LL, m_frame_index + 1LL - (int64_t)m_data.size());
    }

    int64_t index_end()
    {
        return m_frame_index;
    }
};

//------------------------------------------------------------------------------
// Source
//------------------------------------------------------------------------------

namespace search_mode
{
int32_t const PREFER_PAST    = -1;
int32_t const PREFER_NEAREST =  0;
int32_t const PREFER_FUTURE  =  1;
};

class source
{
private:
    hl2ss::mt::ring_buffer<std::shared_ptr<hl2ss::packet>> m_buffer;
    std::unique_ptr<hl2ss::rx> m_rx;
    std::atomic<bool> m_enable;
    std::exception m_error;
    std::thread m_thread;    
    std::shared_mutex m_mutex;
    
    void capture();
    void run();

public:
    source(uint64_t buffer_size, std::unique_ptr<hl2ss::rx> rx);
    ~source();

    void start();
    bool status(std::exception& error);
    void stop();
    
    template <typename T>
    T const* get_rx()
    {
        return dynamic_cast<T*>(m_rx.get());
    }

    std::shared_ptr<hl2ss::packet> get_packet(int64_t& frame_index, int32_t& status);
    std::shared_ptr<hl2ss::packet> get_packet(uint64_t timestamp, int32_t mode, int64_t& frame_index, int32_t& status);
};
}
}
