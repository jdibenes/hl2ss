
#include <shared_mutex>
#include <vector>
#include "hl2ss.h"

namespace hl2ss
{
namespace mt
{
template <typename T>
class ring_buffer
{
private:
    static int64_t const TOO_OLD = -2;
    static int64_t const TOO_NEW = -1;

    int64_t m_frame_stamp;
    int64_t m_write_index;
    std::vector<T> m_data;

    int64_t to_index(int64_t frame_stamp)
    {
        if (frame_stamp < 0) { return TOO_OLD; }
        int64_t d = m_frame_stamp - frame_stamp;
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
        m_frame_stamp = -1;
        m_write_index = -1;
    }

    void insert(T const& data)
    {
        m_write_index = (m_write_index + 1) % m_data.size();
        m_frame_stamp++;
        m_data[m_write_index] = data;
    }

    T get(int64_t& frame_stamp, int32_t& status)
    {
        if (frame_stamp < 0) { frame_stamp += (m_frame_stamp + 1); }
        int64_t index = to_index(frame_stamp);
        if      (index == TOO_NEW) { status =  1; return T(); }
        else if (index == TOO_OLD) { status = -1; return T(); }
        else                       { status =  0; return m_data[index]; }
    }

    int64_t stamp_begin()
    {
        return std::max(0, m_frame_stamp - m_data.size());
    }

    int64_t stamp_end()
    {
        return m_frame_stamp;
    }
};

class source
{
private:
    hl2ss::mt::ring_buffer<std::shared_ptr<hl2ss::packet>> m_buffer;
    std::unique_ptr<hl2ss::rx> m_rx;
    std::atomic<bool> m_enable;
    std::thread m_thread;    
    std::shared_mutex m_mutex;

    void capture();
    void run();

public:
    source(uint64_t buffer_size, std::unique_ptr<hl2ss::rx> rx);
    ~source();

    void start();
    void stop();

    hl2ss::rx const* get_rx();
    std::shared_ptr<hl2ss::packet> source::get_packet(int64_t& frame_stamp, int32_t& status);
};
}
}
