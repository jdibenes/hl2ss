
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
    struct search_interval
    {
        int64_t l;
        int64_t r;
    };

    int64_t m_size;

    std::vector<T> m_frames;
    std::vector<uint64_t> m_timestamps;

    int64_t m_write;
    int64_t m_count;

    search_interval search(uint64_t timestamp, int64_t l, int64_t r, int64_t size) const
    {
        while ((r - l) > 1)
        {
        int64_t m = (r + l) / 2;
        int64_t m_index = m % size;
        uint64_t t = m_timestamps[m_index];
        if (t > timestamp) { r = m; } else if (t < timestamp) { l = m; } else { return { m_index, m_index }; }
        }

        return { l % size, r % size };
    }

    int64_t find_index(uint64_t timestamp, int32_t time_preference, bool tiebreak_right, int64_t size) const
    {
        if (size <= 0) { return -1; }

        search_interval si = search(timestamp, m_write, m_write + size - 1, size);

        if (si.l == si.r) { return si.l; }

        uint64_t tl = m_timestamps[si.l];
        uint64_t tr = m_timestamps[si.r];

        if (timestamp <= tl) { return si.l; }
        if (timestamp >= tr) { return si.r; }

        if (time_preference < 0) { return si.l; }
        if (time_preference > 0) { return si.r; }

        uint64_t dl = timestamp - tl;
        uint64_t dr = tr - timestamp;

        if (dl < dr) { return si.l; }
        if (dl > dr) { return si.r; }

        return tiebreak_right ? si.r : si.l;
    }

public:
    ring_buffer(int64_t size)
    {
        reset(size);
    }

    ring_buffer() : ring_buffer(16)
    {

    }

    void reset()
    {
        m_frames.clear();
        m_timestamps.clear();

        m_frames.resize(m_size);
        m_timestamps.resize(m_size);

        m_write = 0;
        m_count = 0;
    }

    void reset(int64_t size)
    {
        m_size = (size <= 0) ? 1 : size;
        reset();
    }

    int64_t size() const
    {
        return (m_count >= m_size) ? m_size : m_count;
    }

    T insert(T const& in, uint64_t timestamp)
    {
        T old = m_frames[m_write];

        m_frames[m_write] = in;
        m_timestamps[m_write] = timestamp;

        m_write = (m_write + 1) % m_size;
        m_count = (m_count + 1);

        return old;
    }

    int get(int64_t stamp, T& out, uint64_t& t, int64_t& s) const
    {
        int64_t base = size();
        int64_t index = (stamp < 0) ? (base + stamp) : (base + stamp - m_count);

        if (index < 0) { return -1; }
        if (index >= base) { return 1; }

        int64_t slot = (m_write + index) % base;

        out = m_frames[slot];
        t = m_timestamps[slot];
        s = index + m_count - base;
        return 0;
    }

    int get(uint64_t timestamp, int32_t time_preference, bool tiebreak_right, T& out, uint64_t& t, int64_t& s) const
    {
        int64_t base = size();
        int64_t index = find_index(timestamp, time_preference, tiebreak_right, base);

        if (index < 0) { return 1; }

        out = m_frames[index];
        t = m_timestamps[index];
        s = m_count - base + ((base + index - m_write) % base);
        return 0;
    }

    T at(int32_t index) const
    {
        return m_frames[index];
    }
};

//------------------------------------------------------------------------------
// Source
//------------------------------------------------------------------------------

namespace time_preference
{
int32_t const PREFER_PAST    = -1;
int32_t const PREFER_NEAREST =  0;
int32_t const PREFER_FUTURE  =  1;
}

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
    source(int64_t buffer_size, std::unique_ptr<hl2ss::rx> rx);
    ~source();

    void start();
    bool status(std::exception& error);
    void stop();
    
    template <typename T>
    T const* get_rx()
    {
        return dynamic_cast<T*>(m_rx.get());
    }

    std::shared_ptr<hl2ss::packet> get_packet(int64_t& frame_stamp, int32_t& status);
    std::shared_ptr<hl2ss::packet> get_packet(uint64_t timestamp, int32_t time_preference, bool tiebreak_right, int64_t& frame_stamp, int32_t& status);
};
}
}
