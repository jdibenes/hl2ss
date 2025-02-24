
#include <cpr/cpr.h>
#include <queue>
#include <mutex>

namespace hl2ss
{
namespace dp
{
struct mrc_configuration
{
    bool holo;
    bool pv;
    bool mic;
    bool loopback;
    bool RenderFromCamera;
    bool vstab;
    int vstabbuffer;
};

struct chunk_descriptor
{
    uint8_t* data;
    uint64_t size;
};

class client
{
private:
    std::vector<cpr::AsyncResponse> m_response;
    std::queue<chunk_descriptor> m_buffer;
    std::mutex m_mutex;
    uint64_t m_read;

    bool on_write(std::string_view const& data, intptr_t userdata);

public:
    void open(char const* host, char const* port, char const* user, char const* password, mrc_configuration const& configuration);
    uint64_t recv(void* buffer, uint64_t count);
    void download(void* buffer, uint64_t total, uint64_t chunk);
    void close();
};
}
}
