
#include <chrono>
#include <thread>
#include "hl2ss_dp.h"

namespace hl2ss
{
namespace dp
{
//------------------------------------------------------------------------------
// * Client
//------------------------------------------------------------------------------

char const* bool_to_str(bool v)
{
    return v ? "true" : "false";
}

bool client::on_write(std::string_view const& data, intptr_t userdata)
{
    auto size = data.end() - data.begin();

    if (size > 0)
    {
    chunk_descriptor cd{new uint8_t[size], (uint32_t)size};
    memcpy(cd.data, data.data(), size);
    {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_buffer.push(cd);
    }
    }
    
    return !m_response[0].IsCancelled();
}

void client::open(char const* host, char const* port, char const* user, char const* password, mrc_configuration const& configuration)
{
    auto url = cpr::Url{"https://" + std::string(host) + "/api/holographic/stream/" + std::string(port) + ".mp4"};
    auto cb = cpr::WriteCallback([=](std::string_view const& data, intptr_t userdata){ return this->on_write(data, userdata); });
    auto auth = cpr::Authentication{user, password, cpr::AuthMode::BASIC};
    auto verify = cpr::Ssl(cpr::ssl::VerifyHost{false}, cpr::ssl::VerifyPeer{false}, cpr::ssl::VerifyStatus{false});
    auto params = cpr::Parameters{
        {"holo",             bool_to_str(configuration.holo)},
        {"pv",               bool_to_str(configuration.pv)},
        {"mic",              bool_to_str(configuration.mic)},
        {"loopback",         bool_to_str(configuration.loopback)},
        {"RenderFromCamera", bool_to_str(configuration.RenderFromCamera)},
        {"vstab",            bool_to_str(configuration.vstab)},
        {"vstabbuffer",      std::to_string(configuration.vstabbuffer)}
    };
    
    m_read = 0;
    m_response.emplace_back(cpr::GetAsync(url, params, auth, verify, cb));
}

uint64_t client::recv(void* buffer, uint64_t count)
{
    uint8_t* base      = (uint8_t*)buffer;
    uint64_t remaining = count;

    while (remaining > 0)
    {
    chunk_descriptor cd;

    {
    std::lock_guard<std::mutex> guard(m_mutex);
    if (m_buffer.size() <= 0) { break; }
    cd = m_buffer.front();
    }

    uint64_t available = cd.size - m_read;
    uint64_t read = available > remaining ? remaining : available;

    memcpy(base, cd.data + m_read, read);

    base      += read;
    m_read    += read;
    remaining -= read;

    if ((available - read) > 0) { continue; }

    m_read = 0;
    delete[] cd.data;

    {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_buffer.pop();
    }
    }

    return count - remaining;
}

void client::download(void* buffer, uint64_t total, uint64_t chunk)
{
    uint64_t received = 0;
    while (total > 0)
    {
    if (chunk > total) { chunk = total; }
    uint64_t bytes = recv((uint8_t*)buffer + received, chunk);
    total    -= bytes;
    received += bytes;
    if (bytes == 0) { std::this_thread::sleep_for(std::chrono::milliseconds(16)); }
    }
}

void client::close()
{
    if (m_response[0].Cancel() != cpr::CancellationResult::success)
    {
    throw std::runtime_error("hl2ss::dp::client::close : Cancel failed");
    }
    m_response.clear();
    {
    std::lock_guard<std::mutex> guard(m_mutex);
    while (!m_buffer.empty())
    {
    delete[] m_buffer.front().data;
    m_buffer.pop();
    }
    }
}

//------------------------------------------------------------------------------
// * Unpacker
//------------------------------------------------------------------------------

struct box
{
    char type[4];
    uint32_t size;
    uint8_t* data;
}



}
}
