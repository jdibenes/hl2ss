
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"

namespace hl2ss
{
namespace matlab
{
namespace grab_mode
{
uint8_t const BY_FRAME_INDEX = 0;
uint8_t const BY_TIMESTAMP   = 1;
}
}
}

typedef std::vector<uint64_t> options_t;

//------------------------------------------------------------------------------
// (*) MexFunction
//------------------------------------------------------------------------------

class MexFunction : public matlab::mex::Function
{
private:
    std::shared_ptr<matlab::engine::MATLABEngine> m_matlabPtr = getEngine();
    matlab::data::ArrayFactory m_factory;
    uint32_t m_argument_index;
    bool m_initialized;

    std::unique_ptr<hl2ss::mt::source> source_rm_vlc[4];
    std::unique_ptr<hl2ss::mt::source> source_rm_depth_ahat;
    std::unique_ptr<hl2ss::mt::source> source_rm_depth_longthrow;
    std::unique_ptr<hl2ss::mt::source> source_rm_imu[3];
    std::unique_ptr<hl2ss::mt::source> source_pv;
    std::unique_ptr<hl2ss::mt::source> source_microphone;
    std::unique_ptr<hl2ss::mt::source> source_si;
    std::unique_ptr<hl2ss::mt::source> source_eet;
    std::unique_ptr<hl2ss::mt::source> source_extended_audio;
    std::unique_ptr<hl2ss::ipc_rc>     ipc_rc;
    std::unique_ptr<hl2ss::ipc_sm>     ipc_sm;
    std::unique_ptr<hl2ss::ipc_su>     ipc_su;
    std::unique_ptr<hl2ss::ipc_vi>     ipc_vi;
    std::unique_ptr<hl2ss::ipc_umq>    ipc_umq;

public:
    //------------------------------------------------------------------------------
    // (*) Helpers
    //------------------------------------------------------------------------------

    static void default_deleter(void* p)
    {
        delete[] p;
    }

    template <typename T>
    matlab::data::TypedArray<T> get_argument_array(matlab::mex::ArgumentList inputs)
    {
        if (m_argument_index >= inputs.size()) { throw std::runtime_error("Not enough inputs"); }
        return inputs[m_argument_index++];
    }

    template <typename T>
    T get_argument(matlab::mex::ArgumentList inputs)
    {
        return get_argument_array<T>(inputs)[0];
    }

    std::string get_argument_string(matlab::mex::ArgumentList inputs)
    {
        matlab::data::CharArray argument = get_argument_array<CHAR16_T>(inputs);
        return argument.toAscii();
    }

    template <typename T>
    std::vector<T> to_std_vector(matlab::data::TypedArray<T> t)
    {
        return std::vector<T>(t.begin(), t.end());
    }

    template <typename T>
    T* get_pointer(matlab::data::TypedArray<T>& array)
    {
        return array.begin().operator->();
    }

    template <typename T, size_t size>
    matlab::data::TypedArray<T> to_typed_array(T const (&array)[size], matlab::data::ArrayDimensions dims)
    {
        return m_factory.createArray<T>(dims, &array[0], &array[size]);
    }

    template <typename T>
    matlab::data::TypedArray<T> to_typed_array(void const* data, size_t size, matlab::data::ArrayDimensions dims)
    {
        return m_factory.createArray<T>(dims, (T*)data, (T*)(((uint8_t*)data) + size));
    }

    template <typename T>
    T get_field_scalar(matlab::data::TypedArray<T> array)
    {
        return array[0];
    }

    hl2ss::vector_2 get_field_vector_2(matlab::data::TypedArray<float> array)
    {
        return { array[0], array[1] };
    }

    hl2ss::vector_3 get_field_vector_3(matlab::data::TypedArray<float> array)
    {
        return { array[0], array[1], array[2] };
    }

    hl2ss::vector_4 get_field_vector_4(matlab::data::TypedArray<float> array)
    {
        return { array[0], array[1], array[2], array[3] };
    }

    hl2ss::guid get_field_guid(matlab::data::TypedArray<uint64_t> array)
    {
        return { array[0], array[1] };
    }

    void error(char const* message)
    {
        m_matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ m_factory.createScalar(message) });
    }

    //------------------------------------------------------------------------------
    // (*) Open
    //------------------------------------------------------------------------------

    void open_rm_vlc(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t  chunk       =               get_argument<uint64_t>(inputs);
        uint8_t   mode        =               get_argument<uint8_t>(inputs);
        uint8_t   divisor     =               get_argument<uint8_t>(inputs);
        uint8_t   profile     =               get_argument<uint8_t>(inputs);
        uint8_t   level       =               get_argument<uint8_t>(inputs);
        uint32_t  bitrate     =               get_argument<uint32_t>(inputs);
        options_t options     = to_std_vector(get_argument_array<uint64_t>(inputs));
        bool      decoded     =               true;
        uint64_t  buffer_size =               get_argument<uint64_t>(inputs);

        (source_rm_vlc[port - hl2ss::stream_port::RM_VLC_LEFTFRONT] = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, &options, decoded)))->start();
    }

    void open_rm_depth_ahat(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t  chunk       =               get_argument<uint64_t>(inputs);
        uint8_t   mode        =               get_argument<uint8_t>(inputs);
        uint8_t   divisor     =               get_argument<uint8_t>(inputs);
        uint8_t   profile_z   =               get_argument<uint8_t>(inputs);
        uint8_t   profile_ab  =               get_argument<uint8_t>(inputs);
        uint8_t   level       =               get_argument<uint8_t>(inputs);
        uint32_t  bitrate     =               get_argument<uint32_t>(inputs);
        options_t options     = to_std_vector(get_argument_array<uint64_t>(inputs));
        bool      decoded     =               true;
        uint64_t  buffer_size =               get_argument<uint64_t>(inputs);

        (source_rm_depth_ahat = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, &options, decoded)))->start();
    }

    void open_rm_depth_longthrow(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t chunk       = get_argument<uint64_t>(inputs);
        uint8_t  mode        = get_argument<uint8_t>(inputs);
        uint8_t  divisor     = get_argument<uint8_t>(inputs);
        uint8_t  png_filter  = get_argument<uint8_t>(inputs);
        bool     decoded     = true;
        uint64_t buffer_size = get_argument<uint64_t>(inputs);

        (source_rm_depth_longthrow = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter, decoded)))->start();
    }

    void open_rm_imu(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t chunk       = get_argument<uint64_t>(inputs);
        uint8_t  mode        = get_argument<uint8_t>(inputs);
        uint64_t buffer_size = get_argument<uint64_t>(inputs);
        
        (source_rm_imu[port - hl2ss::stream_port::RM_IMU_ACCELEROMETER] = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_imu(host, port, chunk, mode)))->start();
    }

    void open_pv(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint16_t  width          =               get_argument<uint16_t>(inputs);
        uint16_t  height         =               get_argument<uint16_t>(inputs);
        uint8_t   framerate      =               get_argument<uint8_t>(inputs);
        uint64_t  chunk          =               get_argument<uint64_t>(inputs);
        uint8_t   mode           =               get_argument<uint8_t>(inputs);
        uint8_t   divisor        =               get_argument<uint8_t>(inputs);
        uint8_t   profile        =               get_argument<uint8_t>(inputs);
        uint8_t   level          =               get_argument<uint8_t>(inputs);
        uint32_t  bitrate        =               get_argument<uint32_t>(inputs);
        options_t options        = to_std_vector(get_argument_array<uint64_t>(inputs));
        uint8_t   decoded_format =               get_argument<uint8_t>(inputs) % 5;
        uint64_t  buffer_size    =               get_argument<uint64_t>(inputs);

        (source_pv = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_pv(host, port, width, height, framerate, chunk, mode, divisor, profile, level, bitrate, &options, decoded_format)))->start();
    }

    void open_microphone(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t chunk       = get_argument<uint64_t>(inputs);
        uint8_t  profile     = get_argument<uint8_t>(inputs);
        uint8_t  level       = get_argument<uint8_t>(inputs);
        bool     decoded     = true;
        uint64_t buffer_size = get_argument<uint64_t>(inputs);
        
        (source_microphone = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_microphone(host, port, chunk, profile, level, decoded)))->start();
    }

    void open_si(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t chunk       = get_argument<uint64_t>(inputs);
        uint64_t buffer_size = get_argument<uint64_t>(inputs);

        (source_si = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_si(host, port, chunk)))->start();
    }

    void open_eet(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t chunk       = get_argument<uint64_t>(inputs);
        uint8_t  framerate   = get_argument<uint8_t>(inputs);
        uint64_t buffer_size = get_argument<uint64_t>(inputs);

        (source_eet = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_eet(host, port, chunk, framerate)))->start();
    }

    void open_extended_audio(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint64_t chunk           = get_argument<uint64_t>(inputs);
        uint32_t mixer_mode      = get_argument<uint32_t>(inputs);
        float    loopback_gain   = get_argument<float>(inputs);
        float    microphone_gain = get_argument<float>(inputs);
        uint8_t  profile         = get_argument<uint8_t>(inputs);
        uint8_t  level           = get_argument<uint8_t>(inputs);
        bool     decoded         = true;
        uint64_t buffer_size     = get_argument<uint64_t>(inputs);

        (source_extended_audio = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_extended_audio(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level, decoded)))->start();
    }

    void open(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host = get_argument_string(inputs);
        uint16_t    port = get_argument<uint16_t>(inputs);
        
        switch (port)
        {
        // Stream
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    open_rm_vlc(            host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        open_rm_depth_ahat(     host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   open_rm_depth_longthrow(host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  open_rm_imu(            host.c_str(), port, inputs); break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       open_pv(                host.c_str(), port, inputs); break;
        case hl2ss::stream_port::MICROPHONE:           open_microphone(        host.c_str(), port, inputs); break;
        case hl2ss::stream_port::SPATIAL_INPUT:        open_si(                host.c_str(), port, inputs); break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: open_eet(               host.c_str(), port, inputs); break;
        case hl2ss::stream_port::EXTENDED_AUDIO:       open_extended_audio(    host.c_str(), port, inputs); break;
        // IPC
        case hl2ss::ipc_port::REMOTE_CONFIGURATION:    (ipc_rc  = hl2ss::lnm::ipc_rc( host.c_str(), port))->open(); break;
        case hl2ss::ipc_port::SPATIAL_MAPPING:         (ipc_sm  = hl2ss::lnm::ipc_sm( host.c_str(), port))->open(); break;
        case hl2ss::ipc_port::SCENE_UNDERSTANDING:     (ipc_su  = hl2ss::lnm::ipc_su( host.c_str(), port))->open(); break;
        case hl2ss::ipc_port::VOICE_INPUT:             (ipc_vi  = hl2ss::lnm::ipc_vi( host.c_str(), port))->open(); break;
        case hl2ss::ipc_port::UNITY_MESSAGE_QUEUE:     (ipc_umq = hl2ss::lnm::ipc_umq(host.c_str(), port))->open(); break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    //------------------------------------------------------------------------------
    // (*) Close
    //------------------------------------------------------------------------------

    void close(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint16_t port = get_argument<uint16_t>(inputs);

        switch (port)
        {
        // Stream
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:     
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:      
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    source_rm_vlc[port - hl2ss::stream_port::RM_VLC_LEFTFRONT     ] = nullptr; break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        source_rm_depth_ahat                                            = nullptr; break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   source_rm_depth_longthrow                                       = nullptr; break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: 
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:     
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  source_rm_imu[port - hl2ss::stream_port::RM_IMU_ACCELEROMETER]  = nullptr; break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       source_pv                                                       = nullptr; break;
        case hl2ss::stream_port::MICROPHONE:           source_microphone                                               = nullptr; break;
        case hl2ss::stream_port::SPATIAL_INPUT:        source_si                                                       = nullptr; break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: source_eet                                                      = nullptr; break;
        case hl2ss::stream_port::EXTENDED_AUDIO:       source_extended_audio                                           = nullptr; break;
        // IPC
        case hl2ss::ipc_port::REMOTE_CONFIGURATION:    ipc_rc  = nullptr; break;
        case hl2ss::ipc_port::SPATIAL_MAPPING:         ipc_sm  = nullptr; break;
        case hl2ss::ipc_port::SCENE_UNDERSTANDING:     ipc_su  = nullptr; break;
        case hl2ss::ipc_port::VOICE_INPUT:             ipc_vi  = nullptr; break;
        case hl2ss::ipc_port::UNITY_MESSAGE_QUEUE:     ipc_umq = nullptr; break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    //------------------------------------------------------------------------------
    // (*) Grab
    //------------------------------------------------------------------------------

    template <typename T>
    matlab::data::Array unpack_payload(void const* payload, uint32_t offset, uint32_t size, matlab::data::ArrayDimensions dims)
    {
        std::unique_ptr<T[]> payload_copy = std::make_unique<T[]>(size / sizeof(T));
        memcpy(payload_copy.get(), ((uint8_t*)payload) + offset, size);
        return m_factory.createArrayFromBuffer<T>(dims, matlab::data::buffer_ptr_t<T>(payload_copy.release(), default_deleter), matlab::data::MemoryLayout::ROW_MAJOR);
    }

    matlab::data::Array unpack_pose(hl2ss::matrix_4x4 const* pose)
    {
        std::unique_ptr<float[]> pose_copy = std::make_unique<float[]>(hl2ss::packet::SZ_POSE / sizeof(float));
        memcpy(pose_copy.get(), pose, hl2ss::packet::SZ_POSE);
        return m_factory.createArrayFromBuffer<float>({4, 4}, matlab::data::buffer_ptr_t<float>(pose_copy.release(), default_deleter), matlab::data::MemoryLayout::ROW_MAJOR);
    }

    void pack_rm_vlc(int64_t frame_index, int32_t status, hl2ss::packet* packet, matlab::mex::ArgumentList outputs)
    {
        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "image", "pose" });

        o[0]["frame_index"] = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]      = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]   = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        o[0]["image"]       = unpack_payload<uint8_t>(packet->payload.get(), 0, packet->sz_payload, { hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH });
        }
        if (packet->pose)
        {
        o[0]["pose"]        = unpack_pose(packet->pose.get());
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_rm_depth_ahat(int64_t frame_index, int32_t status, hl2ss::packet* packet, matlab::mex::ArgumentList outputs)
    {
        uint32_t size = hl2ss::parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "depth", "ab", "pose" });
        
        o[0]["frame_index"] = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]      = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]   = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        o[0]["depth"]       = unpack_payload<uint16_t>(packet->payload.get(),    0, size, { hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH });
        o[0]["ab"]          = unpack_payload<uint16_t>(packet->payload.get(), size, size, { hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH });
        }
        if (packet->pose)
        {
        o[0]["pose"]        = unpack_pose(packet->pose.get());
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_rm_depth_longthrow(int64_t frame_index, int32_t status, hl2ss::packet* packet, matlab::mex::ArgumentList outputs)
    {
        uint32_t size = hl2ss::parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "depth", "ab", "pose" });
        
        o[0]["frame_index"] = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]      = m_factory.createScalar<int32_t>(status);
        if (packet)
        {
        o[0]["timestamp"]   = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        o[0]["depth"]       = unpack_payload<uint16_t>(packet->payload.get(),    0, size, { hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH });
        o[0]["ab"]          = unpack_payload<uint16_t>(packet->payload.get(), size, size, { hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH });
        }
        if (packet->pose)
        {
        o[0]["pose"]        = unpack_pose(packet->pose.get());
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_rm_imu(int64_t frame_index, int32_t status, hl2ss::packet* packet, matlab::mex::ArgumentList outputs)
    {
        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "sensor_timestamp", "host_timestamp", "x", "y", "z", "temperature", "pose" });

        o[0]["frame_index"]      = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]           = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]        = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        uint32_t samples = packet->sz_payload / sizeof(hl2ss::rm_imu_sample);

        matlab::data::TypedArray<uint64_t> sensor_timestamp = m_factory.createArray<uint64_t>({ samples });
        matlab::data::TypedArray<uint64_t> host_timestamp   = m_factory.createArray<uint64_t>({ samples });
        matlab::data::TypedArray<float>    x                = m_factory.createArray<float>({ samples });
        matlab::data::TypedArray<float>    y                = m_factory.createArray<float>({ samples });
        matlab::data::TypedArray<float>    z                = m_factory.createArray<float>({ samples });
        matlab::data::TypedArray<float>    temperature      = m_factory.createArray<float>({ samples });

        hl2ss::rm_imu_sample* base;
        hl2ss::unpack_rm_imu(packet->payload.get(), &base);

        for (uint32_t i = 0; i < samples; ++i)
        {
        sensor_timestamp[i] = base[i].sensor_timestamp;
        host_timestamp[i]   = base[i].timestamp;
        x[i]                = base[i].x;
        y[i]                = base[i].y;
        z[i]                = base[i].z;
        temperature[i]      = base[i].temperature;
        }

        o[0]["sensor_timestamp"] = std::move(sensor_timestamp);
        o[0]["host_timestamp"]   = std::move(host_timestamp);
        o[0]["x"]                = std::move(x);
        o[0]["y"]                = std::move(y);
        o[0]["z"]                = std::move(z);
        o[0]["temperature"]      = std::move(temperature);
        }
        if (packet->pose)
        {
        o[0]["pose"]             = unpack_pose(packet->pose.get());
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_pv(int64_t frame_index, int32_t status, hl2ss::packet* packet, uint16_t width, uint16_t height, uint8_t channels, matlab::mex::ArgumentList outputs)
    {
        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "image", "intrinsics", "pose" });

        o[0]["frame_index"] = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]      = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]   = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        uint32_t image_size = packet->sz_payload - hl2ss::decoder_pv::K_SIZE;
        o[0]["image"]       = unpack_payload<uint8_t>(packet->payload.get(), 0,                         image_size, { height, width, channels });
        o[0]["intrinsics"]  = unpack_payload<float>(  packet->payload.get(), image_size, hl2ss::decoder_pv::K_SIZE, { 4 });
        }
        if (packet->pose)
        {
        o[0]["pose"]        = unpack_pose(packet->pose.get());
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_microphone(int64_t frame_index, int32_t status, hl2ss::packet* packet, uint8_t profile, uint8_t level, matlab::mex::ArgumentList outputs)
    {
        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "audio" });

        o[0]["frame_index"] = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]      = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]   = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        if (profile != hl2ss::audio_profile::RAW)
        {
        o[0]["audio"]       = unpack_payload<float>(  packet->payload.get(), 0, packet->sz_payload, { hl2ss::parameters_microphone::CHANNELS,       packet->sz_payload / (sizeof(float)   * hl2ss::parameters_microphone::CHANNELS) });
        }
        else if (level == hl2ss::aac_level::L5)
        {
        o[0]["audio"]       = to_typed_array<float>(  packet->payload.get(),    packet->sz_payload, { hl2ss::parameters_microphone::ARRAY_CHANNELS, packet->sz_payload / (sizeof(float)   * hl2ss::parameters_microphone::ARRAY_CHANNELS) });
        }
        else
        {
        o[0]["audio"]       = to_typed_array<int16_t>(packet->payload.get(),    packet->sz_payload, { hl2ss::parameters_microphone::CHANNELS,       packet->sz_payload / (sizeof(int16_t) * hl2ss::parameters_microphone::CHANNELS) });
        }
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_si(int64_t frame_index, int32_t status, hl2ss::packet* packet, matlab::mex::ArgumentList outputs)
    {
        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "valid", "head_pose", "eye_ray", "left_hand", "right_hand" });

        o[0]["frame_index"] = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]      = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]   = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        hl2ss::si_frame* base;
        hl2ss::unpack_si(packet->payload.get(), &base);

        o[0]["valid"]       = m_factory.createScalar<uint32_t>({ base->valid });
        o[0]["head_pose"]   = to_typed_array<float>(&base->head_pose,  sizeof(base->head_pose),  { 3, 3 });
        o[0]["eye_ray"]     = to_typed_array<float>(&base->eye_ray,    sizeof(base->eye_ray),    { 6 });
        o[0]["left_hand"]   = to_typed_array<float>(&base->left_hand,  sizeof(base->left_hand),  { 9, 26 });
        o[0]["right_hand"]  = to_typed_array<float>(&base->right_hand, sizeof(base->right_hand), { 9, 26 });
        } 
        }

        outputs[0] = std::move(o);
    }

    void pack_eet(int64_t frame_index, int32_t status, hl2ss::packet* packet, matlab::mex::ArgumentList outputs)
    {
        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "combined_ray", "left_ray", "right_ray", "left_openness", "right_openness", "vergence_distance", "valid", "pose" });

        o[0]["frame_index"]       = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]            = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]         = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        hl2ss::eet_frame* base;
        hl2ss::unpack_eet(packet->payload.get(), &base);

        o[0]["combined_ray"]      = to_typed_array<float>(&base->combined_ray, sizeof(base->combined_ray), { 6 });
        o[0]["left_ray"]          = to_typed_array<float>(&base->left_ray,     sizeof(base->left_ray),     { 6 });
        o[0]["right_ray"]         = to_typed_array<float>(&base->right_ray,    sizeof(base->right_ray),    { 6 });
        o[0]["left_openness"]     = m_factory.createScalar<float>(base->left_openness);
        o[0]["right_openness"]    = m_factory.createScalar<float>(base->right_openness);
        o[0]["vergence_distance"] = m_factory.createScalar<float>(base->vergence_distance);
        o[0]["valid"]             = m_factory.createScalar<uint32_t>(base->valid);
        }
        if (packet->pose)
        {
        o[0]["pose"]              = unpack_pose(packet->pose.get());
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_extended_audio(int64_t frame_index, int32_t status, hl2ss::packet* packet, uint8_t profile, matlab::mex::ArgumentList outputs)
    {
        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "frame_index", "status", "timestamp", "audio" });

        o[0]["frame_index"] = m_factory.createScalar<int64_t>(frame_index);
        o[0]["status"]      = m_factory.createScalar<int32_t>(status);

        if (packet)
        {
        o[0]["timestamp"]   = m_factory.createScalar<uint64_t>(packet->timestamp);
        if (packet->payload)
        {
        if (profile != hl2ss::audio_profile::RAW)
        {
        o[0]["audio"]       = unpack_payload<float>(  packet->payload.get(), 0, packet->sz_payload, { hl2ss::parameters_extended_audio::CHANNELS, packet->sz_payload / (sizeof(float) * hl2ss::parameters_extended_audio::CHANNELS) });
        }
        else
        {
        o[0]["audio"]       = to_typed_array<int16_t>(packet->payload.get(),    packet->sz_payload, { hl2ss::parameters_extended_audio::CHANNELS, packet->sz_payload / (sizeof(int16_t) * hl2ss::parameters_extended_audio::CHANNELS) });
        }
        }
        }

        outputs[0] = std::move(o);
    }

    std::shared_ptr<hl2ss::packet> grab(hl2ss::mt::source* source, int64_t&frame_index, int32_t& status, matlab::mex::ArgumentList inputs)
    {
        if (!source) { throw std::runtime_error("Port not open"); }

        std::exception source_error;
        if (!source->status(source_error)) { throw source_error; }

        uint8_t type = get_argument<uint8_t>(inputs);
        if (type == hl2ss::matlab::grab_mode::BY_FRAME_INDEX)
        {
        frame_index = get_argument<int64_t>(inputs);
        return source->get_packet(frame_index, status);
        }
        else if (type == hl2ss::matlab::grab_mode::BY_TIMESTAMP)
        {
        uint64_t timestamp = get_argument<uint64_t>(inputs);
        int32_t  mode      = get_argument<int32_t>(inputs);
        return source->get_packet(timestamp, mode, frame_index, status);
        }
        else
        {
        throw std::runtime_error("Unsupported grab type");
        }
    }

    void get_packet_rm_vlc(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        hl2ss::mt::source* source = source_rm_vlc[port - hl2ss::stream_port::RM_VLC_LEFTFRONT].get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);

        pack_rm_vlc(frame_index, status, packet.get(), outputs);
    }

    void get_packet_rm_depth_ahat(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_rm_depth_ahat.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);

        pack_rm_depth_ahat(frame_index, status, packet.get(), outputs);
    }

    void get_packet_rm_depth_longthrow(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_rm_depth_longthrow.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);

        pack_rm_depth_longthrow(frame_index, status, packet.get(), outputs);
    }

    void get_packet_rm_imu(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        hl2ss::mt::source* source = source_rm_imu[port - hl2ss::stream_port::RM_IMU_ACCELEROMETER].get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);

        pack_rm_imu(frame_index, status, packet.get(), outputs);
    }

    void get_packet_pv(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_pv.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);

        uint16_t width;
        uint16_t height;
        uint8_t channels;

        if (packet)
        {
        hl2ss::rx_decoded_pv const* p_rx = source->get_rx<hl2ss::rx_decoded_pv>();
        hl2ss::decoder_pv::resolution_decoded(packet->sz_payload, p_rx->decoded_format, width, height, channels);
        }
        else
        {
        width = height = channels = 0;
        }

        pack_pv(frame_index, status, packet.get(), width, height, channels, outputs);
    }

    void get_packet_microphone(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_microphone.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);
        hl2ss::rx_microphone const* p_rx = source->get_rx<hl2ss::rx_microphone>();

        pack_microphone(frame_index, status, packet.get(), p_rx->profile, p_rx->level, outputs);
    }

    void get_packet_si(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_si.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);

        pack_si(frame_index, status, packet.get(), outputs);
    }

    void get_packet_eet(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_eet.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);

        pack_eet(frame_index, status, packet.get(), outputs);
    }

    void get_packet_extended_audio(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_extended_audio.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);
        hl2ss::rx_extended_audio const* p_rx = source->get_rx<hl2ss::rx_extended_audio>();

        pack_extended_audio(frame_index, status, packet.get(), p_rx->profile, outputs);        
    }

    void get_packet(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint16_t port = get_argument<uint16_t>(inputs);

        switch (port)
        {
        // Stream
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:     
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:      
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    get_packet_rm_vlc(            port, outputs, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        get_packet_rm_depth_ahat(     port, outputs, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   get_packet_rm_depth_longthrow(port, outputs, inputs); break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: 
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:      
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  get_packet_rm_imu(            port, outputs, inputs); break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       get_packet_pv(                port, outputs, inputs); break;               
        case hl2ss::stream_port::MICROPHONE:           get_packet_microphone(        port, outputs, inputs); break;
        case hl2ss::stream_port::SPATIAL_INPUT:        get_packet_si(                port, outputs, inputs); break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: get_packet_eet(               port, outputs, inputs); break;
        case hl2ss::stream_port::EXTENDED_AUDIO:       get_packet_extended_audio(    port, outputs, inputs); break;
        default:                                       throw std::runtime_error("Unsupported port");
        }
    }

    //------------------------------------------------------------------------------
    // (*) Control
    //------------------------------------------------------------------------------

    void start_subsystem_pv(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (source_pv) { throw std::runtime_error("Cannot start subsystem while streaming"); }

        std::string host                       = get_argument_string(inputs);
        uint16_t    port                       = get_argument<uint16_t>(inputs);
        bool        enable_mrc                 = get_argument<bool>(inputs);
        bool        hologram_composition       = get_argument<bool>(inputs);
        bool        recording_indicator        = get_argument<bool>(inputs);
        bool        video_stabilization        = get_argument<bool>(inputs);
        bool        blank_protected            = get_argument<bool>(inputs);
        bool        show_mesh                  = get_argument<bool>(inputs);
        bool        shared                     = get_argument<bool>(inputs);
        float       global_opacity             = get_argument<float>(inputs);
        float       output_width               = get_argument<float>(inputs);
        float       output_height              = get_argument<float>(inputs);
        uint32_t    video_stabilization_length = get_argument<uint32_t>(inputs);
        uint32_t    hologram_perspective       = get_argument<uint32_t>(inputs);

        hl2ss::lnm::start_subsystem_pv(host.c_str(), port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective);
    }

    void stop_subsystem_pv(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (source_pv) { throw std::runtime_error("Cannot stop subsystem while streaming"); }

        std::string host = get_argument_string(inputs);
        uint16_t    port = get_argument<uint16_t>(inputs);

        hl2ss::lnm::stop_subsystem_pv(host.c_str(), port);
    }

    //------------------------------------------------------------------------------
    // (*) Calibration
    //------------------------------------------------------------------------------

    void download_calibration_rm_vlc(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_vlc[port - hl2ss::stream_port::RM_VLC_LEFTFRONT]) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_vlc> data = hl2ss::lnm::download_calibration_rm_vlc(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "uv2xy", "extrinsics", "undistort_map", "intrinsics" });

        o[0]["uv2xy"]         = unpack_payload<float>(&data->uv2xy,         0, sizeof(data->uv2xy),         { 2, hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH });
        o[0]["extrinsics"]    = unpack_payload<float>(&data->extrinsics,    0, sizeof(data->extrinsics),    { 4, 4 });
        o[0]["undistort_map"] = unpack_payload<float>(&data->undistort_map, 0, sizeof(data->undistort_map), { 2, hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH });
        o[0]["intrinsics"]    = to_typed_array<float>(data->intrinsics, { sizeof(data->intrinsics) / sizeof(float) });

        outputs[0] = std::move(o);
    }

    void download_calibration_rm_depth_ahat(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_depth_ahat) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_depth_ahat> data = hl2ss::lnm::download_calibration_rm_depth_ahat(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "uv2xy", "extrinsics", "scale", "alias", "undistort_map", "intrinsics" });

        o[0]["uv2xy"]         = unpack_payload<float>(&data->uv2xy,         0, sizeof(data->uv2xy),         { 2, hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH });
        o[0]["extrinsics"]    = unpack_payload<float>(&data->extrinsics,    0, sizeof(data->extrinsics),    { 4, 4 });
        o[0]["scale"]         = m_factory.createScalar<float>(data->scale);
        o[0]["alias"]         = m_factory.createScalar<float>(data->alias);
        o[0]["undistort_map"] = unpack_payload<float>(&data->undistort_map, 0, sizeof(data->undistort_map), { 2, hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH });
        o[0]["intrinsics"]    = to_typed_array<float>(data->intrinsics, { sizeof(data->intrinsics) / sizeof(float) });

        outputs[0] = std::move(o);
    }

    void download_calibration_rm_depth_longthrow(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_depth_longthrow) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> data = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "uv2xy", "extrinsics", "scale", "undistort_map", "intrinsics" });

        o[0]["uv2xy"]         = unpack_payload<float>(&data->uv2xy,         0, sizeof(data->uv2xy),         { 2, hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH });
        o[0]["extrinsics"]    = unpack_payload<float>(&data->extrinsics,    0, sizeof(data->extrinsics),    { 4, 4 });
        o[0]["scale"]         = m_factory.createScalar<float>(data->scale);
        o[0]["undistort_map"] = unpack_payload<float>(&data->undistort_map, 0, sizeof(data->undistort_map), { 2, hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH });
        o[0]["intrinsics"]    = to_typed_array<float>(data->intrinsics, { sizeof(data->intrinsics) / sizeof(float) });

        outputs[0] = std::move(o);
    }

    void download_calibration_rm_imu(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_imu[port - hl2ss::stream_port::RM_IMU_ACCELEROMETER]) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_imu> data = hl2ss::lnm::download_calibration_rm_imu(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "extrinsics" });

        o[0]["extrinsics"] = unpack_payload<float>(&data->extrinsics, 0, sizeof(data->extrinsics), { 4, 4 });

        outputs[0] = std::move(o);
    }

    void download_calibration_pv(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (source_pv) { throw std::runtime_error("Cannot download calibration while streaming"); }

        uint16_t width     = get_argument<uint16_t>(inputs);
        uint16_t height    = get_argument<uint16_t>(inputs);
        uint8_t  framerate = get_argument<uint8_t>(inputs);

        std::shared_ptr<hl2ss::calibration_pv> data = hl2ss::lnm::download_calibration_pv(host, port, width, height, framerate);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "focal_length", "principal_point", "radial_distortion", "tangential_distortion", "projection", "extrinsics" });

        o[0]["focal_length"]          = to_typed_array<float>(data->focal_length,          { sizeof(data->focal_length)          / sizeof(float) });
        o[0]["principal_point"]       = to_typed_array<float>(data->principal_point,       { sizeof(data->principal_point)       / sizeof(float) });
        o[0]["radial_distortion"]     = to_typed_array<float>(data->radial_distortion,     { sizeof(data->radial_distortion)     / sizeof(float) });
        o[0]["tangential_distortion"] = to_typed_array<float>(data->tangential_distortion, { sizeof(data->tangential_distortion) / sizeof(float) });
        o[0]["projection"]            = unpack_payload<float>(&data->projection, 0, sizeof(data->projection), { 4, 4 });
        o[0]["extrinsics"]            = unpack_payload<float>(&data->extrinsics, 0, sizeof(data->extrinsics), { 4, 4 });

        outputs[0] = std::move(o);
    }

    void download_calibration(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host = get_argument_string(inputs);
        uint16_t    port = get_argument<uint16_t>(inputs);

        switch (port)
        {
        // Stream
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:     
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:      
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    download_calibration_rm_vlc(            host.c_str(), port, outputs, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        download_calibration_rm_depth_ahat(     host.c_str(), port, outputs, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   download_calibration_rm_depth_longthrow(host.c_str(), port, outputs, inputs); break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: 
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:     download_calibration_rm_imu(            host.c_str(), port, outputs, inputs); break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       download_calibration_pv(                host.c_str(), port, outputs, inputs); break;               
        default:                                       throw std::runtime_error("Unsupported port");
        }
    }

    //------------------------------------------------------------------------------
    // (*) IPC
    //------------------------------------------------------------------------------

    void ipc_call_rc(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (!ipc_rc) { throw std::runtime_error("Port not open"); }

        std::string f = get_argument_string(inputs);

        if (f == "get_application_version")
        {
        outputs[0] = to_typed_array<uint16_t>(ipc_rc->get_application_version().field, { 4 });
        }
        else if (f == "get_pv_subsystem_status")
        {
        outputs[0] = m_factory.createScalar<bool>(ipc_rc->get_pv_subsystem_status());
        }
        else if (f == "get_utc_offset")
        {
        uint32_t samples = get_argument<uint32_t>(inputs);
        outputs[0] = m_factory.createScalar<uint64_t>(ipc_rc->get_utc_offset(samples));
        }
        else if (f == "set_hs_marker_state")
        {
        uint32_t state = get_argument<uint32_t>(inputs);
        ipc_rc->set_hs_marker_state(state);
        }
        else if (f == "set_pv_backlight_compensation")
        {
        uint32_t state = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_backlight_compensation(state);
        }
        else if (f == "set_pv_exposure")
        {
        uint32_t mode  = get_argument<uint32_t>(inputs);
        uint32_t value = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_exposure(mode, value);
        }
        else if (f == "set_pv_exposure_priority_video")
        {
        uint32_t enabled = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_exposure_priority_video(enabled);
        }
        else if (f == "set_pv_focus")
        {
        uint32_t mode            = get_argument<uint32_t>(inputs);
        uint32_t range           = get_argument<uint32_t>(inputs);
        uint32_t distance        = get_argument<uint32_t>(inputs);
        uint32_t value           = get_argument<uint32_t>(inputs);
        uint32_t driver_fallback = get_argument<uint32_t>(inputs);

        ipc_rc->set_pv_focus(mode, range, distance, value, driver_fallback);
        }
        else if (f == "set_pv_iso_speed")
        {
        uint32_t mode  = get_argument<uint32_t>(inputs);
        uint32_t value = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_iso_speed(mode, value);
        }
        else if (f == "set_pv_scene_mode")
        {
        uint32_t mode = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_scene_mode(mode);
        }
        else if (f == "set_pv_video_temporal_denoising")
        {
        uint32_t mode = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_video_temporal_denoising(mode);
        }
        else if (f == "set_pv_white_balance_preset")
        {
        uint32_t preset = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_white_balance_preset(preset);
        }
        else if (f == "set_pv_white_balance_value")
        {
        uint32_t value = get_argument<uint32_t>(inputs);
        ipc_rc->set_pv_white_balance_value(value);
        }
        else if (f == "set_flat_mode")
        {
        uint32_t value = get_argument<uint32_t>(inputs);
        ipc_rc->set_flat_mode(value);
        }
        else
        {
        throw std::runtime_error("Unknown method");
        }
    }

    void ipc_call_sm(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (!ipc_sm) { throw std::runtime_error("Port not open"); }

        std::string f = get_argument_string(inputs);

        if (f == "create_observer")
        {
        ipc_sm->create_observer();
        }
        else if (f == "set_volumes")
        {
        matlab::data::StructArray p = get_argument_array<matlab::data::Struct>(inputs);

        hl2ss::sm_bounding_volume volumes;

        for (size_t i = 0; i < p.getNumberOfElements(); ++i)
        {
        switch (get_field_scalar<uint32_t>(p[i]["type"]))
        {
        case hl2ss::sm_volume_type::Box:         volumes.add_box(get_field_vector_3(p[i]["center"]), get_field_vector_3(p[i]["extents"])); break;
        case hl2ss::sm_volume_type::Frustum:     volumes.add_frustum(get_field_vector_4(p[i]["near"]), get_field_vector_4(p[i]["far"]), get_field_vector_4(p[i]["right"]), get_field_vector_4(p[i]["left"]), get_field_vector_4(p[i]["top"]), get_field_vector_4(p[i]["bottom"])); break;
        case hl2ss::sm_volume_type::OrientedBox: volumes.add_oriented_box(get_field_vector_3(p[i]["center"]), get_field_vector_3(p[i]["extents"]), get_field_vector_4(p[i]["orientation"])); break;
        case hl2ss::sm_volume_type::Sphere:      volumes.add_sphere(get_field_vector_3(p[i]["center"]), get_field_scalar<float>(p[i]["radius"])); break;
        default: break;
        }
        }

        ipc_sm->set_volumes(volumes);
        }
        else if (f == "get_observed_surfaces")
        {
        std::vector<hl2ss::sm_surface_info> surface_infos;

        ipc_sm->get_observed_surfaces(surface_infos);

        matlab::data::StructArray o = m_factory.createStructArray({ surface_infos.size() }, { "id", "update_time" });

        for (size_t i = 0; i < surface_infos.size(); ++i)
        {
        auto& info = surface_infos[i];

        o[i]["id"]          = to_typed_array<uint64_t>(&info.id, sizeof(info.id), { sizeof(info.id) / sizeof(uint64_t) });
        o[i]["update_time"] = m_factory.createScalar<uint64_t>(info.update_time);
        }

        outputs[0] = std::move(o);
        }
        else if (f == "get_meshes")
        {
        matlab::data::StructArray p       = get_argument_array<matlab::data::Struct>(inputs);
        uint32_t                  threads = get_argument<uint32_t>(inputs);

        hl2ss::sm_mesh_task tasks;

        for (size_t i = 0; i < p.getNumberOfElements(); ++i)
        {
        tasks.add_task(get_field_guid(p[i]["id"]), get_field_scalar<double>(p[i]["max_triangles_per_cubic_meter"]), get_field_scalar<uint32_t>(p[i]["vertex_position_format"]), get_field_scalar<uint32_t>(p[i]["triangle_index_format"]), get_field_scalar<uint32_t>(p[i]["vertex_normal_format"]), get_field_scalar<bool>(p[i]["include_vertex_normals"]), get_field_scalar<bool>(p[i]["include_bounds"]));
        }

        std::vector<hl2ss::sm_mesh> meshes;

        ipc_sm->get_meshes(tasks, threads, meshes);

        matlab::data::StructArray o = m_factory.createStructArray({ meshes.size() }, { "status", "vertex_position_scale", "pose", "bounds", "vertex_positions", "triangle_indices", "vertex_normals" });

        for (size_t i = 0; i < meshes.size(); ++i)
        {
        auto& mesh = meshes[i];

        o[i]["status"]                = m_factory.createScalar<uint32_t>(mesh.status);
        o[i]["vertex_position_scale"] = to_typed_array<float>(&mesh.vertex_position_scale, sizeof(mesh.vertex_position_scale), { sizeof(mesh.vertex_position_scale) / sizeof(float) });
        o[i]["pose"]                  = unpack_pose(&mesh.pose);
        o[i]["bounds"]                = to_typed_array<float>(&mesh.bounds,                sizeof(mesh.bounds),                { sizeof(mesh.bounds)                / sizeof(float) });

        switch (get_field_scalar<uint32_t>(p[i]["vertex_position_format"]))
        {
        case hl2ss::sm_vertex_position_format::R16G16B16A16IntNormalized: o[i]["vertex_positions"] = to_typed_array<uint16_t>(mesh.vertex_positions.data(), mesh.vertex_positions.size(), { 4, mesh.vertex_positions.size() / (4 * sizeof(uint16_t)) }); break;
        case hl2ss::sm_vertex_position_format::R32G32B32A32Float:         o[i]["vertex_positions"] = to_typed_array<float>(   mesh.vertex_positions.data(), mesh.vertex_positions.size(), { 4, mesh.vertex_positions.size() / (4 * sizeof(float)) });    break;
        }

        switch (get_field_scalar<uint32_t>(p[i]["triangle_index_format"]))
        {
        case hl2ss::sm_triangle_index_format::R16UInt: o[i]["triangle_indices"] = to_typed_array<uint16_t>(mesh.triangle_indices.data(), mesh.triangle_indices.size(), { 3, mesh.triangle_indices.size() / (3 * sizeof(uint16_t)) }); break;
        case hl2ss::sm_triangle_index_format::R32Uint: o[i]["triangle_indices"] = to_typed_array<uint32_t>(mesh.triangle_indices.data(), mesh.triangle_indices.size(), { 3, mesh.triangle_indices.size() / (3 * sizeof(uint32_t)) }); break;
        }

        switch (get_field_scalar<uint32_t>(p[i]["vertex_normal_format"]))
        {
        case hl2ss::sm_vertex_normal_format::R8G8B8A8IntNormalized: o[i]["vertex_normals"] = to_typed_array<uint8_t>(mesh.vertex_normals.data(), mesh.vertex_normals.size(), { 4, mesh.vertex_normals.size() / (4 * sizeof(uint8_t))}); break;
        case hl2ss::sm_vertex_normal_format::R32G32B32A32Float:     o[i]["vertex_normals"] = to_typed_array<float>(  mesh.vertex_normals.data(), mesh.vertex_normals.size(), { 4, mesh.vertex_normals.size() / (4 * sizeof(float))});   break;
        }
        }

        outputs[0] = std::move(o);
        }
        else
        {
        throw std::runtime_error("Unknown method");
        }
    }

    void ipc_call_su(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (!ipc_su) { throw std::runtime_error("Port not open"); }

        std::string f = get_argument_string(inputs);

        if (f == "query")
        {
        matlab::data::StructArray p = get_argument_array<matlab::data::Struct>(inputs);

        hl2ss::su_task task;
        
        task.enable_quads         = get_field_scalar<bool>(p[0]["enable_quads"]);
        task.enable_meshes        = get_field_scalar<bool>(p[0]["enable_meshes"]);
        task.enable_only_observed = get_field_scalar<bool>(p[0]["enable_only_observed"]);
        task.enable_world_mesh    = get_field_scalar<bool>(p[0]["enable_world_mesh"]);
        task.mesh_lod             = get_field_scalar<uint32_t>(p[0]["mesh_lod"]);
        task.query_radius         = get_field_scalar<float>(p[0]["query_radius"]);
        task.create_mode          = get_field_scalar<uint8_t>(p[0]["create_mode"]);
        task.kind_flags           = get_field_scalar<uint8_t>(p[0]["kind_flags"]);
        task.get_orientation      = get_field_scalar<bool>(p[0]["get_orientation"]);
        task.get_position         = get_field_scalar<bool>(p[0]["get_position"]);
        task.get_location_matrix  = get_field_scalar<bool>(p[0]["get_location_matrix"]);
        task.get_quad             = get_field_scalar<bool>(p[0]["get_quad"]);
        task.get_meshes           = get_field_scalar<bool>(p[0]["get_meshes"]); 
        task.get_collider_meshes  = get_field_scalar<bool>(p[0]["get_collider_meshes"]);

        matlab::data::TypedArray<uint64_t> guid_list = p[0]["guid_list"];
        for (size_t i = 0; i < (guid_list.getNumberOfElements() & ~1ULL); i += 2) { task.guid_list.push_back({ guid_list[i], guid_list[i + 1] }); }

        hl2ss::su_result result;

        ipc_su->query(task, result);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "status", "extrinsics", "pose", "items" });

        o[0]["status"]     = m_factory.createScalar<uint32_t>(result.status);
        o[0]["extrinsics"] = unpack_payload<float>(&result.extrinsics, 0, sizeof(result.extrinsics), { 4, 4 });
        o[0]["pose"]       = unpack_payload<float>(&result.pose,       0, sizeof(result.pose),       { 4, 4 });

        if (result.items.size() > 0)
        {
        matlab::data::StructArray items = m_factory.createStructArray({ result.items.size() }, { "id", "kind", "orientation", "position", "location", "alignment", "extents", "meshes", "collider_meshes" });
        
        for (size_t i = 0; i < result.items.size(); ++i)
        {
        auto& item = result.items[i];

        items[i]["id"]          = to_typed_array<uint64_t>(&item.id,          sizeof(item.id),          { sizeof(item.id)          / sizeof(uint64_t) });
        items[i]["kind"]        = m_factory.createScalar<int32_t>(item.kind);
        items[i]["orientation"] = to_typed_array<float>(   &item.orientation, sizeof(item.orientation), { sizeof(item.orientation) / sizeof(float) });
        items[i]["position"]    = to_typed_array<float>(   &item.position,    sizeof(item.position),    { sizeof(item.position)    / sizeof(float) });
        items[i]["location"]    = unpack_pose(&item.location);
        items[i]["alignment"]   = m_factory.createScalar<int32_t>(item.alignment);
        items[i]["extents"]     = to_typed_array<float>(   &item.extents,     sizeof(item.extents),     { sizeof(item.extents)     / sizeof(float) });

        if (item.meshes.size() > 0)
        {
        matlab::data::StructArray meshes = m_factory.createStructArray({ item.meshes.size() }, { "vertex_positions", "triangle_indices" });

        for (size_t j = 0; j < item.meshes.size(); ++j)
        {
        auto& mesh = item.meshes[j];

        meshes[j]["vertex_positions"] = to_typed_array<float>(   mesh.vertex_positions.data(), mesh.vertex_positions.size(), { 3, mesh.vertex_positions.size() / (3 * sizeof(float)) });
        meshes[j]["triangle_indices"] = to_typed_array<uint32_t>(mesh.triangle_indices.data(), mesh.triangle_indices.size(), { 3, mesh.triangle_indices.size() / (3 * sizeof(uint32_t)) });
        }

        items[i]["meshes"] = std::move(meshes);
        }

        if (item.collider_meshes.size() > 0)
        {
        matlab::data::StructArray collider_meshes = m_factory.createStructArray({ item.collider_meshes.size() }, { "vertex_positions", "triangle_indices" });

        for (size_t j = 0; j < item.collider_meshes.size(); ++j)
        {
        auto &collider_mesh = item.collider_meshes[j];

        collider_meshes[j]["vertex_positions"] = to_typed_array<float>(   collider_mesh.vertex_positions.data(), collider_mesh.vertex_positions.size(), { 3, collider_mesh.vertex_positions.size() / (3 * sizeof(float)) });
        collider_meshes[j]["triangle_indices"] = to_typed_array<uint32_t>(collider_mesh.triangle_indices.data(), collider_mesh.triangle_indices.size(), { 3, collider_mesh.triangle_indices.size() / (3 * sizeof(uint32_t)) });
        }

        items[i]["collider_meshes"] = std::move(collider_meshes);
        }
        }

        o[0]["items"] = std::move(items);   
        }

        outputs[0] = std::move(o);
        }
        else
        {
        throw std::runtime_error("Unknown method");
        }
    }

    void ipc_call_vi(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (!ipc_vi) { throw std::runtime_error("Port not open"); }

        std::string f = get_argument_string(inputs);

        if (f == "create_recognizer")
        {
        ipc_vi->create_recognizer();
        }
        else if (f == "register_commands")
        {
        bool                      clear    = get_argument<bool>(inputs);
        matlab::data::StringArray commands = get_argument_array<matlab::data::MATLABString>(inputs);

        std::vector<std::u16string> strings;
        for (size_t i = 0; i < commands.getNumberOfElements(); ++i) { if (commands[i].has_value()) { strings.push_back(commands[i]); } }

        bool ok = ipc_vi->register_commands(clear, strings);

        outputs[0] = m_factory.createScalar<bool>(ok);
        }
        else if (f == "start")
        {
        ipc_vi->start();
        }
        else if (f == "clear")
        {
        ipc_vi->clear();
        }
        else if (f == "pop")
        {
        std::vector<hl2ss::vi_result> results;

        ipc_vi->pop(results);

        matlab::data::StructArray o = m_factory.createStructArray({ results.size() }, { "index", "confidence", "phrase_duration", "phrase_start_time", "raw_confidence" });

        for (size_t i = 0; i < results.size(); ++i)
        {
        auto& result = results[i];

        o[i]["index"]             = m_factory.createScalar<uint32_t>(result.index);
        o[i]["confidence"]        = m_factory.createScalar<uint32_t>(result.confidence);
        o[i]["phrase_duration"]   = m_factory.createScalar<uint64_t>(result.phrase_duration);
        o[i]["phrase_start_time"] = m_factory.createScalar<uint64_t>(result.phrase_start_time);
        o[i]["raw_confidence"]    = m_factory.createScalar<double>(  result.raw_confidence);
        }

        outputs[0] = std::move(o);
        }
        else if (f == "stop")
        {
        ipc_vi->stop();
        }
        else
        {
        throw std::runtime_error("Unknown method");
        }
    }

    void ipc_call_umq(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (!ipc_umq) { throw std::runtime_error("Port not open"); }

        std::string f = get_argument_string(inputs);

        if (f == "push")
        {
        matlab::data::TypedArray<uint8_t> data = get_argument_array<uint8_t>(inputs);
        ipc_umq->push(get_pointer(data), data.getNumberOfElements());
        }
        else if (f == "pull")
        {
        uint32_t count = get_argument<uint32_t>(inputs);
        matlab::data::TypedArray<uint32_t> data = m_factory.createArray<uint32_t>({ count });
        ipc_umq->pull(get_pointer(data), count);
        outputs[0] = std::move(data);
        }
        else
        {
        throw std::runtime_error("Unknown method");
        }
    }

    void ipc_call(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint16_t port = get_argument<uint16_t>(inputs);

        switch (port)
        {
        case hl2ss::ipc_port::REMOTE_CONFIGURATION: ipc_call_rc( outputs, inputs); break;
        case hl2ss::ipc_port::SPATIAL_MAPPING:      ipc_call_sm( outputs, inputs); break;
        case hl2ss::ipc_port::SCENE_UNDERSTANDING:  ipc_call_su( outputs, inputs); break;
        case hl2ss::ipc_port::VOICE_INPUT:          ipc_call_vi( outputs, inputs); break;
        case hl2ss::ipc_port::UNITY_MESSAGE_QUEUE:  ipc_call_umq(outputs, inputs); break;
        default:                                    throw std::runtime_error("Unsupported port");
        }
    }

    //------------------------------------------------------------------------------
    // (*) Entry
    //------------------------------------------------------------------------------

    MexFunction()
    {
        try
        {
        hl2ss::client::initialize();
        m_initialized = true;
        }
        catch (const std::exception& e)
        {
        m_initialized = false;
        }
    }

    ~MexFunction()
    {
        if (m_initialized) { hl2ss::client::cleanup(); }
    }

    void select(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string action = get_argument_string(inputs);

        if      (action == "get_packet")           { get_packet(outputs, inputs); }
        else if (action == "ipc_call")             { ipc_call(outputs, inputs); }
        else if (action == "open")                 { open(outputs, inputs); }
        else if (action == "close")                { close(outputs, inputs); }
        else if (action == "start_subsystem_pv")   { start_subsystem_pv(outputs, inputs); }
        else if (action == "stop_subsystem_pv")    { stop_subsystem_pv(outputs, inputs); }
        else if (action == "download_calibration") { download_calibration(outputs, inputs); }
        else                                       { throw std::runtime_error("Unknown action"); }
    }

    void operator() (matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        m_argument_index = 0;
        try { select(outputs, inputs); } catch (const std::exception& e) { error(e.what()); }
    }
};
