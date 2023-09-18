
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"


static void default_deleter(void* p)
{
    delete[] p;
}

typedef std::vector<uint64_t> options_t;

class MexFunction : public matlab::mex::Function
{
private:
    std::shared_ptr<matlab::engine::MATLABEngine> m_matlabPtr = getEngine();
    matlab::data::ArrayFactory m_factory;
    uint32_t m_argument_index;

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
    MexFunction()
    {
    }

    ~MexFunction()
    {
    }

    //------------------------------------------------------------------------------
    // Helpers
    //------------------------------------------------------------------------------

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

    template <>
    std::string get_argument(matlab::mex::ArgumentList inputs)
    {
        matlab::data::CharArray argument = get_argument_array<CHAR16_T>(inputs);
        return argument.toAscii();
    }

    template <typename T>
    std::vector<T> to_std_vector(matlab::data::TypedArray<T> t)
    {
        return std::vector<T>(t.begin(), t.end());
    }

    void error(char const* message)
    {
        m_matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ m_factory.createScalar(message) });
    }

    //------------------------------------------------------------------------------
    // Open
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
        uint8_t   decoded_format =               get_argument<uint8_t>(inputs);
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
        std::string host = get_argument<std::string>(inputs);
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
    // Close
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
    // Grab
    //------------------------------------------------------------------------------

    template <typename T>
    matlab::data::Array unpack_payload(uint8_t const* payload, uint32_t offset, uint32_t size, matlab::data::ArrayDimensions dims)
    {
        std::unique_ptr<T[]> payload_copy = std::make_unique<T[]>(size / sizeof(T));
        memcpy(payload_copy.get(), payload + offset, size);
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

    void pack_pv(int64_t frame_index, int32_t status, hl2ss::packet* packet, uint16_t width, uint16_t height, matlab::mex::ArgumentList outputs)
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
        o[0]["image"]       = unpack_payload<uint8_t>(packet->payload.get(), 0,                         image_size, { height,  width, image_size / (sizeof(uint8_t) *  height *  width) });
        o[0]["intrinsics"]  = unpack_payload<float>(  packet->payload.get(), image_size, hl2ss::decoder_pv::K_SIZE, { 4 });
        }
        if (packet->pose)
        {
        o[0]["pose"]        = unpack_pose(packet->pose.get());
        }
        }

        outputs[0] = std::move(o);
    }

    void pack_microphone(int64_t frame_index, int32_t status, hl2ss::packet* packet, uint8_t profile, matlab::mex::ArgumentList outputs)
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
        o[0]["audio"]       = unpack_payload<float>(  packet->payload.get(), 0, packet->sz_payload, { hl2ss::parameters_microphone::CHANNELS, packet->sz_payload / (sizeof(float) * hl2ss::parameters_microphone::CHANNELS) });
        }
        else
        {
        o[0]["audio"]       = unpack_payload<int16_t>(packet->payload.get(), 0, packet->sz_payload, { packet->sz_payload / (sizeof(int16_t) * hl2ss::parameters_microphone::CHANNELS), hl2ss::parameters_microphone::CHANNELS });
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
        o[0]["head_pose"]   = m_factory.createArray<float>({ 3, 3 },  (float*)&base->head_pose,  (float*)((uint8_t*)(&base->head_pose)  + sizeof(base->head_pose)));
        o[0]["eye_ray"]     = m_factory.createArray<float>({ 6 },     (float*)&base->eye_ray,    (float*)((uint8_t*)(&base->eye_ray)    + sizeof(base->eye_ray)));
        o[0]["left_hand"]   = m_factory.createArray<float>({ 9, 26 }, (float*)&base->left_hand,  (float*)((uint8_t*)(&base->left_hand)  + sizeof(base->left_hand)));
        o[0]["right_hand"]  = m_factory.createArray<float>({ 9, 26 }, (float*)&base->right_hand, (float*)((uint8_t*)(&base->right_hand) + sizeof(base->right_hand)));
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

        o[0]["combined_ray"]      = m_factory.createArray<float>({ 6 }, (float*)&base->combined_ray, (float*)((uint8_t*)(&base->combined_ray) + sizeof(base->combined_ray)));
        o[0]["left_ray"]          = m_factory.createArray<float>({ 6 }, (float*)&base->left_ray,     (float*)((uint8_t*)(&base->left_ray)     + sizeof(base->left_ray)));
        o[0]["right_ray"]         = m_factory.createArray<float>({ 6 }, (float*)&base->right_ray,    (float*)((uint8_t*)(&base->right_ray)    + sizeof(base->right_ray)));
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
        o[0]["audio"]       = unpack_payload<int16_t>(packet->payload.get(), 0, packet->sz_payload, { packet->sz_payload / (sizeof(int16_t) * hl2ss::parameters_extended_audio::CHANNELS), hl2ss::parameters_extended_audio::CHANNELS });
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
        if (type == 0)
        {
        frame_index = get_argument<int64_t>(inputs);
        return source->get_packet(frame_index, status);
        }
        else if (type == 1)
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
        hl2ss::rx_pv const* p_rx = source->get_rx<hl2ss::rx_pv>();

        pack_pv(frame_index, status, packet.get(), p_rx->width, p_rx->height, outputs);       
    }

    void get_packet_microphone(uint16_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)port;

        hl2ss::mt::source* source = source_microphone.get();

        int64_t frame_index;
        int32_t status;
        std::shared_ptr<hl2ss::packet> packet = grab(source, frame_index, status, inputs);
        hl2ss::rx_microphone const* p_rx = source->get_rx<hl2ss::rx_microphone>();

        pack_microphone(frame_index, status, packet.get(), p_rx->profile, outputs);        
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
    // Control
    //------------------------------------------------------------------------------

    void start_subsystem_pv(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host                       = get_argument<std::string>(inputs);
        uint16_t    port                       = get_argument<uint16_t>(inputs);
        bool        enable_mrc                 = get_argument<bool>(inputs);
        bool        hologram_composition       = get_argument<bool>(inputs);
        bool        recording_indicator        = get_argument<bool>(inputs);
        bool        video_stabilization        = get_argument<bool>(inputs);
        bool        blank_protected            = get_argument<bool>(inputs);
        bool        show_mesh                  = get_argument<bool>(inputs);
        float       global_opacity             = get_argument<float>(inputs);
        float       output_width               = get_argument<float>(inputs);
        float       output_height              = get_argument<float>(inputs);
        uint32_t    video_stabilization_length = get_argument<uint32_t>(inputs);
        uint32_t    hologram_perspective       = get_argument<uint32_t>(inputs);

        hl2ss::lnm::start_subsystem_pv(host.c_str(), port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective);
    }

    void stop_subsystem_pv(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host = get_argument<std::string>(inputs);
        uint16_t    port = get_argument<uint16_t>(inputs);

        hl2ss::lnm::stop_subsystem_pv(host.c_str(), port);
    }

    //------------------------------------------------------------------------------
    // Calibration
    //------------------------------------------------------------------------------

    void download_calibration_rm_vlc(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_vlc[port - hl2ss::stream_port::RM_VLC_LEFTFRONT]) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_vlc> data = hl2ss::lnm::download_calibration_rm_vlc(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "uv2xy", "extrinsics", "undistort_map", "intrinsics" });

        o[0]["uv2xy"]         = unpack_payload<float>((uint8_t*)&data->uv2xy,         0, sizeof(data->uv2xy),         { 2, hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH });
        o[0]["extrinsics"]    = unpack_payload<float>((uint8_t*)&data->extrinsics,    0, sizeof(data->extrinsics),    { 4, 4 });
        o[0]["undistort_map"] = unpack_payload<float>((uint8_t*)&data->undistort_map, 0, sizeof(data->undistort_map), { 2, hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH });
        o[0]["intrinsics"]    = m_factory.createArray<float>({ 4 }, (float*)&data->intrinsics, (float*)((uint8_t*)(&data->intrinsics) + sizeof(data->intrinsics)));

        outputs[0] = std::move(o);
    }

    void download_calibration_rm_depth_ahat(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_depth_ahat) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_depth_ahat> data = hl2ss::lnm::download_calibration_rm_depth_ahat(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "uv2xy", "extrinsics", "scale", "alias", "undistort_map", "intrinsics" });

        o[0]["uv2xy"]         = unpack_payload<float>((uint8_t*)&data->uv2xy,         0, sizeof(data->uv2xy),         { 2, hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH });
        o[0]["extrinsics"]    = unpack_payload<float>((uint8_t*)&data->extrinsics,    0, sizeof(data->extrinsics),    { 4, 4 });
        o[0]["scale"]         = m_factory.createScalar<float>(data->scale);
        o[0]["alias"]         = m_factory.createScalar<float>(data->alias);
        o[0]["undistort_map"] = unpack_payload<float>((uint8_t*)&data->undistort_map, 0, sizeof(data->undistort_map), { 2, hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH });
        o[0]["intrinsics"]    = m_factory.createArray<float>({ 4 }, (float*)&data->intrinsics, (float*)((uint8_t*)(&data->intrinsics) + sizeof(data->intrinsics)));

        outputs[0] = std::move(o);
    }

    void download_calibration_rm_depth_longthrow(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_depth_longthrow) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> data = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "uv2xy", "extrinsics", "scale", "undistort_map", "intrinsics" });

        o[0]["uv2xy"]         = unpack_payload<float>((uint8_t*)&data->uv2xy,         0, sizeof(data->uv2xy),         { 2, hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH });
        o[0]["extrinsics"]    = unpack_payload<float>((uint8_t*)&data->extrinsics,    0, sizeof(data->extrinsics),    { 4, 4 });
        o[0]["scale"]         = m_factory.createScalar<float>(data->scale);
        o[0]["undistort_map"] = unpack_payload<float>((uint8_t*)&data->undistort_map, 0, sizeof(data->undistort_map), { 2, hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH });
        o[0]["intrinsics"]    = m_factory.createArray<float>({ 4 }, (float*)&data->intrinsics, (float*)((uint8_t*)(&data->intrinsics) + sizeof(data->intrinsics)));

        outputs[0] = std::move(o);
    }

    void download_calibration_rm_imu(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        (void)inputs;

        if (source_rm_imu[port - hl2ss::stream_port::RM_IMU_ACCELEROMETER]) { throw std::runtime_error("Cannot download calibration while streaming"); }

        std::shared_ptr<hl2ss::calibration_rm_imu> data = hl2ss::lnm::download_calibration_rm_imu(host, port);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "extrinsics" });

        o[0]["extrinsics"] = unpack_payload<float>((uint8_t*)&data->extrinsics, 0, sizeof(data->extrinsics), { 4, 4 });

        outputs[0] = std::move(o);
    }

    void download_calibration_pv(char const* host, uint32_t port, matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        if (source_pv) { throw std::runtime_error("Cannot download calibration while streaming"); }

        uint16_t width     = get_argument<uint16_t>(inputs);
        uint16_t height    = get_argument<uint16_t>(inputs);
        uint8_t  framerate = get_argument<uint8_t>(inputs);

        std::shared_ptr<hl2ss::calibration_pv> data = hl2ss::lnm::download_calibration_pv(host, port, width, height, framerate);

        matlab::data::StructArray o = m_factory.createStructArray({ 1 }, { "focal_length", "principal_point", "radial_distortion", "tangential_distortion", "projection" });

        o[0]["focal_length"]          = m_factory.createArray<float>({ 2 }, (float*)&data->focal_length,          (float*)((uint8_t*)(&data->focal_length)          + sizeof(data->focal_length)));
        o[0]["principal_point"]       = m_factory.createArray<float>({ 2 }, (float*)&data->principal_point,       (float*)((uint8_t*)(&data->principal_point)       + sizeof(data->principal_point)));
        o[0]["radial_distortion"]     = m_factory.createArray<float>({ 3 }, (float*)&data->radial_distortion,     (float*)((uint8_t*)(&data->radial_distortion)     + sizeof(data->radial_distortion)));
        o[0]["tangential_distortion"] = m_factory.createArray<float>({ 2 }, (float*)&data->tangential_distortion, (float*)((uint8_t*)(&data->tangential_distortion) + sizeof(data->tangential_distortion)));
        o[0]["projection"]            = unpack_payload<float>((uint8_t*)&data->projection, 0, sizeof(data->projection), { 4, 4 });

        outputs[0] = std::move(o);
    }

    void download_calibration(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host = get_argument<std::string>(inputs);
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
    // IPC
    //------------------------------------------------------------------------------


    void ipc_select(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {

    }

    void select(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string action = get_argument<std::string>(inputs);

        if      (action == "get_packet")           { get_packet(outputs, inputs); }
        else if (action == "ipc_select")           { ipc_select(outputs, inputs); }
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
        try { select(outputs, inputs); } catch(const std::exception& e) { error(e.what()); }
    }
};
