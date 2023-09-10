
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"


static void default_deleter(void* p)
{
    delete[] p;
}

class MexFunction : public matlab::mex::Function
{
private:
    std::shared_ptr<matlab::engine::MATLABEngine> m_matlabPtr = getEngine();
    matlab::data::ArrayFactory m_factory;
    uint32_t m_argument_index;

    uint16_t pv_width;
    uint16_t pv_height;
    uint8_t  pv_framerate;
    uint8_t  pv_bpp;

    std::unique_ptr<hl2ss::mt::source> source_rm_vlc_leftfront;
    std::unique_ptr<hl2ss::mt::source> source_rm_vlc_leftleft;
    std::unique_ptr<hl2ss::mt::source> source_rm_vlc_rightfront;
    std::unique_ptr<hl2ss::mt::source> source_rm_vlc_rightright;
    std::unique_ptr<hl2ss::mt::source> source_rm_depth_ahat;
    std::unique_ptr<hl2ss::mt::source> source_rm_depth_longthrow;
    std::unique_ptr<hl2ss::mt::source> source_rm_imu_accelerometer;
    std::unique_ptr<hl2ss::mt::source> source_rm_imu_gyroscope;
    std::unique_ptr<hl2ss::mt::source> source_rm_imu_magnetometer;
    std::unique_ptr<hl2ss::mt::source> source_pv;
    std::unique_ptr<hl2ss::mt::source> source_microphone;
    std::unique_ptr<hl2ss::mt::source> source_si;
    std::unique_ptr<hl2ss::mt::source> source_eet;

public:
    MexFunction()
    {
    }

    ~MexFunction()
    {
    }

    template <typename T>
    T get_argument(matlab::mex::ArgumentList inputs)
    {
        if (m_argument_index >= inputs.size()) { throw std::runtime_error("Not enough inputs"); }
        matlab::data::TypedArray<T> argument = std::move(inputs[m_argument_index++]);
        return argument[0];
    }

    template <>
    std::string get_argument(matlab::mex::ArgumentList inputs)
    {
        if (m_argument_index >= inputs.size()) { throw std::runtime_error("Not enough inputs"); }
        matlab::data::CharArray argument = std::move(inputs[m_argument_index++]);
        return argument.toAscii();
    }

    void error(char const* message)
    {
        m_matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ m_factory.createScalar(message) });
    }

    void open_rm_vlc_leftfront(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        // PARAMS
        source_rm_vlc_leftfront = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_vlc(host, port));
        source_rm_vlc_leftfront->start();
    }

    void open_rm_vlc_leftleft(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        // PARAMS
        source_rm_vlc_leftleft = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_vlc(host, port));
        source_rm_vlc_leftleft->start();
    }

    void open_rm_vlc_rightfront(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        // PARAMS
        source_rm_vlc_rightfront = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_vlc(host, port));
        source_rm_vlc_rightfront->start();
    }

    void open_rm_vlc_rightright(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        // PARAMS
        source_rm_vlc_rightright = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_vlc(host, port));
        source_rm_vlc_rightright->start();
    }

    void open_rm_depth_ahat(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        // PARAMS
        source_rm_depth_ahat = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_ahat(host, port));
        source_rm_depth_ahat->start();
    }

    void open_rm_depth_longthrow(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        // PARAMS
        source_rm_depth_longthrow = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_longthrow(host, port));
        source_rm_depth_longthrow->start();
    }

    void open_pv(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        uint16_t width          = get_argument<uint16_t>(inputs);
        uint16_t height         = get_argument<uint16_t>(inputs);
        uint8_t  framerate      = get_argument<uint8_t>(inputs);
        uint8_t  decoded_format = get_argument<uint8_t>(inputs);
        // PARAMS

        std::unique_ptr<hl2ss::rx> rx = hl2ss::lnm::rx_pv(host, port, width, height, framerate);
        static_cast<hl2ss::rx_decoded_pv*>(rx.get())->decoded_format = decoded_format;

        pv_width     = width;
        pv_height    = height;
        pv_framerate = framerate;
        pv_bpp       = hl2ss::decoder_pv::decoded_bpp(decoded_format);

        source_pv = std::make_unique<hl2ss::mt::source>(buffer_size, std::move(rx));
        source_pv->start();
    }

    void open_microphone(uint64_t buffer_size, char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        // PARAMS
        source_microphone = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_microphone(host, port));
        source_microphone->start();
    }

    void open(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint64_t    buffer_size = get_argument<uint64_t>(inputs);
        std::string host        = get_argument<std::string>(inputs);
        uint16_t    port        = get_argument<uint16_t>(inputs);
        // STREAMS
        
        switch (port)
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:     open_rm_vlc_leftfront(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:      open_rm_vlc_leftleft(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    open_rm_vlc_rightfront(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    open_rm_vlc_rightright(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        open_rm_depth_ahat(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   open_rm_depth_longthrow(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:     break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       open_pv(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::MICROPHONE:           open_microphone(buffer_size, host.c_str(), port, inputs); break;
        case hl2ss::stream_port::SPATIAL_INPUT:        break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    template <typename T>
    void get_packet(matlab::mex::ArgumentList outputs, hl2ss::mt::source* source, int64_t frame_stamp, matlab::data::ArrayDimensions dims)
    {
        int32_t state;
        std::shared_ptr<hl2ss::packet> packet = source->get_packet(frame_stamp, state);

        outputs[0] = m_factory.createScalar<int64_t>(frame_stamp);
        outputs[1] = m_factory.createScalar<int32_t>(state);

        if (state != 0)
        {
        outputs[2] = m_factory.createEmptyArray();
        outputs[3] = m_factory.createEmptyArray();
        outputs[4] = m_factory.createEmptyArray();
        }
        else
        {
        outputs[2] = m_factory.createScalar<uint64_t>(packet->timestamp);

        std::unique_ptr<T[]> payload = std::make_unique<T[]>((packet->sz_payload) / sizeof(T));
        memcpy(payload.get(), packet->payload.get(), packet->sz_payload);
        outputs[3] = m_factory.createArrayFromBuffer<T>(dims, matlab::data::buffer_ptr_t<T>(payload.release(), default_deleter), matlab::data::MemoryLayout::ROW_MAJOR);

        if (packet->pose)
        {
        std::unique_ptr<float[]> pose = std::make_unique<float[]>(packet->NE_POSE);
        memcpy(pose.get(), packet->pose.get(), packet->SZ_POSE);
        outputs[4] = m_factory.createArrayFromBuffer<float>({4, 4}, matlab::data::buffer_ptr_t<float>(pose.release(), default_deleter), matlab::data::MemoryLayout::ROW_MAJOR);
        }
        else
        {
        outputs[4] = m_factory.createEmptyArray();
        }
        }
    }

    void get_packet(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint16_t port        = get_argument<uint16_t>(inputs);
        int64_t  frame_stamp = get_argument<int64_t>(inputs);
        // STREAMS

        switch (port)
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:     get_packet<uint8_t>( outputs, source_rm_vlc_leftfront.get(),   frame_stamp, {     hl2ss::parameters_rm_vlc::HEIGHT,             hl2ss::parameters_rm_vlc::WIDTH });              break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:      get_packet<uint8_t>( outputs, source_rm_vlc_leftleft.get(),    frame_stamp, {     hl2ss::parameters_rm_vlc::HEIGHT,             hl2ss::parameters_rm_vlc::WIDTH });              break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    get_packet<uint8_t>( outputs, source_rm_vlc_rightfront.get(),  frame_stamp, {     hl2ss::parameters_rm_vlc::HEIGHT,             hl2ss::parameters_rm_vlc::WIDTH });              break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    get_packet<uint8_t>( outputs, source_rm_vlc_rightright.get(),  frame_stamp, {     hl2ss::parameters_rm_vlc::HEIGHT,             hl2ss::parameters_rm_vlc::WIDTH });              break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        get_packet<uint16_t>(outputs, source_rm_depth_ahat.get(),      frame_stamp, { 2 * hl2ss::parameters_rm_depth_ahat::HEIGHT,      hl2ss::parameters_rm_depth_ahat::WIDTH });       break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   get_packet<uint16_t>(outputs, source_rm_depth_longthrow.get(), frame_stamp, { 2 * hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH });  break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:     break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       get_packet<uint8_t>( outputs, source_pv.get(),                 frame_stamp, { pv_height, pv_width, pv_bpp }); break;
        case hl2ss::stream_port::MICROPHONE:           get_packet<float>(   outputs, source_microphone.get(),         frame_stamp, {     hl2ss::parameters_microphone::CHANNELS,       hl2ss::parameters_microphone::GROUP_SIZE_AAC }); break;
        case hl2ss::stream_port::SPATIAL_INPUT:        break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    void close(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint16_t port = get_argument<uint16_t>(inputs);

        switch (port)
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:     source_rm_vlc_leftfront     = nullptr; break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:      source_rm_vlc_leftleft      = nullptr; break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    source_rm_vlc_rightfront    = nullptr; break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    source_rm_vlc_rightright    = nullptr; break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        source_rm_depth_ahat        = nullptr; break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   source_rm_depth_longthrow   = nullptr; break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: source_rm_imu_accelerometer = nullptr; break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:     source_rm_imu_gyroscope     = nullptr; break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  source_rm_imu_magnetometer  = nullptr; break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       source_pv                   = nullptr; break;
        case hl2ss::stream_port::MICROPHONE:           source_microphone           = nullptr; break;
        case hl2ss::stream_port::SPATIAL_INPUT:        source_si                   = nullptr; break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: source_eet                  = nullptr; break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    void start_subsystem_pv(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host       = get_argument<std::string>(inputs);
        uint16_t    port       = get_argument<uint16_t>(inputs);
        bool        enable_mrc = get_argument<bool>(inputs);
        // PARAMS

        hl2ss::lnm::start_subsystem_pv(host.c_str(), port, enable_mrc);
    }

    void stop_subsystem_pv(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host = get_argument<std::string>(inputs);
        uint16_t    port = get_argument<uint16_t>(inputs);

        hl2ss::lnm::stop_subsystem_pv(host.c_str(), port);
    }

    void select(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string action = get_argument<std::string>(inputs);

        if      (action == "get_packet")         { get_packet(outputs, inputs); }
        else if (action == "open")               { open(outputs, inputs); }
        else if (action == "close")              { close(outputs, inputs); }
        else if (action == "start_subsystem_pv") { start_subsystem_pv(outputs, inputs); }
        else if (action == "stop_subsystem_pv")  { stop_subsystem_pv(outputs, inputs); }
        // IPC
        else                                     { throw std::runtime_error("Unknown action"); }
    }

    void operator() (matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        m_argument_index = 0;
        select(outputs, inputs);
    }
};
