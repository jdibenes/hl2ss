
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "hl2ss_lnm.h"

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

    std::unique_ptr<hl2ss::rx_rm_vlc> rx_rm_vlc_leftfront;
    std::unique_ptr<hl2ss::rx_rm_vlc> rx_rm_vlc_leftleft;
    std::unique_ptr<hl2ss::rx_rm_vlc> rx_rm_vlc_rightfront;
    std::unique_ptr<hl2ss::rx_rm_vlc> rx_rm_vlc_rightright;
    std::unique_ptr<hl2ss::rx_rm_depth_ahat> rx_rm_depth_ahat;
    std::unique_ptr<hl2ss::rx_rm_depth_longthrow> rx_rm_depth_longthrow;
    std::unique_ptr<hl2ss::rx_rm_imu> rx_rm_imu_accelerometer;
    std::unique_ptr<hl2ss::rx_rm_imu> rx_rm_imu_gyroscope;
    std::unique_ptr<hl2ss::rx_rm_imu> rx_rm_imu_magnetometer;
    std::unique_ptr<hl2ss::rx_pv> rx_pv;
    std::unique_ptr<hl2ss::rx_microphone> rx_microphone;
    std::unique_ptr<hl2ss::rx_si> rx_si;
    std::unique_ptr<hl2ss::rx_eet> rx_eet;



    
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
        matlab::data::TypedArray<T> argument = std::move(inputs[m_argument_index++]);
        return argument[0];
    }

    template <>
    std::string get_argument(matlab::mex::ArgumentList inputs)
    {
        matlab::data::CharArray argument = std::move(inputs[m_argument_index++]);
        return argument.toAscii();
    }

    void error(char const* message)
    {
        m_matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ m_factory.createScalar(message) });
    }



    
    





    





    void close_rx_rm_vlc_leftfront()
    {
        if (!rx_rm_vlc_leftfront) { return; }
        rx_rm_vlc_leftfront->close();
        rx_rm_vlc_leftfront = nullptr;
    }

    void close_rx_rm_vlc_leftleft()
    {
        if (!rx_rm_vlc_leftleft) { return; }
        rx_rm_vlc_leftleft->close();
        rx_rm_vlc_leftleft = nullptr;
    }

    void close_rx_rm_vlc_rightfront()
    {
        if (!rx_rm_vlc_rightfront) { return; }
        rx_rm_vlc_rightfront->close();
        rx_rm_vlc_rightfront = nullptr;
    }

    void close_rx_rm_vlc_rightright()
    {
        if (!rx_rm_vlc_rightright) { return; }
        rx_rm_vlc_rightright->close();
        rx_rm_vlc_rightright = nullptr;
    }

    void close_rx_rm_depth_ahat()
    {
        if (!rx_rm_depth_ahat) { return; }
        rx_rm_depth_ahat->close();
        rx_rm_depth_ahat = nullptr;
    }

    void close_rx_rm_depth_longthrow()
    {
        if (!rx_rm_depth_longthrow) { return; }
        rx_rm_depth_longthrow->close();
        rx_rm_depth_longthrow = nullptr;
    }

    void close_rx_rm_imu_accelerometer()
    {
        if (!rx_rm_imu_accelerometer) { return; }
        rx_rm_imu_accelerometer->close();
        rx_rm_imu_accelerometer = nullptr;
    }

    void close_rx_rm_imu_gyroscope()
    {
        if (!rx_rm_imu_gyroscope) { return; }
        rx_rm_imu_gyroscope->close();
        rx_rm_imu_gyroscope = nullptr;
    }

    void close_rx_rm_imu_magnetometer()
    {
        if (!rx_rm_imu_magnetometer) { return; }
        rx_rm_imu_magnetometer->close();
        rx_rm_imu_magnetometer = nullptr;
    }

    void close_rx_pv()
    {
        if (!rx_pv) { return; }
        rx_pv->close();
        hl2ss::lnm::stop_subsystem_pv(rx_pv->host.c_str(), rx_pv->port);
        rx_pv = nullptr;
    }

    void close_rx_microphone()
    {
        if (!rx_microphone) { return; }
        rx_microphone->close();
        rx_microphone = nullptr;
    }

    void close_rx_si()
    {
        if (!rx_si) { return; }
        rx_si->close();
        rx_si = nullptr;
    }

    void close_rx_eet()
    {
        if (!rx_eet) { return; }
        rx_eet->close();
        rx_eet = nullptr;
    }




    


    void get_next_packet_rx_pv(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::shared_ptr<hl2ss::packet> packet = rx_pv->get_next_packet();

        outputs[0] = m_factory.createScalar<uint64_t>(packet->timestamp);
        outputs[1] = m_factory.createArrayFromBuffer<uint8_t>({rx_pv->height, rx_pv->width, 3}, matlab::data::buffer_ptr_t<uint8_t>(packet->payload.release(), default_deleter), matlab::data::MemoryLayout::ROW_MAJOR);
        outputs[2] = m_factory.createArrayFromBuffer<float>({4, 4}, matlab::data::buffer_ptr_t<float>(packet->pose.release(), default_deleter), matlab::data::MemoryLayout::ROW_MAJOR);
    }

    void open_pv(char const* host, uint16_t port, matlab::mex::ArgumentList inputs)
    {
        bool     enable_mrc = get_argument<bool>(inputs);
        uint16_t width      = get_argument<uint16_t>(inputs);
        uint16_t height     = get_argument<uint16_t>(inputs);
        uint8_t  framerate  = get_argument<uint8_t>(inputs);

        hl2ss::lnm::start_subsystem_pv(host, port, enable_mrc);
        rx_pv = hl2ss::lnm::rx_pv(host, port, width, height, framerate);
        rx_pv->open();
    }





    void open(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string host = get_argument<std::string>(inputs);
        uint16_t port    = get_argument<uint16_t>(inputs);

        switch (port)
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT: break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT: break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT: break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT: break;
        case hl2ss::stream_port::RM_DEPTH_AHAT: break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW: break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE: break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER: break;
        case hl2ss::stream_port::PERSONAL_VIDEO: open_pv(host.c_str(), port, inputs); break;
        case hl2ss::stream_port::MICROPHONE: break;
        case hl2ss::stream_port::SPATIAL_INPUT: break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    void get_next_packet(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint16_t port = get_argument<uint16_t>(inputs);

        switch (port)
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT: break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT: break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT: break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT: break;
        case hl2ss::stream_port::RM_DEPTH_AHAT: break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW: break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE: break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER: break;
        case hl2ss::stream_port::PERSONAL_VIDEO: get_next_packet_rx_pv(outputs, inputs); break;
        case hl2ss::stream_port::MICROPHONE: break;
        case hl2ss::stream_port::SPATIAL_INPUT: break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    void close(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        uint16_t port = get_argument<uint16_t>(inputs);

        switch (port)
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:     close_rx_rm_vlc_leftfront();     break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:      close_rx_rm_vlc_leftleft();      break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    close_rx_rm_vlc_rightfront();    break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    close_rx_rm_vlc_rightright();    break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:        close_rx_rm_depth_ahat();        break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   close_rx_rm_depth_longthrow();   break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER: close_rx_rm_imu_accelerometer(); break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:     close_rx_rm_imu_gyroscope();     break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  close_rx_rm_imu_magnetometer();  break;
        case hl2ss::stream_port::PERSONAL_VIDEO:       close_rx_pv();                   break;
        case hl2ss::stream_port::MICROPHONE:           close_rx_microphone();           break;
        case hl2ss::stream_port::SPATIAL_INPUT:        close_rx_si();                   break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: close_rx_eet();                  break;
        default:                                       throw std::runtime_error("Unknown port");
        }
    }

    void select(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        std::string action = get_argument<std::string>(inputs);

        if      (action == "open")            { open(outputs, inputs); }
        else if (action == "get_next_packet") { get_next_packet(outputs, inputs); }
        else if (action == "close")           { close(outputs, inputs); }
        else                                  { throw std::runtime_error("Unknown action"); }
    }

    void operator() (matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs)
    {
        m_argument_index = 0;
        select(outputs, inputs);
    }
};

      // if (inputs.size() < 1) { m_matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ m_factory.createScalar("No action specified") }); }
      //if (inputs[0].getType() != matlab::data::ArrayType::CHAR) { m_matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ m_factory.createScalar("Input 0 must be char") }); }
      //        if (inputs.size() < 2) { m_matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ m_factory.createScalar("No port specified") }); }