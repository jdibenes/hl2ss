
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/highgui.hpp>
#include "hl2ss_ulm.h"



void test_sources(char const* host)
{
    hl2ss::svc::start_subsystem_pv(host, hl2ss::stream_port::PERSONAL_VIDEO);
    hl2ss::ulm::configuration_pv cfg_pv;
    hl2ss::ulm::configuration_rm_vlc cfg_vlc;
    hl2ss::svc::create_configuration_pv(cfg_pv);
    hl2ss::svc::create_configuration_rm_vlc(cfg_vlc);
    
    int const x= sizeof(hl2ss::ulm::configuration_pv);

    std::unique_ptr<hl2ss::svc::source> source_pv = hl2ss::svc::open_stream(host, hl2ss::stream_port::PERSONAL_VIDEO, 300, &cfg_pv);

    std::shared_ptr<hl2ss::svc::device_list> dev_list = hl2ss::svc::download_device_list(host, hl2ss::stream_port::EXTENDED_AUDIO);
    std::shared_ptr<hl2ss::svc::calibration<hl2ss::calibration_rm_vlc>> calibration = hl2ss::svc::download_calibration<hl2ss::calibration_rm_vlc>(host, hl2ss::stream_port::RM_VLC_LEFTFRONT, &cfg_vlc);

    std::cout << calibration->data->intrinsics[0] << ", " << calibration->data->intrinsics[1] << std::endl;

    //std::shared_ptr<hl2ss::svc::ipc_rc> ipc_rc = hl2ss::svc::open_ipc<hl2ss::svc::ipc_rc>(host, hl2ss::ipc_port::REMOTE_CONFIGURATION);
    //ipc_rc->set_pv_exposure(hl2ss::pv_exposure_mode::Manual, hl2ss::pv_exposure_value::Min);
    //ipc_rc.reset();


    while (true)
    {
        std::shared_ptr<hl2ss::svc::packet> packet_pv = source_pv->get_by_index(-1);
        if (packet_pv->status != 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else
        {
            uint8_t* image;
            hl2ss::pv_metadata* metadata;
            hl2ss::unpack_pv(packet_pv->payload, packet_pv->sz_payload, image, metadata);

            cv::Mat mat_image = cv::Mat(cfg_pv.height, cfg_pv.width, CV_8UC3, image);
            cv::imshow("Video", mat_image);
            if ((cv::waitKey(1) & 0xFF) == 27) { break; }
        }
    }

    hl2ss::svc::stop_subsystem_pv(host, hl2ss::stream_port::PERSONAL_VIDEO);
}


int main()
{
    try
    {
        hl2ss::ulm::initialize();
        test_sources("192.168.1.7");
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    //test_ipc("192.168.1.7");
    return 0;
}
