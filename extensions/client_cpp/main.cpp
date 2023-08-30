
#include <iostream>
#include <opencv2/highgui.hpp>

#include "hl2ss.h"

void test_rm_depth_ahat()
{
    std::vector<uint64_t> options;
    options.push_back(hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize);
    options.push_back(45);

    hl2ss::rx_decoded_rm_depth_ahat client(
        "192.168.1.7",
        hl2ss::stream_port::RM_DEPTH_AHAT,
        4096,
        hl2ss::stream_mode::MODE_1,
        1, 
        hl2ss::depth_profile::SAME,
        hl2ss::video_profile::H265_MAIN,
        hl2ss::h26x_level::DEFAULT,
        8*1024*1024,
        options
    );

    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        cv::Mat depth = cv::Mat(512, 512, CV_16UC1, data->payload.get());
        cv::Mat ab = cv::Mat(512, 512, CV_16UC1, data->payload.get() + (512*512*2));
        cv::imshow("depth", depth * 16);
        cv::imshow("ab", ab);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client.close();
}

void test_rm_vlc()
{
    std::vector<uint64_t> options;
    options.push_back(hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize);
    options.push_back(30);

    hl2ss::rx_decoded_rm_vlc client(
        "192.168.1.7",
        hl2ss::stream_port::RM_VLC_LEFTFRONT, 
        4096, 
        hl2ss::stream_mode::MODE_1, 
        1, 
        hl2ss::video_profile::H265_MAIN, 
        hl2ss::h26x_level::DEFAULT, 
        1*1024*1024, 
        options);

    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        cv::Mat image = cv::Mat(480, 640, CV_8UC1, data->payload.get());
        cv::imshow("VLC", image);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client.close();
}

void test_rm_depth_longthrow()
{
    hl2ss::rx_decoded_rm_depth_longthrow client(
        "192.168.1.7",
        hl2ss::stream_port::RM_DEPTH_LONGTHROW,
        4096, hl2ss::stream_mode::MODE_1,
        1,
        hl2ss::png_filter_mode::PAETH
        );

    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        cv::Mat depth = cv::Mat(288, 320, CV_16UC1, data->payload.get());
        cv::Mat ab = cv::Mat(288, 320, CV_16UC1, data->payload.get() + hl2ss::parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t));
        cv::imshow("Test DEPTH", depth * 8);
        //cv::imshow("Test AB", ab);
        std::cout << "LT: " << data->timestamp << std::endl;
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client.close();
}

void test_rm_imu()
{
    hl2ss::rx_rm_imu client = hl2ss::rx_rm_imu("192.168.1.7", hl2ss::stream_port::RM_IMU_ACCELEROMETER, 4096, hl2ss::stream_mode::MODE_1);
    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        std::cout << "Timestamp: " << data->timestamp << std::endl;
    }
    client.close();
}

void test_pv()
{
    std::vector<uint64_t> options;
    options.push_back(hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize);
    options.push_back(30);

    hl2ss::start_subsystem_pv("192.168.1.7", hl2ss::stream_port::PERSONAL_VIDEO);

    hl2ss::rx_decoded_pv client(
        "192.168.1.7",
        hl2ss::stream_port::PERSONAL_VIDEO,
        4096,
        hl2ss::stream_mode::MODE_1,
        640,
        360,
        30,
        1,
        hl2ss::video_profile::H265_MAIN,
        hl2ss::h26x_level::DEFAULT,
        2*1024*1024,
        options,
        hl2ss::pv_decoded_format::BGR
    );

    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        cv::Mat image = cv::Mat(360, 640, CV_8UC3, data->payload.get());
        cv::imshow("TEST BGR", image);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client.close();
}

void test_microphone()
{
    hl2ss::rx_decoded_microphone client(
        "192.168.1.7",
        hl2ss::stream_port::MICROPHONE,
        hl2ss::chunk_size::MICROPHONE,
        hl2ss::audio_profile::AAC_24000,
        hl2ss::aac_level::L2
    );

    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        std::cout << "TS: " << data->timestamp << std::endl;
    }
    client.close();
}

void test_si()
{
    hl2ss::rx_si client(
        "192.168.1.7",
        hl2ss::stream_port::SPATIAL_INPUT,
        hl2ss::chunk_size::SPATIAL_INPUT
    );

    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        std::cout << "TS: " << data->timestamp << std::endl;
    }
    client.close();
}

void test_eet()
{
    hl2ss::rx_eet client(
        "192.168.1.7",
        hl2ss::stream_port::EXTENDED_EYE_TRACKER,
        hl2ss::chunk_size::EXTENDED_EYE_TRACKER,
        hl2ss::eet_framerate::FPS_30
    );

    client.open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();
        std::cout << "TS: " << data->timestamp << std::endl;
    }
    client.close();
}


//hl2ss::frame* frame = decoder.decode(data->payload, data->sz_payload);

        //cv::Mat bgr = hl2ss::frame_to_opencv_mat(*(frame->f));
        //cv::Mat bgr = cv::Mat(480, 640, CV_8UC1, data->payload);
        

        //cv::Mat bgr = cv::Mat(client.height, client.width, CV_8UC3, data->payload);
        //
        
        //cv::waitKey(1);

        //frame->Release();
        //std::cout << "PAYLOAD SIZE: " << data->sz_payload << std::endl;

int main()
{

    //hl2ss::rx_decoded_rm_vlc client("192.168.1.7", 3800, 4096, 1, 1, 3, 0xFF, 1*1024*1024, options);
    //hl2ss::rx_rm_depth_ahat client("192.168.1.7", 3804, 4096, 1, 1, 0, 3, 0xFF, 8*1024*1024, options);
    //hl2ss::rx_decoded_rm_depth_longthrow client("192.168.1.7", 3805, 4096, 1, 1, 5);
    //hl2ss::rx_rm_imu client("192.168.1.7", 3806, 4096, 1);
    //hl2ss::start_subsystem_pv("192.168.1.7", 3810);
    //hl2ss::rx_decoded_pv client("192.168.1.7", 3810, 4096, 1, 1920, 1080, 30, 1, 0, 0xFF, 5*1024*1024, options, 0);
    //hl2ss::rx_microphone client("192.168.1.7", 3811, 4096, 3, 0x29);
    //hl2ss::rx_si client("192.168.1.7", 3812, 4096);
    //hl2ss::rx_eet client("192.168.1.7", 3817, 4096, 30);
    //hl2ss::rx_decoded_microphone client("192.168.1.7", hl2ss::stream_port::MICROPHONE, 4096, hl2ss::audio_profile::AAC_24000, hl2ss::aac_level::L2);

    /*
    hl2ss::rx_rm_depth_ahat 
    client(
        "192.168.1.7",
        hl2ss::stream_port::RM_DEPTH_AHAT,
        4096,
        hl2ss::stream_mode::MODE_1,
        1, 
        hl2ss::depth_profile::SAME,
        hl2ss::video_profile::H265_MAIN,
        hl2ss::h26x_level::DEFAULT,
        8*1024*1024,
        options
    );
    */
    //

    try
    {
        int test_id = 4;

        switch (test_id)
        {
        case 0: test_rm_vlc(); break;
        case 1: test_rm_depth_ahat(); break;
        case 2: test_rm_depth_longthrow(); break;
        case 3: test_rm_imu(); break;
        case 4: test_pv(); break;
        case 5: test_microphone(); break;
        case 6: test_si(); break;
        case 7: test_eet();
        }
        /*
        std::shared_ptr<hl2ss::calibration_rm_vlc> data1 = hl2ss::download_calibration_rm_vlc("192.168.1.7", hl2ss::stream_port::RM_VLC_LEFTFRONT);
        std::cout << "1) " << data1->intrinsics[3] << std::endl;

        std::shared_ptr<hl2ss::calibration_rm_depth_ahat> data2 = hl2ss::download_calibration_rm_depth_ahat("192.168.1.7", hl2ss::stream_port::RM_DEPTH_AHAT);
        std::cout << "2) " << data2->intrinsics[3] << std::endl;

        std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> data3 = hl2ss::download_calibration_rm_depth_longthrow("192.168.1.7", hl2ss::stream_port::RM_DEPTH_LONGTHROW);
        std::cout << "3) " << data3->intrinsics[3] << std::endl;

        std::shared_ptr<hl2ss::calibration_rm_imu> data4 = hl2ss::download_calibration_rm_imu("192.168.1.7", hl2ss::stream_port::RM_IMU_ACCELEROMETER);
        std::cout << "4) " << data4->extrinsics[3][3] << std::endl;

        hl2ss::start_subsystem_pv("192.168.1.7", hl2ss::stream_port::PERSONAL_VIDEO);

        std::shared_ptr<hl2ss::calibration_pv> data5 = hl2ss::download_calibration_pv("192.168.1.7", hl2ss::stream_port::PERSONAL_VIDEO, 1920, 1080, 30);
        std::cout << "5) " << data5->projection[3][3] << std::endl;
        */
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }   
    
    return 0;
}
