
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
        hl2ss::depth_profile::ZDEPTH,
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

    hl2ss::start_subsystem_pv("192.168.1.7", hl2ss::stream_port::PERSONAL_VIDEO, true, true, false, false, false, false, 0.9f, 0.0f, 0.0f, 0, hl2ss::hologram_perspective::PV);

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

void test_vi()
{
    hl2ss::ipc_vi client("192.168.1.7", hl2ss::ipc_port::VOICE_INPUT);
    std::vector<std::u16string> commands;
    std::vector<hl2ss::vi_result> results;
    bool run = true;

    commands.push_back(u"cat");
    commands.push_back(u"dog");

    client.open();
    client.create_recognizer();
    bool status = client.register_commands(true, commands);
    std::cout << "REGISTER COMMANDS " << status << std::endl;
    client.start();
    while (run)
    {
        client.pop(results);
        for (size_t i = 0; i < results.size(); ++i)
        {
            std::cout << "EVENT" << " Index: "      << results[i].index 
                                 << " Confidence: " << results[i].confidence 
                                 << " Start: " << results[i].phrase_start_time
                                 << " Duration: " << results[i].phrase_duration
                                 << " Confidence: " << results[i].raw_confidence << std::endl;
            if (results[i].index == 1) { run = false; }
        }
    }
    client.stop();
    client.clear();
    client.close();
}

void test_rc()
{
    hl2ss::ipc_rc client("192.168.1.7", hl2ss::ipc_port::REMOTE_CONFIGURATION);
    client.open();
    hl2ss::version v = client.get_application_version();
    std::cout << "VERSION: " << v.field[0] << "." << v.field[1] << "." << v.field[2] << "." << v.field[3] << std::endl;
    uint64_t offset = client.get_utc_offset(32);
    std::cout << "UTC OFFSET: " << offset << std::endl;
    client.close();
}

void test_sm()
{
    hl2ss::ipc_sm client("192.168.1.7", hl2ss::ipc_port::SPATIAL_MAPPING);
    hl2ss::sm_bounding_volume volumes;
    std::vector<hl2ss::sm_surface_info> surfaces;
    hl2ss::sm_mesh_task task;
    std::vector<hl2ss::sm_mesh> meshes;

    volumes.add_box({0.0f, 0.0f, 0.0f}, {5.0f, 5.0f, 5.0f});
    client.open();
    client.create_observer();
    client.set_volumes(volumes);
    client.get_observed_surfaces(surfaces);
    for (size_t i = 0; i < surfaces.size(); ++i)
    {
    task.add_task(surfaces[i].id, 1000.0, hl2ss::sm_vertex_position_format::R32G32B32A32Float, hl2ss::sm_triangle_index_format::R32Uint, hl2ss::sm_vertex_normal_format::R32G32B32A32Float, true, false);
    std::cout << "SURFACE " << i << ": " << surfaces[i].update_time << std::endl;
    }
    client.get_meshes(task, 2, meshes);
    client.close();

    std::cout << "Observed surfaces: " << surfaces.size() << std::endl;
    std::cout << "Meshes: " << meshes.size() << std::endl;
}

void test_su()
{
    hl2ss::ipc_su client("192.168.1.7", hl2ss::ipc_port::SCENE_UNDERSTANDING);
    hl2ss::su_task task;
    hl2ss::su_result result;

    task.enable_quads = true;
    task.enable_meshes = true;
    task.enable_only_observed = true;
    task.enable_world_mesh = true;
    task.mesh_lod = hl2ss::su_mesh_lod::Medium;
    task.query_radius = 5.0f;
    task.create_mode = hl2ss::su_create::New;
    task.kind_flags = 0xFF;
    task.get_orientation = true;
    task.get_position = true;;
    task.get_location_matrix = true;;
    task.get_quad = true;;
    task.get_meshes = true;; 
    task.get_collider_meshes = true;;

    std::cout << "SU" << std::endl;

    client.open();
    client.query(task, result);
    client.close();

    std::cout << "Status: " << result.status << std::endl;
    std::cout << "Items: " << result.items.size() << std::endl;
    std::cout << "Meshes in 0: " << result.items[0].meshes.size() << std::endl;
}

void test_umq()
{
    hl2ss::ipc_umq client("192.168.1.7", hl2ss::ipc_port::UNITY_MESSAGE_QUEUE);
    hl2ss::umq_command_buffer buffer;
    
    uint32_t type = 3;
    uint32_t mode = 1;
    uint32_t key = 0;
    uint32_t active = 1;
    std::vector<uint8_t> data;
    hl2ss::vector_3 position{0.0f, 0.0f, 0.0f};
    hl2ss::quaternion orientation{0.0f, 0.0f, 0.0f, 1.0f};
    hl2ss::vector_3 scale{0.2f, 0.2f, 0.2f};

    buffer.add( 0, &type, sizeof(type));
    buffer.add(20, &mode, sizeof(mode));
    data.clear();
    data.insert(data.end(), (uint8_t*)&key, ((uint8_t*)&key) + sizeof(key));
    data.insert(data.end(), (uint8_t*)&position, ((uint8_t*)&position) + sizeof(position));
    data.insert(data.end(), (uint8_t*)&orientation, ((uint8_t*)&orientation) + sizeof(orientation));
    data.insert(data.end(), (uint8_t*)&scale, ((uint8_t*)&scale) + sizeof(scale));
    buffer.add( 2, data.data(), data.size());
    data.clear();
    data.insert(data.end(), (uint8_t*)&key, ((uint8_t*)&key) + sizeof(key));
    data.insert(data.end(), (uint8_t*)&active, ((uint8_t*)&active) + sizeof(active));
    buffer.add( 1, data.data(), data.size());

    client.open();
    client.push(buffer.data(), buffer.size());
    std::vector<uint32_t> response;
    response.resize(buffer.count());
    client.pull(response.data(), buffer.count());
    client.close();
}

int main()
{
    try
    {
        hl2ss::client::initialize();
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
        case 7: test_eet(); break;
        case 8: test_rc(); break;
        case 9: test_sm(); break;
        case 10: test_su(); break;
        case 11: test_vi(); break;
        case 12: test_umq(); break;
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
