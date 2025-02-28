
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/highgui.hpp>
#include "hl2ss_ulm.h"

void sleep(uint32_t ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void print(hl2ss::rm_vlc_metadata* metadata)
{
    if (!metadata) { return; }
    std::cout << "sensor_ticks: " << metadata->sensor_ticks << std::endl;
    std::cout << "exposure: " << metadata->exposure << std::endl;
    std::cout << "gain: " << metadata->gain << std::endl;
}

void print(hl2ss::rm_depth_ahat_metadata* metadata)
{
    if (!metadata) { return; }
    std::cout << "sensor_ticks: " << metadata->sensor_ticks << std::endl;
}

void print(hl2ss::rm_depth_longthrow_metadata* metadata)
{
    if (!metadata) { return; }
    std::cout << "sensor_ticks: " << metadata->sensor_ticks << std::endl;
}

template<typename T>
void print(char const* name, T* v, int count)
{
    std::cout << name << ": [";
    for (int i = 0; i < count; ++i) { std::cout << v[i] << ((i < (count -1)) ? ", " : ""); }
    std::cout << "]" << std::endl;
}

template<typename T>
void print(char const* name, T* m, int rows, int cols)
{
    std::cout << name << std::endl;
    std::cout << "[" << std::endl;
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j) { std::cout << *(m + (i * cols) + j)  << ((j < (cols - 1)) ? ", " : ""); }
        std::cout << ";" << std::endl;
    }
    std::cout << "]" << std::endl;
}

void print(hl2ss::pv_metadata* metadata)
{
    if (!metadata) { return; }
    std::cout << "focal length: [" << metadata->f.x << "," << metadata->f.y << "]" << std::endl;
    std::cout << "principal point: [" << metadata->f.x << "," << metadata->f.y << "]" << std::endl;
    std::cout << "exposure_time: " << metadata->exposure_time << std::endl;
    std::cout << "exposure_compentation: [" << metadata->exposure_compensation.val[0] << "," << metadata->exposure_compensation.val[1] << "]" << std::endl;
    std::cout << "lens_position: " << metadata->lens_position << std::endl;
    std::cout << "focus_state: " << metadata->focus_state << std::endl;
    std::cout << "iso_speed: " << metadata->iso_speed << std::endl;
    std::cout << "white_balance: " << metadata->white_balance << std::endl;
    std::cout << "iso_gains: [" << metadata->iso_gains.x << "," << metadata->iso_gains.y << "]" << std::endl;
    std::cout << "white_balance_gains: [" << metadata->white_balance_gains.x << "," << metadata->white_balance_gains.y << "," << metadata->white_balance_gains.z << "]" << std::endl;
    std::cout << "resolution: " << metadata->width << "x" << metadata->height << std::endl;
}

void print(hl2ss::rm_imu_sample* sample)
{
    std::cout << "sensor_timestamp=" << sample->sensor_timestamp << " timestamp=" << sample->timestamp << " x=" << sample->x << " y=" << sample->y << " z=" << sample->z << " temperature=" << sample->temperature << std::endl;
}

void print(char const* name, hl2ss::matrix_4x4* m)
{
    if (!m) { return; }
    std::cout << name << std::endl;
    std::cout << "[" << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j) { std::cout << m->m[i][j] << ((j < 3) ? ", " : ""); }
        std::cout << ";" << std::endl;
    }
    std::cout << "]" << std::endl;
}

void print(hl2ss::extended_depth_metadata* metadata)
{
    std::cout << "embedded dimensions: " << metadata->width << "x" << metadata->height << std::endl;
}

void test_rm_vlc(char const* host)
{
    uint16_t port = hl2ss::stream_port::RM_VLC_LEFTFRONT;
    uint64_t buffer_size = 300;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_vlc>();
    auto calibration = hl2ss::svc::download_calibration<hl2ss::calibration_rm_vlc>(host, port, &configuration);
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);    

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000 / hl2ss::parameters_rm_vlc::FPS);
            continue;
        }

        auto region = data->unpack<hl2ss::map_rm_vlc>();
        
        cv::Mat image = cv::Mat(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH, CV_8UC1, region.image);
        cv::imshow(window_name, image);

        std::cout << "timestamp: " << data->timestamp << std::endl;
        print(region.metadata);
        print("pose", data->pose);
    }
}

void test_rm_depth_ahat(char const* host)
{
    uint16_t port = hl2ss::stream_port::RM_DEPTH_AHAT;
    uint64_t buffer_size = 450;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_depth_ahat>();
    auto calibration = hl2ss::svc::download_calibration<hl2ss::calibration_rm_depth_ahat>(host, port, &configuration);
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name_base = hl2ss::get_port_name(port);
    std::string window_name_depth = window_name_base + "-depth";
    std::string window_name_ab = window_name_base + "-ab";

    cv::namedWindow(window_name_depth);
    cv::namedWindow(window_name_ab);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000 / hl2ss::parameters_rm_depth_ahat::FPS);
            continue;
        }

        auto region = data->unpack<hl2ss::map_rm_depth_ahat>();

        cv::Mat depth = cv::Mat(hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH, CV_16UC1, region.depth);
        cv::Mat ab = cv::Mat(hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH, CV_16UC1, region.ab);

        cv::imshow(window_name_depth, depth * 32);
        cv::imshow(window_name_ab, ab * 8);

        std::cout << "timestamp: " << data->timestamp << std::endl;
        print(region.metadata);
        print("pose", data->pose);
    }
}

void test_rm_depth_longthrow(char const* host)
{
    uint16_t port = hl2ss::stream_port::RM_DEPTH_LONGTHROW;
    uint64_t buffer_size = 50;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_depth_longthrow>();
    auto calibration = hl2ss::svc::download_calibration<hl2ss::calibration_rm_depth_longthrow>(host, port, &configuration);
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name_base = hl2ss::get_port_name(port);
    std::string window_name_depth = window_name_base + "-depth";
    std::string window_name_ab = window_name_base + "-ab";

    cv::namedWindow(window_name_depth);
    cv::namedWindow(window_name_ab);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000 / hl2ss::parameters_rm_depth_longthrow::FPS);
            continue;
        }

        auto region = data->unpack<hl2ss::map_rm_depth_longthrow>();

        cv::Mat depth = cv::Mat(hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH, CV_16UC1, region.depth);
        cv::Mat ab = cv::Mat(hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH, CV_16UC1, region.ab);

        cv::imshow(window_name_depth, depth * 4);
        cv::imshow(window_name_ab, ab * 8);

        std::cout << "timestamp: " << data->timestamp << std::endl;
        print(region.metadata);
        print("pose", data->pose);
    }
}

void test_rm_imu(char const* host)
{
    uint16_t port = hl2ss::stream_port::RM_IMU_ACCELEROMETER;
    uint64_t buffer_size = 100;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_imu>();
    try
    {
        auto calibration = hl2ss::svc::download_calibration<hl2ss::calibration_rm_imu>(host, port, &configuration);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    cv::namedWindow(hl2ss::get_port_name(port));

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_rm_imu>();

        std::cout << "timestamp: " << data->timestamp << std::endl;
        std::cout << "received " << (data->sz_payload / sizeof(hl2ss::rm_imu_sample)) << " samples, first sample is" << std::endl;
        print(&region.samples[0]);
        print("pose", data->pose);
    }
}

void test_pv(char const* host)
{
    uint16_t port = hl2ss::stream_port::PERSONAL_VIDEO;
    uint64_t buffer_size = 300;

    auto configuration_subsystem = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv_subsystem>();
    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv>();

    configuration.width = 1920;
    configuration.height = 1080;
    configuration.framerate = 30;
    configuration.decoded_format = hl2ss::pv_decoded_format::BGR;

    int cv_type;
    switch (configuration.decoded_format)
    {
    case hl2ss::pv_decoded_format::BGR:  cv_type = CV_8UC3; break;
    case hl2ss::pv_decoded_format::RGB:  cv_type = CV_8UC3; break;
    case hl2ss::pv_decoded_format::BGRA: cv_type = CV_8UC4; break;
    case hl2ss::pv_decoded_format::RGBA: cv_type = CV_8UC4; break;
    case hl2ss::pv_decoded_format::GRAY: cv_type = CV_8UC1; break;
    default: throw std::runtime_error("Invalid PV decoded format");
    }

    hl2ss::svc::start_subsystem_pv(host, port, configuration_subsystem);

    auto calibration = hl2ss::svc::download_calibration<hl2ss::calibration_pv>(host, port, &configuration);
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_pv>();

        cv::Mat image = cv::Mat(configuration.height, configuration.width, cv_type, region.image);
        cv::imshow(window_name, image);

        std::cout << "timestamp: " << data->timestamp << std::endl;
        print(region.metadata);
        print("pose", data->pose);
    }

    hl2ss::svc::stop_subsystem_pv(host, port);
}

void test_microphone(char const* host)
{
    uint16_t port = hl2ss::stream_port::MICROPHONE;
    uint64_t buffer_size = 100;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_microphone>();
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_microphone_aac>();

        std::cout << "timestamp: " << data->timestamp << std::endl;
        std::cout << "samples: " << (data->sz_payload / sizeof(float)) << std::endl;
    }
}

void test_si(char const* host)
{
    uint16_t port = hl2ss::stream_port::SPATIAL_INPUT;
    uint64_t buffer_size = 300;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_si>();
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_si>();

        std::cout << "timestamp: " << data->timestamp << std::endl;
        std::cout << "valid: " << region.tracking->valid << std::endl;
        print("head_pose", (float*)&region.tracking->head_pose, sizeof(region.tracking->head_pose) / sizeof(float));
        print("eye_ray", (float*)&region.tracking->eye_ray, sizeof(region.tracking->eye_ray) / sizeof(float));
        print("left_wrist", (float*)&region.tracking->left_hand[hl2ss::si_hand_joint_kind::Wrist], sizeof(hl2ss::si_hand_joint) / sizeof(float));
        print("right_wrist", (float*)&region.tracking->left_hand[hl2ss::si_hand_joint_kind::Wrist], sizeof(hl2ss::si_hand_joint) / sizeof(float));
    }
}

void test_eet(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_EYE_TRACKER;
    uint64_t buffer_size = 900;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_eet>();

    configuration.framerate = hl2ss::eet_framerate::FPS_90;

    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_eet>();

        std::cout << "timestamp: " << data->timestamp << std::endl;
        std::cout << "valid: " << region.tracking->valid << std::endl;
        print("combined_ray", (float*)&region.tracking->combined_ray, sizeof(region.tracking->combined_ray) / sizeof(float));
        print("left_ray", (float*)&region.tracking->left_ray, sizeof(region.tracking->left_ray) / sizeof(float));
        print("right_ray", (float*)&region.tracking->right_ray, sizeof(region.tracking->right_ray) / sizeof(float));
        std::cout << "vergence_distance: " << region.tracking->vergence_distance << std::endl;
        std::cout << "left_openness: " << region.tracking->left_openness << std::endl;
        std::cout << "right_openness: " << region.tracking->right_openness << std::endl;
        print("pose", data->pose);
    }
}

void test_extended_audio(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_AUDIO;
    uint64_t buffer_size = 100;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_extended_audio>();
    auto device_list = hl2ss::svc::download_device_list(host, port);
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_microphone_aac>();

        std::cout << "timestamp: " << data->timestamp << std::endl;
        std::cout << "samples: " << (data->sz_payload / sizeof(float)) << std::endl;
    }
}

void test_extended_video(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_VIDEO;
    uint64_t buffer_size = 100;
    float group_index = 0;
    float source_index = 2;
    float profile_index = 4;

    auto configuration_subsystem = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv_subsystem>();

    configuration_subsystem.global_opacity = group_index;
    configuration_subsystem.output_width = source_index;
    configuration_subsystem.output_height = profile_index;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv>();

    configuration.width = 1280;
    configuration.height = 720;
    configuration.framerate = 30;
    configuration.decoded_format = hl2ss::pv_decoded_format::BGR;

    int cv_type;
    switch (configuration.decoded_format)
    {
    case hl2ss::pv_decoded_format::BGR:  cv_type = CV_8UC3; break;
    case hl2ss::pv_decoded_format::RGB:  cv_type = CV_8UC3; break;
    case hl2ss::pv_decoded_format::BGRA: cv_type = CV_8UC4; break;
    case hl2ss::pv_decoded_format::RGBA: cv_type = CV_8UC4; break;
    case hl2ss::pv_decoded_format::GRAY: cv_type = CV_8UC1; break;
    default: throw std::runtime_error("Invalid PV decoded format");
    }

    hl2ss::svc::start_subsystem_pv(host, port, configuration_subsystem);

    auto device_list = hl2ss::svc::download_device_list(host, port);
    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_pv>();

        cv::Mat image = cv::Mat(configuration.height, configuration.width, cv_type, region.image);
        cv::imshow(window_name, image);

        std::cout << "timestamp: " << data->timestamp << std::endl;
    }

    hl2ss::svc::stop_subsystem_pv(host, port);
}

void test_pv_shared(char const* host)
{
    uint16_t port = hl2ss::stream_port::PERSONAL_VIDEO;
    uint64_t buffer_size = 300;

    auto configuration_subsystem = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv_subsystem>();

    configuration_subsystem.shared = true;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv>();

    configuration.width = 1920;
    configuration.height = 1080;
    configuration.framerate = 30;
    configuration.decoded_format = hl2ss::pv_decoded_format::BGR;

    int cv_type;
    switch (configuration.decoded_format)
    {
    case hl2ss::pv_decoded_format::BGR:  cv_type = CV_8UC3; break;
    case hl2ss::pv_decoded_format::RGB:  cv_type = CV_8UC3; break;
    case hl2ss::pv_decoded_format::BGRA: cv_type = CV_8UC4; break;
    case hl2ss::pv_decoded_format::RGBA: cv_type = CV_8UC4; break;
    case hl2ss::pv_decoded_format::GRAY: cv_type = CV_8UC1; break;
    default: throw std::runtime_error("Invalid PV decoded format");
    }

    hl2ss::svc::start_subsystem_pv(host, port, configuration_subsystem);

    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_pv>();

        source->get_pv_dimensions(configuration.width, configuration.height);
        
        cv::Mat image = cv::Mat(configuration.height, configuration.width, cv_type, region.image);
        cv::imshow(window_name, image);

        std::cout << "timestamp: " << data->timestamp << std::endl;
        print(region.metadata);
        print("pose", data->pose);
        std::cout << "dimensions: " << configuration.width << " x " << configuration.height << std::endl;
    }

    hl2ss::svc::stop_subsystem_pv(host, port);
}

void test_rc(char const* host)
{
    uint16_t port = hl2ss::ipc_port::REMOTE_CONFIGURATION;

    auto ipc = hl2ss::svc::open_ipc<hl2ss::svc::ipc_rc>(host, port);

    hl2ss::version v = ipc->get_application_version();
    std::cout << "version: " << v.field[0] << "." << v.field[1] << "." << v.field[2] << "." << v.field[3] << std::endl;
    std::cout << "utf_offset: " << ipc->get_utc_offset() << std::endl;
    ipc->set_hs_marker_state(hl2ss::hs_marker_state::Disable);
    std::cout << "pv status: " << ipc->get_pv_subsystem_status() << std::endl;
    ipc->wait_for_pv_subsystem(false);
    ipc->set_pv_focus(hl2ss::pv_focus_mode::Manual, hl2ss::pv_auto_focus_range::Normal, hl2ss::pv_manual_focus_distance::Infinity, 1000, hl2ss::pv_driver_fallback::Disable);
    ipc->set_pv_video_temporal_denoising(hl2ss::pv_video_temporal_denoising_mode::On);
    ipc->set_pv_white_balance_preset(hl2ss::pv_color_temperature_preset::Auto);
    ipc->set_pv_white_balance_value(hl2ss::pv_white_balance_value::Min);
    ipc->set_pv_exposure(hl2ss::pv_exposure_mode::Auto, hl2ss::pv_exposure_value::Min);
    ipc->set_pv_exposure_priority_video(hl2ss::pv_exposure_priority_video::Enabled);
    ipc->set_pv_iso_speed(hl2ss::pv_iso_speed_mode::Auto, hl2ss::pv_iso_speed_value::Min);
    ipc->set_pv_backlight_compensation(hl2ss::pv_backlight_compensation_state::Enable);
    ipc->set_pv_scene_mode(hl2ss::pv_capture_scene_mode::Auto);
    ipc->set_flat_mode(false);
    ipc->set_rm_eye_selection(false);
    ipc->set_pv_desired_optimization(hl2ss::pv_media_capture_optimization::LatencyThenPower);
    ipc->set_pv_primary_use(hl2ss::pv_capture_use::Video);
    ipc->set_pv_optical_image_stabilization(hl2ss::pv_optical_image_stabilization_mode::On);
    ipc->set_pv_hdr_video(hl2ss::pv_hdr_video_mode::Off);
    ipc->set_pv_regions_of_interest(true, true, true, true, true, hl2ss::pv_region_of_interest_type::Unknown, 100, 0.0, 0.0, 1.0, 1.0);
    ipc->set_interface_priority(hl2ss::stream_port::PERSONAL_VIDEO, hl2ss::interface_priority::ABOVE_NORMAL);
    ipc->set_quiet_mode(false);
}

void test_sm(char const* host)
{
    uint16_t port = hl2ss::ipc_port::SPATIAL_MAPPING;
    uint32_t threads = 2;

    hl2ss::sm_bounding_volume volumes;
    hl2ss::sm_mesh_task tasks;

    auto ipc = hl2ss::svc::open_ipc<hl2ss::svc::ipc_sm>(host, port);

    volumes.add_sphere({ 0.0f, 0.0f, 0.0f, 8.0f });

    ipc->set_volumes(volumes);

    auto surfaces = ipc->get_observed_surfaces();
    std::cout << "got " << surfaces->size << " surfaces" << std::endl;

    for (uint64_t i = 0; i < surfaces->size; ++i) { tasks.add_task(surfaces->data[i].id, 1000.0, hl2ss::sm_vertex_position_format::R32G32B32A32Float, hl2ss::sm_triangle_index_format::R32Uint, hl2ss::sm_vertex_normal_format::R32G32B32A32Float, true, false); }
    auto data = ipc->get_meshes(tasks);

    int index = 0;
    for (auto const& mesh : data->meshes)
    {
        std::cout << "mesh_index: " << index++ << std::endl;
        std::cout << "mesh_status: " << mesh.status << std::endl;
        std::cout << "mesh_vertices: " << (mesh.vertex_positions_size / (4 * sizeof(float))) << std::endl;
        std::cout << "mesh_triangles: " << (mesh.triangle_indices_size / (3 * sizeof(uint32_t))) << std::endl;
        std::cout << "mesh_normals: " << (mesh.vertex_normals_size / (4 * sizeof(float))) << std::endl;
        print("vertex scale", (float*)&mesh.vertex_position_scale, sizeof(mesh.vertex_position_scale) / sizeof(float));
        print("pose", mesh.pose);
    }
}

void test_su(char const* host)
{
    uint16_t port = hl2ss::ipc_port::SCENE_UNDERSTANDING;

    auto ipc = hl2ss::svc::open_ipc<hl2ss::svc::ipc_su>(host, port);

    hl2ss::su_task task;

    task.enable_quads = true;
    task.enable_meshes = true;
    task.enable_only_observed = true;
    task.enable_world_mesh = true;
    task.mesh_lod = hl2ss::su_mesh_lod::Medium;
    task.query_radius = 5.0f;
    task.create_mode = hl2ss::su_create::New;
    task.kind_flags = hl2ss::su_kind_flag::Ceiling | hl2ss::su_kind_flag::Floor | hl2ss::su_kind_flag::Platform | hl2ss::su_kind_flag::Wall | hl2ss::su_kind_flag::World;
    task.get_orientation = true;
    task.get_position = true;
    task.get_location_matrix = true;
    task.get_quad = true;
    task.get_meshes = true;
    task.get_collider_meshes = true;

    auto result = ipc->query(task);

    if (result->status != 0)
    {
        std::cout << "su query failed" << std::endl;
        return;
    }

    std::cout << "got " << result->count << " objects" << std::endl;
    print("extrinsics", result->extrinsics);
    print("pose", result->pose);

    int index = 0;
    for (auto const& item : result->items)
    {
        std::cout << "item_index: " << index++ << std::endl;
        std::cout << "item_kind: " << item.kind << std::endl;
        std::cout << "item_id: (" << item.id.h << "," << item.id.l << ")" << std::endl;
        std::cout << "item_alignment: " << item.alignment << std::endl;
        std::cout << "item_meshes_count: " << item.meshes_count << std::endl;
        std::cout << "item_collider_meshes_count: " << item.collider_meshes_count << std::endl;
        print("item_location", item.location);
        for (auto const& mesh : item.unpacked_meshes)
        {
            std::cout << "mesh_info" << std::endl;
            std::cout << "vertices: " << (mesh.vertex_positions_size / (3 * sizeof(float))) << std::endl;
            std::cout << "triangles: " << (mesh.triangle_indices_size / (3 * sizeof(uint32_t))) << std::endl;
        }
        for (auto const& mesh : item.unpacked_collider_meshes)
        {
            std::cout << "collider_mesh_info" << std::endl;
            std::cout << "vertices: " << (mesh.vertex_positions_size / (3 * sizeof(float))) << std::endl;
            std::cout << "triangles: " << (mesh.triangle_indices_size / (3 * sizeof(uint32_t))) << std::endl;
        }
    }
}

void test_vi(char const* host)
{
    uint16_t port = hl2ss::ipc_port::VOICE_INPUT;

    auto ipc = hl2ss::svc::open_ipc<hl2ss::svc::ipc_vi>(host, port);

    ipc->start("cat\0dog\0red\0blue\0");

    std::cout << "waiting for voice command..." << std::endl;
    while (true)
    {
        auto result = ipc->pop();
        if (result->size < 1) { continue; }
        std::cout << "index: " << result->data[0].index << std::endl;
        std::cout << "confidence: " << result->data[0].confidence << std::endl;
        std::cout << "phrase_start_time: " << result->data[0].phrase_start_time << std::endl;
        std::cout << "phrase_duration: " << result->data[0].phrase_duration << std::endl;
        std::cout << "raw_confidence: " << result->data[0].raw_confidence << std::endl;
        break;
    }

    ipc->stop();
}

void test_umq(char const* host)
{
    uint16_t port = hl2ss::ipc_port::UNITY_MESSAGE_QUEUE;

    auto ipc = hl2ss::svc::open_ipc<hl2ss::svc::ipc_umq>(host, port);

    auto buffer = hl2ss::umq_command_buffer();
    char const text[] = "Hello";
    uint32_t response[1];

    buffer.add(0xFFFFFFFE, text, sizeof(text));

    ipc->push(buffer.get_data(), buffer.get_size());
    ipc->pull(response, sizeof(response) / sizeof(uint32_t));

    std::cout << "response: " << response[0] << std::endl;
}

void test_gmq(char const* host)
{
    uint16_t port = hl2ss::ipc_port::GUEST_MESSAGE_QUEUE;

    auto ipc = hl2ss::svc::open_ipc<hl2ss::svc::ipc_gmq>(host, port);

    while (true)
    {
        auto msg = ipc->pull();
        switch (msg->command)
        {
        case 0xFFFFFFFE:
        {
            std::string text = std::string(msg->data, msg->data + msg->size);
            std::cout << text << std::endl;
            break;
        }
        case 0xFFFFFFFF:
            continue;
        default:
            std::cout << "id=" << msg->command << " size=" << msg->size << std::endl;
        }
        break;
    }

    uint32_t response[] = { 1 };
    ipc->push(response, sizeof(response) / sizeof(uint32_t));
}

void test_extended_depth(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_DEPTH;
    float group_index = 0;
    float source_index = 0;
    float profile_index = 0;
    uint64_t media_index = 15;
    uint64_t buffer_size = 300;

    auto configuration_subsystem = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv_subsystem>();

    configuration_subsystem.global_opacity = group_index;
    configuration_subsystem.output_width = source_index;
    configuration_subsystem.output_height = profile_index;

    auto configuration = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_extended_depth>();

    configuration.media_index = 15;

    hl2ss::svc::start_subsystem_pv(host, port, configuration_subsystem);

    auto source = hl2ss::svc::open_stream(host, port, buffer_size, &configuration);

    std::string window_name = hl2ss::get_port_name(port);

    cv::namedWindow(window_name);

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        auto data = source->get_by_index(-1);
        if (data->status != 0)
        {
            sleep(1000);
            continue;
        }

        auto region = data->unpack<hl2ss::map_extended_depth>();

        uint16_t width;
        uint16_t height;
        source->get_pv_dimensions(width, height);
        
        cv::Mat image = cv::Mat(height, width, CV_16UC1, region.depth) * 16;
        cv::imshow(window_name, image);

        std::cout << "timestamp: " << data->timestamp << std::endl;
        print(region.metadata);
        print("pose", data->pose);
        std::cout << "dimensions: " << width << " x " << height << std::endl;
    }

    hl2ss::svc::stop_subsystem_pv(host, port);
}

int main()
{
    char const* host = "192.168.1.7";
    int test_id = 10;

    try
    {
        hl2ss::svc::initialize();

        switch (test_id)
        {
        case 0: test_rm_vlc(host); break;
        case 1: test_rm_depth_ahat(host); break;
        case 2: test_rm_depth_longthrow(host); break;
        case 3: test_rm_imu(host); break;
        case 4: test_pv(host); break;
        case 5: test_microphone(host); break;
        case 6: test_si(host); break;
        case 7: test_eet(host); break;
        case 8: test_extended_audio(host); break;
        case 9: test_extended_video(host); break;
        case 10: test_rc(host); break;
        case 11: test_sm(host); break;
        case 12: test_su(host); break;
        case 13: test_vi(host); break;
        case 14: test_umq(host); break;
        case 15: test_gmq(host); break;
        case 16: test_pv_shared(host); break;
        case 17: test_extended_depth(host); break;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
