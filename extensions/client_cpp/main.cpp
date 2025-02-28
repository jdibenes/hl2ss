
#include <iostream>
#include <opencv2/highgui.hpp>
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"

//-----------------------------------------------------------------------------
// Common
//-----------------------------------------------------------------------------

void print_matrix(hl2ss::matrix_4x4* matrix)
{
    std::cout << "[" << std::endl;
    for (int row = 0; row < 4; ++row)
    {
    for (int col = 0; col < 4; ++col)
    {
    std::cout << matrix->m[row][col] << ", ";
    }
    std::cout << std::endl;
    }
    std::cout << "]" << std::endl;
}

void print_packet_metadata(uint64_t timestamp, hl2ss::matrix_4x4* matrix)
{
    std::cout << "Pose at time " << timestamp << std::endl;
    if (matrix)
    {
    print_matrix(matrix);
    }
    else
    {
    std::cout << "None" << std::endl;
    }
}

//-----------------------------------------------------------------------------
// RM VLC
//-----------------------------------------------------------------------------

void test_rm_vlc(char const* host, uint16_t port)
{
    std::unique_ptr<hl2ss::rx_rm_vlc> client = hl2ss::lnm::rx_rm_vlc(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    std::cout << "Downloading calibration for " << port_name << " ..." << std::endl;
    std::shared_ptr<hl2ss::calibration_rm_vlc> calibration = hl2ss::lnm::download_calibration_rm_vlc(host, port);
    std::cout << "Done." << std::endl;

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_rm_vlc region = hl2ss::unpack_rm_vlc(data->payload.get());

        print_packet_metadata(data->timestamp, data->pose.get());
        std::cout << "Sensor Ticks: " << region.metadata->sensor_ticks << std::endl;
        std::cout << "Exposure: " << region.metadata->exposure << std::endl;
        std::cout << "Gain: " << region.metadata->gain << std::endl;

        cv::Mat mat_image = cv::Mat(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH, CV_8UC1, region.image);
        cv::imshow(port_name, mat_image);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// RM Depth AHAT
//-----------------------------------------------------------------------------

void test_rm_depth_ahat(char const* host)
{
    uint16_t port = hl2ss::stream_port::RM_DEPTH_AHAT;
    std::unique_ptr<hl2ss::rx_rm_depth_ahat> client = hl2ss::lnm::rx_rm_depth_ahat(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    std::cout << "Downloading calibration for " << port_name << " ..." << std::endl;
    std::shared_ptr<hl2ss::calibration_rm_depth_ahat> calibration = hl2ss::lnm::download_calibration_rm_depth_ahat(host, port);
    std::cout << "Done." << std::endl;

    std::string name_depth = port_name + "-depth";
    std::string name_ab    = port_name + "-ab";

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_rm_depth_ahat region = hl2ss::unpack_rm_depth_ahat(data->payload.get());

        print_packet_metadata(data->timestamp, data->pose.get());
        std::cout << "Sensor Ticks: " << region.metadata->sensor_ticks << std::endl;

        cv::Mat mat_depth = cv::Mat(hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH, CV_16UC1, region.depth);
        cv::Mat mat_ab    = cv::Mat(hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::WIDTH, CV_16UC1, region.ab);
        cv::imshow(name_depth, mat_depth * 16); // scaled for visibility
        cv::imshow(name_ab,    mat_ab);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// RM Depth Longthrow
//-----------------------------------------------------------------------------

void test_rm_depth_longthrow(char const* host)
{
    uint16_t port = hl2ss::stream_port::RM_DEPTH_LONGTHROW;
    std::unique_ptr<hl2ss::rx_rm_depth_longthrow> client = hl2ss::lnm::rx_rm_depth_longthrow(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    std::cout << "Downloading calibration for " << port_name << " ..." << std::endl;
    std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> calibration = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);
    std::cout << "Done." << std::endl;

    std::string name_depth = port_name + "-depth";
    std::string name_ab    = port_name + "-ab";

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_rm_depth_longthrow region = hl2ss::unpack_rm_depth_longthrow(data->payload.get());

        print_packet_metadata(data->timestamp, data->pose.get());
        std::cout << "Sensor Ticks: " << region.metadata->sensor_ticks << std::endl;

        cv::Mat mat_depth = cv::Mat(hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH, CV_16UC1, region.depth);
        cv::Mat mat_ab    = cv::Mat(hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH, CV_16UC1, region.ab);
        cv::imshow(name_depth, mat_depth * 8); // scaled for visibility
        cv::imshow(name_ab,    mat_ab);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// RM IMU
//-----------------------------------------------------------------------------

void test_rm_imu(char const* host, uint16_t port)
{
    std::unique_ptr<hl2ss::rx_rm_imu> client = hl2ss::lnm::rx_rm_imu(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    if (port != hl2ss::stream_port::RM_IMU_MAGNETOMETER)
    {
    std::cout << "Downloading calibration for " << port_name << " ..." << std::endl;
    std::shared_ptr<hl2ss::calibration_rm_imu> calibration = hl2ss::lnm::download_calibration_rm_imu(host, port);
    std::cout << "Done." << std::endl;
    }

    cv::namedWindow(port_name);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_rm_imu region = hl2ss::unpack_rm_imu(data->payload.get());

        print_packet_metadata(data->timestamp, data->pose.get());

        std::cout << "First sample: " << region.samples[0].sensor_timestamp << ", " << region.samples[0].timestamp << ", " << region.samples[0].x << ", " << region.samples[0].y << ", " << region.samples[0].z << ", " << region.samples[0].temperature << std::endl;
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// PV
//-----------------------------------------------------------------------------

void test_pv(char const* host, uint16_t width, uint16_t height, uint8_t framerate, bool enable_mrc)
{
    uint16_t port = hl2ss::stream_port::PERSONAL_VIDEO;
    std::unique_ptr<hl2ss::rx_pv> client = hl2ss::lnm::rx_pv(host, port, width, height, framerate);
    std::string port_name = hl2ss::get_port_name(port);

    hl2ss::lnm::start_subsystem_pv(host, port, enable_mrc);

    std::cout << "Downloading calibration for " << port_name << " ..." << std::endl;
    std::shared_ptr<hl2ss::calibration_pv> calibration = hl2ss::download_calibration_pv(host, port, width, height, framerate);
    std::cout << "Done." << std::endl;

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_pv region = hl2ss::unpack_pv(data->payload.get(), data->sz_payload);

        print_packet_metadata(data->timestamp, data->pose.get());

        std::cout << "Focal length: "    << region.metadata->f.x << ", " << region.metadata->f.y << std::endl;
        std::cout << "Principal point: " << region.metadata->c.x << ", " << region.metadata->c.y << std::endl;

        std::cout << "exposure_time: " << region.metadata->exposure_time << std::endl;
        std::cout << "exposure_compensation: " << region.metadata->exposure_compensation.val[0] << "," << region.metadata->exposure_compensation.val[1] << std::endl;
        std::cout << "lens_position: " << region.metadata->lens_position << std::endl;
        std::cout << "focus_state: " << region.metadata->focus_state << std::endl;
        std::cout << "iso_speed: " << region.metadata->iso_speed << std::endl;
        std::cout << "white_balance: " << region.metadata->white_balance << std::endl;
        std::cout << "iso_gains: " << region.metadata->iso_gains.x << "," << region.metadata->iso_gains.y << std::endl;
        std::cout << "white_balance_gains: " << region.metadata->white_balance_gains.x << "," << region.metadata->white_balance_gains.y << "," << region.metadata->white_balance_gains.z << std::endl;

        cv::Mat mat_image = cv::Mat(height, width, CV_8UC3, region.image);
        cv::imshow(port_name, mat_image);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();

    hl2ss::lnm::stop_subsystem_pv(host, port);
}

//-----------------------------------------------------------------------------
// Microphone
//-----------------------------------------------------------------------------

void test_microphone(char const* host)
{
    uint16_t port = hl2ss::stream_port::MICROPHONE;
    std::unique_ptr<hl2ss::rx_microphone> client = hl2ss::lnm::rx_microphone(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    cv::namedWindow(port_name);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_microphone_aac region = hl2ss::unpack_microphone_aac(data->payload.get());
        //region.samples

        print_packet_metadata(data->timestamp, data->pose.get());

        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// Spatial Input
//-----------------------------------------------------------------------------

void test_si(char const* host)
{
    uint16_t port = hl2ss::stream_port::SPATIAL_INPUT;
    std::unique_ptr<hl2ss::rx_si> client = hl2ss::lnm::rx_si(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    cv::namedWindow(port_name);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_si region = hl2ss::unpack_si(data->payload.get());

        print_packet_metadata(data->timestamp, data->pose.get());

        std::cout << "Head position: " << region.tracking->head_pose.position.x << ", " << region.tracking->head_pose.position.y << ", " << region.tracking->head_pose.position.z << std::endl;
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// Extended Eye Tracker
//-----------------------------------------------------------------------------

void test_eet(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_EYE_TRACKER;
    std::unique_ptr<hl2ss::rx_eet> client = hl2ss::lnm::rx_eet(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    cv::namedWindow(port_name);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_eet region = hl2ss::unpack_eet(data->payload.get());

        print_packet_metadata(data->timestamp, data->pose.get());

        std::cout << "Valid: " << region.tracking->valid << std::endl;
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// Extended Audio
//-----------------------------------------------------------------------------

void test_extended_audio(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_AUDIO;
    std::unique_ptr<hl2ss::rx_extended_audio> client = hl2ss::lnm::rx_extended_audio(host, port);
    std::string port_name = hl2ss::get_port_name(port);

    cv::namedWindow(port_name);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_extended_audio_aac region = hl2ss::unpack_extended_audio_aac(data->payload.get());

        print_packet_metadata(data->timestamp, data->pose.get());

        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();
}

//-----------------------------------------------------------------------------
// Remote Configuration
//-----------------------------------------------------------------------------

void test_rc(char const* host)
{
    std::unique_ptr<hl2ss::ipc_rc> client = hl2ss::lnm::ipc_rc(host, hl2ss::ipc_port::REMOTE_CONFIGURATION);
    client->open();
    hl2ss::version v = client->get_application_version();
    std::cout << "Version: " << v.field[0] << "." << v.field[1] << "." << v.field[2] << "." << v.field[3] << std::endl;
    uint64_t offset = client->get_utc_offset();
    std::cout << "UTC offset: " << offset << std::endl;
    client->close();
}

//-----------------------------------------------------------------------------
// Spatial Mapping
//-----------------------------------------------------------------------------

void test_sm(char const* host)
{
    std::unique_ptr<hl2ss::ipc_sm> client = hl2ss::lnm::ipc_sm(host, hl2ss::ipc_port::SPATIAL_MAPPING);
    hl2ss::sm_bounding_volume volumes;
    std::vector<hl2ss::sm_surface_info> surfaces;
    hl2ss::sm_mesh_task task;
    std::vector<hl2ss::sm_mesh> meshes;

    volumes.add_box({0.0f, 0.0f, 0.0f, 8.0f, 8.0f, 8.0f});
    client->open();
    client->set_volumes(volumes);
    client->get_observed_surfaces(surfaces);
    for (size_t i = 0; i < surfaces.size(); ++i)
    {
    task.add_task(surfaces[i].id, 1000.0, hl2ss::sm_vertex_position_format::R32G32B32A32Float, hl2ss::sm_triangle_index_format::R32Uint, hl2ss::sm_vertex_normal_format::R32G32B32A32Float);
    std::cout << "SURFACE " << i << ": " << surfaces[i].update_time << std::endl;
    }
    client->get_meshes(task, meshes);
    client->close();

    std::cout << "Observed surfaces: " << surfaces.size() << std::endl;
    std::cout << "Meshes: " << meshes.size() << std::endl;
}

//-----------------------------------------------------------------------------
// Scene Understanding
//-----------------------------------------------------------------------------

void test_su(char const* host)
{
    std::unique_ptr<hl2ss::ipc_su> client = hl2ss::lnm::ipc_su(host, hl2ss::ipc_port::SCENE_UNDERSTANDING);
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
    task.get_position = true;
    task.get_location_matrix = true;
    task.get_quad = true;
    task.get_meshes = true;
    task.get_collider_meshes = true;

    std::cout << "SU" << std::endl;

    client->open();
    client->query(task, result);
    client->close();

    std::cout << "Status: " << result.status << std::endl;
    std::cout << "Items: " << result.items.size() << std::endl;
    std::cout << "Meshes in 0: " << result.items[0].meshes.size() << std::endl;
}

//-----------------------------------------------------------------------------
// Voice Input
//-----------------------------------------------------------------------------

void test_vi(char const* host)
{
    std::unique_ptr<hl2ss::ipc_vi> client = hl2ss::lnm::ipc_vi(host, hl2ss::ipc_port::VOICE_INPUT);
    std::vector<std::u16string> commands;
    std::vector<hl2ss::vi_result> results;
    bool run = true;

    commands.push_back(u"cat");
    commands.push_back(u"dog");

    client->open();
    client->start(commands);
    while (run)
    {
        client->pop(results);
        for (size_t i = 0; i < results.size(); ++i)
        {
            std::cout << "[Event]" << " Index: "      << results[i].index 
                                   << " Confidence: " << results[i].confidence 
                                   << " Start: "      << results[i].phrase_start_time
                                   << " Duration: "   << results[i].phrase_duration
                                   << " Confidence: " << results[i].raw_confidence << std::endl;
            if (results[i].index == 1) { run = false; }
        }
    }
    client->stop();
    client->close();
}

//-----------------------------------------------------------------------------
// Unity Message Queue
//-----------------------------------------------------------------------------

void test_umq(char const* host)
{
    std::unique_ptr<hl2ss::ipc_umq> client = hl2ss::lnm::ipc_umq(host, hl2ss::ipc_port::UNITY_MESSAGE_QUEUE);
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

    client->open();
    client->push(buffer.get_data(), buffer.get_size());
    std::vector<uint32_t> response;
    response.resize(buffer.get_count());
    client->pull(response.data(), buffer.get_count());
    client->close();
}

//-----------------------------------------------------------------------------
// Multithreading Example
//-----------------------------------------------------------------------------

void test_mt(char const* host)
{
    // PV camera configuration
    uint16_t pv_width = 640;
    uint16_t pv_height = 360;
    uint8_t pv_fps = 30;

    // Buffer size in seconds
    uint64_t buffer_size = 10;

    // Initialize PV camera
    hl2ss::lnm::start_subsystem_pv(host, hl2ss::stream_port::PERSONAL_VIDEO);

    // Create OpenCV windows for visualization
    std::string pv_name = hl2ss::get_port_name(hl2ss::stream_port::PERSONAL_VIDEO);
    std::string lt_name = hl2ss::get_port_name(hl2ss::stream_port::RM_DEPTH_LONGTHROW);
    std::string lt_depth_name = lt_name + "-depth";
    std::string lt_ab_name = lt_name + "-ab";

    cv::namedWindow(pv_name);
    cv::namedWindow(lt_depth_name);
    cv::namedWindow(lt_ab_name);

    // Create PV and Depth clients
    std::unique_ptr<hl2ss::rx_pv> client_pv = hl2ss::lnm::rx_pv(host, hl2ss::stream_port::PERSONAL_VIDEO, pv_width, pv_height, pv_fps);
    std::unique_ptr<hl2ss::rx_rm_depth_longthrow> client_lt = hl2ss::lnm::rx_rm_depth_longthrow(host, hl2ss::stream_port::RM_DEPTH_LONGTHROW);

    // Give PV and Depth client ownership to multithreaded handler
    // This allows to receive data in a non-blocking manner and associate data from different streams
    // Client must be in closed state
    std::unique_ptr<hl2ss::mt::source> source_pv = std::make_unique<hl2ss::mt::source>(buffer_size*pv_fps, std::move(client_pv));
    std::unique_ptr<hl2ss::mt::source> source_lt = std::make_unique<hl2ss::mt::source>(buffer_size*hl2ss::parameters_rm_depth_longthrow::FPS, std::move(client_lt));

    // Start capture
    source_pv->start();
    source_lt->start();

    // Initialize PV frame index for sequential capture
    int64_t pv_frame_index = 0; 

    // Capture data until stopped (ESC pressed)
    for (;;)
    {
        // Check for errors (e.g, network error, decoding error, etc.)
        // In this example we just re-throw the internal exception
        std::exception error;
        if (!source_pv->status(error)) { throw error; }
        if (!source_lt->status(error)) { throw error; }

        // Get PV frame by index
        // Return value: 0 if frame retrieved successfully
        int32_t pv_status; 
        // Alternatively pass index -1 for most recent frame, -2 for second most recent frame, etc., will repeat/drop frames if necessary
        std::shared_ptr<hl2ss::packet> data_pv = source_pv->get_packet(pv_frame_index, pv_status);

        // wait value for cv::waitKey
        int wait_key_ms = 1;

        if (pv_status < 0) 
        {
            // Requested frame is too old and has been dropped from the buffer (data_pv is null)
            // Advance to next frame
            pv_frame_index++;
        }
        else if (pv_status == 0)
        { 
            // Frame succesfully retrieved (data_pv is not null)
            // Advance to next frame
            pv_frame_index++;

            // Unpack PV image and show
            hl2ss::map_pv region = hl2ss::unpack_pv(data_pv->payload.get(), data_pv->sz_payload);
            cv::Mat pv_mat = cv::Mat(pv_height, pv_width, CV_8UC3, region.image);
            cv::imshow(pv_name, pv_mat);

            // Get depth frame closest (in time) to the PV frame
            // Search mode:
            // NEAREST: return closest
            // PAST: return closest with timestamp <= data_pv->timestamp
            // FUTURE: return closest with timestamp >= data_pv->timestamp
            int32_t search_mode = hl2ss::mt::time_preference::PREFER_NEAREST; 
            // Choose frame with timestamp > data->timestamp if search mode is NEAREST and the two nearest frames are at the distance
            bool tiebreak_right = false;
            // Return value: frame_index of the returned depth frame
            int64_t lt_frame_index;
            // Return value: status is 0 if depth frame was retrieved successfully, < 0 if too old, > 0 if not received yet
            int32_t lt_status;
            // Get depth frame
            std::shared_ptr<hl2ss::packet> data_lt = source_lt->get_packet(data_pv->timestamp, search_mode, tiebreak_right, lt_frame_index, lt_status);

            // Check if depth frame was retrieved successfully
            if (data_lt)
            {
                // Unpack depth image and show
                hl2ss::map_rm_depth_longthrow region = hl2ss::unpack_rm_depth_longthrow(data_lt->payload.get());                
                cv::Mat lt_depth_mat = cv::Mat(hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH, CV_16UC1, region.depth);
                cv::Mat lt_ab_mat = cv::Mat(hl2ss::parameters_rm_depth_longthrow::HEIGHT, hl2ss::parameters_rm_depth_longthrow::WIDTH, CV_16UC1, region.ab);
                cv::imshow(lt_depth_name, lt_depth_mat * 8); // Scaled for visibility otherwise image will be too dark
                cv::imshow(lt_ab_name, lt_ab_mat * 4); // Scaled for visibility, might overflow
            }
        }
        else // pv_status > 0 
        {
            // Requested frame has not been received from the server yet (data_pv is null)
            // Do not advance to next frame
            // Wait 1 frame in ms
            wait_key_ms = 1000 / pv_fps;
        }

        // Stop when ESC is pressed
        if ((cv::waitKey(wait_key_ms) & 0xFF) == 27) { break; }
    }

    // Stop capture
    source_pv->stop();
    source_lt->stop();

    // Stop PV camera
    hl2ss::lnm::stop_subsystem_pv(host, hl2ss::stream_port::PERSONAL_VIDEO);
}

//-----------------------------------------------------------------------------
// Guest Message Queue
//-----------------------------------------------------------------------------

void test_gmq(char const* host)
{
    std::unique_ptr<hl2ss::ipc_gmq> client = hl2ss::lnm::ipc_gmq(host, hl2ss::ipc_port::GUEST_MESSAGE_QUEUE);
    hl2ss::gmq_message msg;

    client->open();
    
    while (true)
    {
        client->pull(msg);
        if (msg.command == ~0U) { continue; }
        uint32_t response = 1;
        client->push(&response, sizeof(response) / sizeof(uint32_t));
        break;
    }

    client->close();

    switch (msg.command)
    {
    case 0xFFFFFFFE:
        std::cout << msg.data.get() << std::endl;
        break;
    default:
        std::cout << "Received command id=" << msg.command << std::endl;
    }
}

//-----------------------------------------------------------------------------
// PV + UMQ Example
//-----------------------------------------------------------------------------

void test_pv_umq(char const* host)
{
    // Camera parameters
    uint16_t const pv_width = 640;
    uint16_t const pv_height = 360;
    uint8_t const pv_fps = 30;

    // Buffer size in seconds
    int64_t const buffer_size = 5;

    // Test message to send to the HoloLens app
    uint32_t const command_id = 0xFFFFFFFE; // Output to debugger
    char const command_data[] = "C++ client test message"; // UTF-8 string parameter
    hl2ss::umq_command_buffer buffer;
    std::vector<uint32_t> response;
    buffer.add(command_id, command_data, sizeof(command_data));
    response.resize(buffer.get_count());

    // Initialize PV camera
    hl2ss::lnm::start_subsystem_pv(host, hl2ss::stream_port::PERSONAL_VIDEO);

    // Start clients
    std::unique_ptr<hl2ss::mt::source> source_pv = std::make_unique<hl2ss::mt::source>(buffer_size*pv_fps, hl2ss::lnm::rx_pv(host, hl2ss::stream_port::PERSONAL_VIDEO, pv_width, pv_height, pv_fps));
    std::unique_ptr<hl2ss::ipc_umq> client_umq = hl2ss::lnm::ipc_umq(host, hl2ss::ipc_port::UNITY_MESSAGE_QUEUE);

    source_pv->start();
    client_umq->open();
   
    // Main loop
    while (true)
    {
        // Check for errors
        std::exception error;
        if (!source_pv->status(error)) { throw error; }

        // Get most recent frame
        int64_t frame_stamp = -1;
        int32_t status;
        std::shared_ptr<hl2ss::packet> data_pv = source_pv->get_packet(frame_stamp, status);

        if (data_pv)
        {
            // Show video frame
            // Note that the unpacked pointers are only valid as long as data_pv exists    
            hl2ss::map_pv region = hl2ss::unpack_pv(data_pv->payload.get(), data_pv->sz_payload);
            cv::Mat pv_mat = cv::Mat(pv_height, pv_width, CV_8UC3, region.image);
            cv::imshow("PV", pv_mat);

            // Send test message to the HoloLens app
            client_umq->push(buffer.get_data(), buffer.get_size());
            client_umq->pull(response.data(), buffer.get_count());
            std::cout << "Response from HoloLens app: " << response[0] << std::endl;
        }

        // Update window and stop if ESC was pressed
        int key = cv::waitKey(1);
        if ((key & 0xFF) == 27) { break; }
    }

    // Stop clients
    client_umq->close();
    source_pv->stop();

    // Stop PV camera
    hl2ss::lnm::stop_subsystem_pv(host, hl2ss::stream_port::PERSONAL_VIDEO);
}

//-----------------------------------------------------------------------------
// Extended Video
//-----------------------------------------------------------------------------

void test_extended_video(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_VIDEO;
    uint16_t width = 1280;
    uint16_t height = 720;
    uint8_t framerate = 30;

    std::unique_ptr<hl2ss::rx_pv> client = hl2ss::lnm::rx_pv(host, port, width, height, framerate);
    std::string port_name = hl2ss::get_port_name(port);

    hl2ss::lnm::start_subsystem_pv(host, port, false, false, false, false, false, false, false, 0, 2, 4, 0, 0);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_pv region = hl2ss::unpack_pv(data->payload.get(), data->sz_payload);

        print_packet_metadata(data->timestamp, data->pose.get());

        std::cout << "Focal length: "    << region.metadata->f.x << ", " << region.metadata->f.y << std::endl;
        std::cout << "Principal point: " << region.metadata->c.x << ", " << region.metadata->c.y << std::endl;
        std::cout << "Resolution: " << region.metadata->width << "x" << region.metadata->height << std::endl;

        cv::Mat mat_image = cv::Mat(height, width, CV_8UC3, region.image);
        cv::imshow(port_name, mat_image);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();

    hl2ss::lnm::stop_subsystem_pv(host, port);
}

//-----------------------------------------------------------------------------
// PV (shared)
//-----------------------------------------------------------------------------

void test_pv_shared(char const* host)
{
    uint16_t port = hl2ss::stream_port::PERSONAL_VIDEO;
    uint16_t width = 1920;
    uint16_t height = 1080;
    uint8_t channels = hl2ss::decoder_pv::decoded_bpp(hl2ss::pv_decoded_format::BGR);
    uint8_t framerate = 30;
    bool enable_mrc = false;
    bool shared = true;

    std::unique_ptr<hl2ss::rx_pv> client = hl2ss::lnm::rx_pv(host, port, width, height, framerate);
    std::string port_name = hl2ss::get_port_name(port);

    hl2ss::lnm::start_subsystem_pv(host, port, enable_mrc, true, false, false, false, false, shared);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_pv region = hl2ss::unpack_pv(data->payload.get(), data->sz_payload);

        print_packet_metadata(data->timestamp, data->pose.get());

        std::cout << "Focal length: "    << region.metadata->f.x << ", " << region.metadata->f.y << std::endl;
        std::cout << "Principal point: " << region.metadata->c.x << ", " << region.metadata->c.y << std::endl;

        std::cout << "exposure_time: " << region.metadata->exposure_time << std::endl;
        std::cout << "exposure_compensation: " << region.metadata->exposure_compensation.val[0] << "," << region.metadata->exposure_compensation.val[1] << std::endl;
        std::cout << "lens_position: " << region.metadata->lens_position << std::endl;
        std::cout << "focus_state: " << region.metadata->focus_state << std::endl;
        std::cout << "iso_speed: " << region.metadata->iso_speed << std::endl;
        std::cout << "white_balance: " << region.metadata->white_balance << std::endl;
        std::cout << "iso_gains: " << region.metadata->iso_gains.x << "," << region.metadata->iso_gains.y << std::endl;
        std::cout << "white_balance_gains: " << region.metadata->white_balance_gains.x << "," << region.metadata->white_balance_gains.y << "," << region.metadata->white_balance_gains.z << std::endl;

        std::cout << "dimensions: " << client->width << " x " << client->height << " x " << (int)channels << std::endl;

        cv::Mat mat_image = cv::Mat(client->height, client->width, CV_8UC(channels), region.image);
        cv::imshow(port_name, mat_image);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();

    hl2ss::lnm::stop_subsystem_pv(host, port);
}

//-----------------------------------------------------------------------------
// Extended Depth
//-----------------------------------------------------------------------------

void test_extended_depth(char const* host)
{
    uint16_t port = hl2ss::stream_port::EXTENDED_DEPTH;
    float group_index = 1;
    float source_index = 0;
    float profile_index = 0;
    uint64_t media_index = 15;

    std::unique_ptr<hl2ss::rx_extended_depth> client = hl2ss::lnm::rx_extended_depth(host, port, media_index);
    std::string port_name = hl2ss::get_port_name(port);

    hl2ss::lnm::start_subsystem_pv(host, port, false, false, false, false, false, false, false, group_index, source_index, profile_index, 0, 0);

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_extended_depth region = hl2ss::unpack_extended_depth(data->payload.get(), data->sz_payload);

        print_packet_metadata(data->timestamp, data->pose.get());

        std::cout << "resolution: " << region.metadata->width << " x " << region.metadata->height << std::endl;

        cv::Mat depth = cv::Mat(region.metadata->height, region.metadata->width, CV_16UC1, region.depth) * 4;
        cv::imshow(port_name, depth);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    client->close();

    hl2ss::lnm::stop_subsystem_pv(host, port);
}

//-----------------------------------------------------------------------------
// Device Portal MRC
//-----------------------------------------------------------------------------

void test_dp_mrc(char const* host)
{
#ifdef HL2SS_ENABLE_DP
    hl2ss::dp::mrc_configuration configuration{true, true, true, true, true, false, 0};
    
    std::unique_ptr<hl2ss::dp::rx_mrc> client = hl2ss::lnm::rx_mrc(host, hl2ss::dp::stream_port::LIVE, "user", "pass");

    client->open();
    for (;;)
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();

        print_packet_metadata(data->timestamp, data->pose.get());

        uint32_t kind = hl2ss::dp::mrc_get_kind(data->payload[0]);        

        if (kind == hl2ss::dp::stream_kind::VIDEO)
        {
            hl2ss::dp::map_mrc_video region = hl2ss::dp::unpack_mrc_video(data->payload.get());

            cv::Mat mat_image = cv::Mat(region.header->height, region.header->width, CV_8UC(3), region.data);
            cv::imshow("mrc", mat_image);
            if ((cv::waitKey(1) & 0xFF) == 27) { break; }
        }
    }
    client->close();
#else
    throw std::runtime_error("main : HL2SS_ENABLE_DP not defined");
#endif
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main()
{
    char const* host = "192.168.1.7";
    int test_id = 26;

    try
    {
        hl2ss::client::initialize();
        
        switch (test_id)
        {
        case  0: test_rm_vlc(host, hl2ss::stream_port::RM_VLC_LEFTFRONT); break; // OK
        case  1: test_rm_vlc(host, hl2ss::stream_port::RM_VLC_LEFTLEFT); break; // OK
        case  2: test_rm_vlc(host, hl2ss::stream_port::RM_VLC_RIGHTFRONT); break; // OK
        case  3: test_rm_vlc(host, hl2ss::stream_port::RM_VLC_RIGHTRIGHT); break; // OK
        case  4: test_rm_depth_ahat(host); break; // OK
        case  5: test_rm_depth_longthrow(host); break; // OK
        case  6: test_rm_imu(host, hl2ss::stream_port::RM_IMU_ACCELEROMETER); break; // OK
        case  7: test_rm_imu(host, hl2ss::stream_port::RM_IMU_GYROSCOPE); break; // OK
        case  8: test_rm_imu(host, hl2ss::stream_port::RM_IMU_MAGNETOMETER); break; // OK
        case  9: test_pv(host, 1920, 1080, 30, false); break; // OK
        case 10: test_pv(host, 1920, 1080, 30, true); break; // OK
        case 11: test_microphone(host); break; // OK
        case 12: test_si(host); break; // OK
        case 13: test_eet(host); break; // OK
        case 14: test_rc(host); break; // OK
        case 15: test_sm(host); break; // OK
        case 16: test_su(host); break; // OK
        case 17: test_vi(host); break; // OK
        case 18: test_umq(host); break; // OK
        case 19: test_extended_audio(host); break; // OK
        case 20: test_mt(host); break; // OK
        case 21: test_gmq(host); break; // OK
        case 22: test_pv_umq(host); break; // OK
        case 23: test_extended_video(host); break; // OK
        case 24: test_pv_shared(host); break; // OK
        case 25: test_extended_depth(host); break; // OK
        case 26: test_dp_mrc(host); break;
        default: std::cout << "NO TEST" << std::endl; break;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }   
    
    return 0;
}
