
#include <iostream>
#include <opencv2/highgui.hpp>
#include "hl2ss_ulm.h"

hl2ss::calibration_rm_vlc calibration_rm_vlc[4];
hl2ss::calibration_rm_depth_ahat calibration_rm_depth_ahat;
hl2ss::calibration_rm_depth_longthrow calibration_rm_depth_longthrow;
hl2ss::calibration_rm_imu calibration_rm_imu[2];
hl2ss::calibration_pv calibration_pv;

void test_sources(char const* host)
{
    hl2ss::ulm::initialize();

    uint16_t source_ports[] = 
    {
        //hl2ss::stream_port::RM_VLC_LEFTFRONT,
        //hl2ss::stream_port::RM_VLC_LEFTLEFT,
        //hl2ss::stream_port::RM_VLC_RIGHTFRONT,
        //hl2ss::stream_port::RM_VLC_RIGHTRIGHT,
        //hl2ss::stream_port::RM_DEPTH_AHAT,
        //hl2ss::stream_port::RM_DEPTH_LONGTHROW,
        //hl2ss::stream_port::RM_IMU_ACCELEROMETER,
        //hl2ss::stream_port::RM_IMU_GYROSCOPE,
        //hl2ss::stream_port::RM_IMU_MAGNETOMETER,
        hl2ss::stream_port::PERSONAL_VIDEO,
        //hl2ss::stream_port::MICROPHONE,
        //hl2ss::stream_port::SPATIAL_INPUT,
        //hl2ss::stream_port::EXTENDED_EYE_TRACKER,
        //hl2ss::stream_port::EXTENDED_AUDIO,
        //hl2ss::stream_port::EXTENDED_VIDEO,
    };

    constexpr int source_count = sizeof(source_ports) / sizeof(uint16_t);
    void* source_objects[source_count];

    for (int i = 0; i < source_count; ++i)
    {
        switch (source_ports[i])
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:
            hl2ss::ulm::download_calibration_rm_vlc(host, source_ports[i], calibration_rm_vlc[0]);
            source_objects[i] = hl2ss::ulm::open_rm_vlc(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:
            hl2ss::ulm::download_calibration_rm_vlc(host, source_ports[i], calibration_rm_vlc[1]);
            source_objects[i] = hl2ss::ulm::open_rm_vlc(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
            hl2ss::ulm::download_calibration_rm_vlc(host, source_ports[i], calibration_rm_vlc[2]);  
            source_objects[i] = hl2ss::ulm::open_rm_vlc(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:
            hl2ss::ulm::download_calibration_rm_vlc(host, source_ports[i], calibration_rm_vlc[3]);
            source_objects[i] = hl2ss::ulm::open_rm_vlc(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:
            hl2ss::ulm::download_calibration_rm_depth_ahat(host, source_ports[i], calibration_rm_depth_ahat);
            source_objects[i] = hl2ss::ulm::open_rm_depth_ahat(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:
            hl2ss::ulm::download_calibration_rm_depth_longthrow(host, source_ports[i], calibration_rm_depth_longthrow);
            source_objects[i] = hl2ss::ulm::open_rm_depth_longthrow(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
            hl2ss::ulm::download_calibration_rm_imu(host, source_ports[i], calibration_rm_imu[0]);
            source_objects[i] = hl2ss::ulm::open_rm_imu(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:
            hl2ss::ulm::download_calibration_rm_imu(host, source_ports[i], calibration_rm_imu[1]); 
            source_objects[i] = hl2ss::ulm::open_rm_imu(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  
            source_objects[i] = hl2ss::ulm::open_rm_imu(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::PERSONAL_VIDEO:
            hl2ss::ulm::start_subsystem_pv(host, source_ports[i]);
            hl2ss::ulm::download_calibration_pv(host, source_ports[i], 640, 360, 30, calibration_pv);
            source_objects[i] = hl2ss::ulm::open_pv(host, source_ports[i], 640, 360, 30);
            break;
        case hl2ss::stream_port::MICROPHONE:           
            source_objects[i] = hl2ss::ulm::open_microphone(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::SPATIAL_INPUT:        
            source_objects[i] = hl2ss::ulm::open_si(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: 
            source_objects[i] = hl2ss::ulm::open_eet(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::EXTENDED_AUDIO:       
            source_objects[i] = hl2ss::ulm::open_extended_audio(host, source_ports[i]); 
            break;
        case hl2ss::stream_port::EXTENDED_VIDEO:
            hl2ss::ulm::start_subsystem_pv(host, source_ports[i], 0, 0, 0, 0, 0, 0, 0, 0, 2, 4, 0, 1);
            source_objects[i] = hl2ss::ulm::open_pv(host, source_ports[i], 640, 360, 30); 
            break;
        }
    }

    cv::namedWindow("CONTROL");

    while (true)
    {
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        int64_t frame_stamp = -1;
        int32_t status;
        void* frame;
        uint64_t timestamp;
        uint32_t payload_size;
        uint8_t* payload;
        hl2ss::matrix_4x4* pose;
        uint64_t target_timestamp;
        int32_t result;

        uint8_t* pv_image;
        hl2ss::pv_intrinsics* pv_intrinsics;

        for (int i = 0; i < source_count; ++i)
        {
            result = hl2ss::ulm::get_by_index(source_objects[i], frame_stamp, status, frame, timestamp, payload_size, payload, pose);
            if (result < 0) { continue; } // fatal error (network error, decode error, ...), must close and open anew
            if (status != 0) { continue; } // status < 0: frame has been dropped, status == 0: OK, status > 0: frame has not arrived yet

            switch (source_ports[i])
            {
            case hl2ss::stream_port::PERSONAL_VIDEO:
            case hl2ss::stream_port::EXTENDED_VIDEO:
                hl2ss::unpack_pv(payload, payload_size, pv_image, pv_intrinsics);
                cv::imshow(hl2ss::get_port_name(source_ports[i]), cv::Mat(360, 640, CV_8UC3, pv_image));
                break;
            }

            hl2ss::ulm::release_frame(frame); // all pointers from hl2ss::ulm::get_by_[...] and hl2ss::unpack_[...] are now "invalid"

            target_timestamp = timestamp;

            result = hl2ss::ulm::get_by_timestamp(source_objects[i], target_timestamp, hl2ss::mt::time_preference::PREFER_NEAREST, false, frame_stamp, status, frame, timestamp, payload_size, payload, pose);
            if (result < 0) { continue; } // fatal error (network error, decode error, ...), must close and open anew
            if (status != 0) { continue; } // status != 0: buffer is empty, status == 0: OK

            hl2ss::ulm::release_frame(frame);
        }
    }

    for (int i = 0; i < source_count; ++i)
    {
        switch (source_ports[i])
        {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_DEPTH_AHAT:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_DEPTH_LONGTHROW:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_IMU_GYROSCOPE:
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::PERSONAL_VIDEO:
            hl2ss::ulm::close_source(source_objects[i]);
            hl2ss::ulm::stop_subsystem_pv(host, source_ports[i]);
            break;
        case hl2ss::stream_port::MICROPHONE:           
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::SPATIAL_INPUT:        
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::EXTENDED_EYE_TRACKER: 
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::EXTENDED_AUDIO:       
            hl2ss::ulm::close_source(source_objects[i]);
            break;
        case hl2ss::stream_port::EXTENDED_VIDEO:
            hl2ss::ulm::close_source(source_objects[i]);
            hl2ss::ulm::stop_subsystem_pv(host, source_ports[i]);
            break;
        }
    }
}

int main()
{
    test_sources("192.168.1.7");
    return 0;
}
