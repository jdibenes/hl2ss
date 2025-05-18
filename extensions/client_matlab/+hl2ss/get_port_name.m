
function [name] = get_port_name(port)
if (port == stream_port.RM_VLC_LEFTFRONT)
    name = "rm_vlc_leftfront";
elseif (port == stream_port.RM_VLC_LEFTLEFT)
    name = "rm_vlc_leftleft";
elseif (port == stream_port.RM_VLC_RIGHTFRONT)
    name = "rm_vlc_rightfront";
elseif (port == stream_port.RM_VLC_RIGHTRIGHT)
    name = "rm_vlc_rightright";
elseif (port == stream_port.RM_DEPTH_AHAT)
    name = "rm_depth_ahat";
elseif (port == stream_port.RM_DEPTH_LONGTHROW)
    name = "rm_depth_longthrow";
elseif (port == stream_port.RM_IMU_ACCELEROMETER)
    name = "rm_imu_accelerometer";
elseif (port == stream_port.RM_IMU_GYROSCOPE)
    name = "rm_imu_gyroscope";
elseif (port == stream_port.RM_IMU_MAGNETOMETER)
    name = "rm_imu_magnetometer";
elseif (port == ipc_port.REMOTE_CONFIGURATION)
    name = "remote_configuration";
elseif (port == stream_port.PERSONAL_VIDEO)
    name = "personal_video";
elseif (port == stream_port.MICROPHONE)
    name = "microphone";
elseif (port == stream_port.SPATIAL_INPUT)
    name = "spatial_input";
elseif (port == ipc_port.SPATIAL_MAPPING)
    name = "spatial_mapping";
elseif (port == ipc_port.SCENE_UNDERSTANDING)
    name = "scene_understanding";
elseif (port == ipc_port.VOICE_INPUT)
    name = "voice_input";
elseif (port == ipc_port.UNITY_MESSAGE_QUEUE)
    name = "unity_message_queue";
elseif (port == stream_port.EXTENDED_EYE_TRACKER)
    name = "extended_eye_tracker";
elseif (port == stream_port.EXTENDED_AUDIO) 
    name = "extended_audio";
elseif (port == stream_port.EXTENDED_VIDEO)
    name = "extended_video";
elseif (port == ipc_port.GUEST_MESSAGE_QUEUE)
    name = "guest_message_queue";
elseif (port == stream_port.EXTENDED_DEPTH)
    name = "extended_depth";
else
    name = "";
end
end
