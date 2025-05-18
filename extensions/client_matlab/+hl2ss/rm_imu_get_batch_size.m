
function [count] = rm_imu_get_batch_size(port)
if (port == hl2ss.stream_port.RM_IMU_ACCELEROMETER)
    count = hl2ss.parameters_rm_imu_accelerometer.BATCH_SIZE;
elseif (port == hl2ss.stream_port.RM_IMU_GYROSCOPE)
    count = hl2ss.parameters_rm_imu_gyroscope.BATCH_SIZE;
elseif (port == hl2ss.stream_port.RM_IMU_MAGNETOMETER)
    count = hl2ss.parameters_rm_imu_magnetometer.BATCH_SIZE;
else
    count = 0;
end
