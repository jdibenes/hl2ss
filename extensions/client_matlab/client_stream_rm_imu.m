%%
% This script receives IMU samples from the HoloLens and prints them.
% Close the figure to stop.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Port
% Options:
% hl2ss.stream_port.RM_IMU_ACCELEROMETER
% hl2ss.stream_port.RM_IMU_GYROSCOPE
% hl2ss.stream_port.RM_IMU_MAGNETOMETER
port = hl2ss.stream_port.RM_IMU_ACCELEROMETER;

%%

client = hl2ss.mt.sink_rm_imu(host, port);
if (port ~= hl2ss.stream_port.RM_IMU_MAGNETOMETER)
    calibration = client.download_calibration();
end
client.open();

h = figure();

try
while (true)
    data = client.get_packet_by_index(-1); % -1 for most recent frame
    if (data.status == 0) % got packet
        subplot(3, 1, 1, 'Parent', h);
        plot(data.x);
        title('x');
        
        subplot(3, 1, 2, 'Parent', h);        
        plot(data.y);
        title('y');
        
        subplot(3, 1, 3, 'Parent', h);
        plot(data.z);
        title('z');
        
        drawnow
    else % no data
        pause(1); % wait for data
    end
end
catch ME
    disp(ME.message);
end

client.close();
