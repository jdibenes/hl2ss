%%
% This script receives video from the HoloLens front RGB camera and plays
% it.
% Close the figure to stop.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Camera parameters
width     = 1280;
height    = 720;
framerate = 30;

% Enable Mixed Reality Capture (Holograms)
enable_mrc = false;

%%

client = hl2ss.mt.sink_pv(host, hl2ss.stream_port.PERSONAL_VIDEO, width, height, framerate);

client.start_subsystem(enable_mrc);
calibration = client.download_calibration();
client.open();

h = []; % figure handle

try
while (true)
    data = client.get_packet_by_index(-1); % -1 for most recent frame
    if (data.status == 0) % got packet
        frame = data.image;
        if (isempty(h))
            h = image(frame);
        else
            h.CData = frame;
        end
        drawnow
    else % no data
        pause(1); % wait for data
    end
end
catch ME
    disp(ME.message);
end

client.close();
client.stop_subsystem();
