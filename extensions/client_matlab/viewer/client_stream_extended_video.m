%%
% This script receives video from the HoloLens front RGB camera and plays
% it.
% Close the figure to stop.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Camera selection (default is PV)
group_index = 0;
source_index = 2;
profile_index = 4;

% Camera parameters
width     = 1280;
height    = 720;
framerate = 30;

%%

client = hl2ss.mt.sink_pv(host, hl2ss.stream_port.EXTENDED_VIDEO, width, height, framerate);

media_group = client.download_calibration();
disp(native2unicode(media_group.device_list, "UTF-16LE").');

client.start_subsystem(false, false, false, false, false, false, false, group_index, source_index, profile_index, 0, 0);
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
