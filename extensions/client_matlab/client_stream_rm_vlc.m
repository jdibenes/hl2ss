%%
% This script receives video from one of the HoloLens sideview grayscale
% cameras and plays it.
% Close the figure to stop.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Port
% Options:
% hl2ss.stream_port.RM_VLC_LEFTFRONT
% hl2ss.stream_port.RM_VLC_LEFTLEFT
% hl2ss.stream_port.RM_VLC_RIGHTFRONT
% hl2ss.stream_port.RM_VLC_RIGHTRIGHT
port = hl2ss.stream_port.RM_VLC_LEFTFRONT;

%%

client = hl2ss.mt.sink_rm_vlc(host, port);
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
            colormap gray
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
