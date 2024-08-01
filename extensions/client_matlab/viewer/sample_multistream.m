%%
% This example demonstrates how to associate data from multiple streams.
% Close the figure to stop.

%% Settings

% HoloLens address
host = '192.168.1.7';

%%

client_lf = hl2ss.mt.sink_rm_vlc(host, hl2ss.stream_port.RM_VLC_LEFTFRONT);
client_rf = hl2ss.mt.sink_rm_vlc(host, hl2ss.stream_port.RM_VLC_RIGHTFRONT);

h = []; % figure handle

try
client_lf.open();
client_rf.open();
    
while (true)
    data_lf = client_lf.get_packet_by_index(-6); % get sixth most recent frame (artificial delay)
    if (data_lf.status ~= 0) % no data
        pause(1); % wait for data
        continue;
    end
    data_rf = client_rf.get_packet_by_timestamp(data_lf.timestamp, hl2ss.grab_preference.PREFER_NEAREST, false);
    if (data_rf.status ~= 0) % no data
        pause(1); % wait for data
        continue;
    end
    frame = [imrotate(data_lf.image, -90), imrotate(data_rf.image, 90)];
    if (isempty(h))
        h = image(frame);
        colormap gray
    else
        h.CData = frame;
    end
    drawnow
end
catch ME
    disp(ME.message);
end

client_lf.close();
client_rf.close();
