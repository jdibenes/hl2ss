%%
% This script receives depth frames from an external USB-C camera connected
% to the HoloLens and plays it.
% Close the figure to stop.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Camera selection
group_index = 0;
source_index = 0;
profile_index = 0;
media_index = 15;

%%

client = hl2ss.mt.sink_extended_depth(host, hl2ss.stream_port.EXTENDED_DEPTH, media_index);

client.start_subsystem(false, false, false, false, false, false, false, group_index, source_index, profile_index, false, 0);
client.open();

h = []; % figure handle

try
while (true)
    data = client.get_packet_by_index(-1); % -1 for most recent frame
    if (data.status == 0) % got packet
        % normalize for visibility
        depth = double(data.depth);
        depth = depth / max(depth, [], 'all');
        frame = depth * 255;
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
client.stop_subsystem();
