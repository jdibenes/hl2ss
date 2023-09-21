%%
% This script receives spatial input data from the HoloLens.

%% Settings

% HoloLens address
host = '192.168.1.7';

%%

client = hl2ss.mt.sink_si(host, hl2ss.stream_port.SPATIAL_INPUT);
client.open();

try
while (true)
    data = client.get_packet_by_index(-1);
    if (data.status == 0) % got packet
        break
    else
        pause(1); % wait for data
    end
end
catch ME
    disp(ME.message);
end

client.close();
