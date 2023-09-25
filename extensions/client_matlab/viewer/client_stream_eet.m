%%
% This script receives extended eye tracking data from the HoloLens.

%% Settings

% HoloLens address
host = '192.168.1.7';

%%

client = hl2ss.mt.sink_eet(host, hl2ss.stream_port.EXTENDED_EYE_TRACKER);
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
