%%
% This script records microphone audio from the HoloLens and plays it.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Recording length in seconds
length = 20;

%%

client = hl2ss.mt.sink_microphone(host, hl2ss.stream_port.MICROPHONE);
client.open();

packet_duration = hl2ss.parameters_microphone.GROUP_SIZE_AAC / hl2ss.parameters_microphone.SAMPLE_RATE;
packet_count = length / packet_duration;
record = [];
frame_index = -1; % -1 for most recent frame

try
while (packet_count > 0)
    data = client.get_packet_by_index(frame_index);
    if (data.status == 0) % got packet
        frame_index = data.frame_index + 1;
        record = [record, data.audio];
        packet_count = packet_count - 1;
    else % no data
        pause(packet_duration * 10); % wait for data
    end    
end
catch ME
    disp(ME.message);
end

client.close();

% play recorded sound
sound(record, hl2ss.parameters_microphone.SAMPLE_RATE);
