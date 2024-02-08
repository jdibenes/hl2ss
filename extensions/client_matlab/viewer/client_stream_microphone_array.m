%%
% This script records microphone audio from the HoloLens and plays it.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Recording length in seconds
length = 10;

% Channel
% Options:
% hl2ss.parameters_microphone.ARRAY_TOP_LEFT
% hl2ss.parameters_microphone.ARRAY_TOP_CENTER
% hl2ss.parameters_microphone.ARRAY_TOP_RIGHT
% hl2ss.parameters_microphone.ARRAY_BOTTOM_LEFT
% hl2ss.parameters_microphone.ARRAY_BOTTOM_RIGHT
channel = hl2ss.parameters_microphone.ARRAY_TOP_LEFT;

%%

client = hl2ss.mt.sink_microphone(host, hl2ss.stream_port.MICROPHONE, 4096, hl2ss.audio_profile.RAW, hl2ss.aac_level.L5);
client.open();

packet_duration = hl2ss.parameters_microphone.GROUP_SIZE_RAW / hl2ss.parameters_microphone.SAMPLE_RATE;
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
sound(record(channel, :), hl2ss.parameters_microphone.SAMPLE_RATE);
