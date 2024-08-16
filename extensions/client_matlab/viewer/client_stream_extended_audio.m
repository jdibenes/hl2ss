%%
% This script records microphone and system audio from the HoloLens and
% plays it.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Recording length in seconds
length = 20;

%%

client = hl2ss.mt.sink_extended_audio(host, hl2ss.stream_port.EXTENDED_AUDIO);

media_group = client.download_calibration();
disp(native2unicode(media_group.device_list, "UTF-16LE").');

client.open();

packet_duration = hl2ss.parameters_extended_audio.GROUP_SIZE_AAC / hl2ss.parameters_extended_audio.SAMPLE_RATE;
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
sound(record, hl2ss.parameters_extended_audio.SAMPLE_RATE);
