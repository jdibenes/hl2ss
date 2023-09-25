%%
% This script sends a string to the Unity sample app running on the
% HoloLens which is then spoken using TTS.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Text to say
text = 'hello from matlab!';

%%

client = hl2ss.mt.ipc_umq(host, hl2ss.ipc_port.UNITY_MESSAGE_QUEUE);
client.open();

try
% pack command into byte array
buffer = hl2ss.umq_command_buffer();
buffer.add(8, uint8(text)); % tts command id is 8

client.push(buffer.data);
response = client.pull(buffer.count);
catch ME
    disp(ME.message);
end

client.close();
