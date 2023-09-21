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
command = typecast(uint32(8), 'uint8'); % tts command id is 8
text = uint8(text); % ascii text to uint8
length = typecast(uint32(numel(text)), 'uint8'); % encode size of text in bytes

data = [command, length, text]; % message: command, size of parameters, parameters

client.push(data);
response = client.pull(1);
catch ME
    disp(ME.message);
end

client.close();
