%%
% This script registers voice commands on the HoloLens and continously
% checks if any of the registered commands has been heard.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Voice commands
commands = ["cat", "dog", "red", "blue"];

%%

client = hl2ss.mt.ipc_vi(host, hl2ss.ipc_port.VOICE_INPUT);
client.open();

try
client.start(commands);

% run until one of the commands is detected
disp('Ready. Try saying any of the commands you defined.');
while (true)
    results = client.pop();
    if (numel(results) > 0)
        break;
    end
end

client.stop();    
catch ME
    disp(ME.message);
end

client.close();

disp('Received command');
disp(commands(results(1).index + 1));
