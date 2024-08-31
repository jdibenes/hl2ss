%%
% This script demonstrates how to receive a command with a string parameter 
% from a Unity app using the plugin.

%% Settings

% HoloLens address
host = '192.168.1.7';

%%

client = hl2ss.mt.ipc_gmq(host, hl2ss.ipc_port.GUEST_MESSAGE_QUEUE);
client.open();

try
[command, data] = client.pull(); % get command
if (command == 0xFFFFFFFE)
    text = native2unicode(data, "UTF-8").';
    client.push(1)
    disp(text);
end
catch ME
    disp(ME.message);
end

client.close();
