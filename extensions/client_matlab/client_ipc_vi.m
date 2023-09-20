
hl2ss_matlab('open', '192.168.1.7', uint16(3815));

hl2ss_matlab('ipc_call', uint16(3815), 'create_recognizer');

commands = string([]);
commands(1) = 'cat';
commands(2) = 'dog';

ok = hl2ss_matlab('ipc_call', uint16(3815), 'register_commands', true, commands);

hl2ss_matlab('ipc_call', uint16(3815), 'start');

while (true)
result = hl2ss_matlab('ipc_call', uint16(3815), 'pop');
if (numel(result) > 0)
    break;
end
pause(0.1);
end

hl2ss_matlab('ipc_call', uint16(3815), 'stop');

hl2ss_matlab('close', uint16(3815));
