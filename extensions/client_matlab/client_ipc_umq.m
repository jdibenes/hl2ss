
hl2ss_matlab('open', '192.168.1.7', uint16(3816));

command = typecast(uint32(8), 'uint8');
text = uint8('hello from matlab!');
length = typecast(uint32(numel(text)), 'uint8');

data = [command, length, text];

hl2ss_matlab('ipc_call', uint16(3816), 'push', data);
response = hl2ss_matlab('ipc_call', uint16(3816), 'pull', uint32(1));

hl2ss_matlab('close', uint16(3816));
