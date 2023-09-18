
%OK

hl2ss_matlab('open', '192.168.1.7', uint16(3817), uint64(4096), uint8(30), uint64(300));
while (true)
    response = hl2ss_matlab('get_packet', uint16(3817), uint8(0), int64(-1));
    if (response.status == 0)
        break;
    end
    pause(1);
end
hl2ss_matlab('close', uint16(3817));
