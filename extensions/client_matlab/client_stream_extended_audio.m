
%OK

hl2ss_matlab('open', '192.168.1.7', uint16(3818), uint64(4096), uint32(2), single(1.0), single(1.0), uint8(0xFF), uint8(41), uint64(300));
record = [];
length = 500;
frame_index = int64(-1);
while (true)
    response = hl2ss_matlab('get_packet', uint16(3818), uint8(0), frame_index);
    if (response.status ~= 0)
        pause(0.1);
    else
        frame_index = int64(response.frame_index + 1);
        record = [record, response.audio.'];
        length = length - 1;
        if (length <= 0), break; end
    end
end
hl2ss_matlab('close', uint16(3818));
