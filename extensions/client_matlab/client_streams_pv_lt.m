
%OK

hl2ss_matlab('start_subsystem_pv', '192.168.1.7', uint16(3810), logical(false))
hl2ss_matlab('open', '192.168.1.7', uint16(3810), uint16(1920), uint16(1080), uint8(30), uint64(4096), uint8(1), uint8(1), uint8(3), uint8(255), uint32(0), uint64([10, 30]), uint8(1), uint64(300));
hl2ss_matlab('open', '192.168.1.7', uint16(3805), uint64(4096), uint8(1), uint8(1), uint8(5), uint64(300));
while (true)
    response_lt = hl2ss_matlab('get_packet', uint16(3805), uint8(0), int64(-1));
    if (response_lt.status ~= 0)
        pause(1);
        continue
    end
    response_pv = hl2ss_matlab('get_packet', uint16(3810), uint8(1), uint64(response_lt.timestamp), int32(0));
    if (response_pv.status == 0)
        disp(['delta: ' num2str(double(response_lt.timestamp) - double(response_pv.timestamp))]);
        pause(0.2);
    end
end
hl2ss_matlab('close', uint16(3805));
hl2ss_matlab('close', uint16(3810));
hl2ss_matlab('stop_subsystem_pv', '192.168.1.7', uint16(3810));
