
hl2ss_matlab('open', '192.168.1.7', uint16(3810), logical(false), uint16(640), uint16(360), uint8(30));
index = 0;

while (capture)
[timestamp, payload, pose] = hl2ss_matlab('get_next_packet', uint16(3810));
index = index + 1;
if (index == 1)
    h = image(payload);
else
    h.CData = payload;
end
drawnow();
disp(timestamp);
disp(pose);
if (index == 300)
    break;
end
end

hl2ss_matlab('close', uint16(3810));
