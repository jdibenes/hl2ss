
hl2ss_matlab('start_subsystem_pv', '192.168.1.7', uint16(3810), logical(false));

hl2ss_matlab('open', uint64(300), '192.168.1.7', uint16(3810), uint16(1920), uint16(1080), uint8(30), uint8(1));
hl2ss_matlab('open', uint64(300), '192.168.1.7', uint16(3800));
%hl2ss_matlab('open', uint64(300), '192.168.1.7', uint16(3804));
hl2ss_matlab('open', uint64(300), '192.168.1.7', uint16(3805));
hl2ss_matlab('open', uint64(300), '192.168.1.7', uint16(3811));

index = 0;
capture = true;

pv_index = 0;
lf_index = 0;

%videoPlayer_pv = vision.VideoPlayer;
%videoPlayer_lf = vision.VideoPlayer;

prev_pv = [];
prev_lf = [];

mc_collect = [];
mc_framestamp = int64(0);

while (capture)
[frame_stamp, status, timestamp_pv, payload_pv, pose] = hl2ss_matlab('get_packet', uint16(3810), int64(-1));
[frame_stamp, status, timestamp_lf, payload_lf, pose] = hl2ss_matlab('get_packet', uint16(3800), int64(-1));
%[frame_stamp, status, timestamp_ht, payload_ht, pose] = hl2ss_matlab('get_packet', uint16(3804), int64(-1));
[frame_stamp, status, timestamp_lt, payload_lt, pose] = hl2ss_matlab('get_packet', uint16(3805), int64(-1));
[frame_stamp, status, timestamp_mc, payload_mc, pose] = hl2ss_matlab('get_packet', uint16(3811), int64(mc_framestamp));

if (~isempty(payload_mc))
    mc_collect = [mc_collect, payload_mc];
    mc_framestamp = mc_framestamp + 1;
    if (mc_framestamp >= 500)
        break;
    end
end


%{
if (~isempty(payload_lf))
    if (~isempty(prev_lf))
        disp('LF');
        disp(timestamp_lf - prev_lf);
    end
    prev_lf = timestamp_lf;
%    videoPlayer_lf(payload_lf);
end
if (~isempty(payload_pv))
    if (~isempty(prev_pv))
        disp('PV');
        disp(timestamp_pv - prev_pv);
    end
    prev_pv = timestamp_pv;
%    videoPlayer_pv(payload_pv);
end

if (~isempty(payload_lt))
    pv_index = pv_index + 1;
    if (pv_index == 1)
        pv_h = image(payload_lt);
    else
        pv_h.CData = payload_lt;
    end
end
%}

%{
if (~isempty(payload_pv))
    pv_index = pv_index + 1;
    if (pv_index == 1)
        pv_h = image(payload_pv);
    else
        pv_h.CData = payload_pv;
    end
end

if (~isempty(payload_lf))
    lf_index = lf_index + 1;
    if (lf_index == 1)
        lf_h = image(payload_lf);
    else
        lf_h.CData = payload_lf;
    end
end
%}
%drawnow();
%if (index == 300)
%    break;
%end
end

hl2ss_matlab('close', uint16(3810));
hl2ss_matlab('close', uint16(3800));
%hl2ss_matlab('close', uint16(3804));
hl2ss_matlab('close', uint16(3805));
hl2ss_matlab('close', uint16(3811));

hl2ss_matlab('stop_subsystem_pv', '192.168.1.7', uint16(3810));
