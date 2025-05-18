
function [t] = ts_filetime_to_qpc(timestamp_filetime, utc_offset)
t = timestamp_filetime - utc_offset;
end
