
function [t] = ts_unix_hns_to_filetime(timestamp_unix_hns)
t = timestamp_unix_hns + time_base.UNIX_EPOCH;
end
