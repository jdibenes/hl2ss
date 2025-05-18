
function [t] = ts_unix_hns_to_unix(timestamp_unix_hns)
t = timestamp_unix_hns / time_base.HUNDREDS_OF_NANOSECONDS;
end
