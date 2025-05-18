
function [t] = ts_unix_to_unix_hns(timestamp_unix)
t = timestamp_unix * time_base.HUNDREDS_OF_NANOSECONDS;
end
