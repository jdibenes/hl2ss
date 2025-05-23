
% Using the MEX library on Ubuntu currently requires
% export LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libtiff.so
% matlab -softwareopengl

mex('hl2ss_matlab.cpp', ...
    '-I../client_cpp', ...
    '-L../client_cpp', ...
    '-lhl2ss_ulm', ...
    'LDFLAGS=$LDFLAGS -Wl,-rpath,\$ORIGIN' ...
);
