
function data = unpack_microphone(response, audio_size, audio_type)
if (response.status == 0)
    data = struct('frame_index', response.frame_index, ...
                  'status',      response.status,      ...
                  'timestamp',   response.timestamp,   ...
                  'audio',       response.audio(:, 1:audio_size(2)));
    % TODO:                                        ^^^ FIX THIS ^^^
else    
    data = struct('frame_index', response.frame_index,            ...
                  'status',      response.status,                 ...
                  'timestamp',   zeros([1, 1],         'uint64'), ...
                  'audio',       zeros(audio_size,     audio_type));
end
end
