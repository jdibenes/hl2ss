%%
% This script downloads Spatial Mapping data from the HoloLens.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Maximum number of active threads (on the HoloLens) to compute meshes
threads = 2;

%%

client = hl2ss.mt.ipc_sm(host, hl2ss.ipc_port.SPATIAL_MAPPING);
client.open();

try
% sample sphere region
buffer = hl2ss.sm_bounding_volume();
buffer.add_sphere([0, 0, 0], 5);

client.set_volumes(buffer.volumes);

surface_infos = client.get_observed_surfaces();

% download all observed surfaces
tasks = struct([]);
for k = 1:numel(surface_infos)
    task = struct();
    task.id                            = surface_infos(k).id;
    task.max_triangles_per_cubic_meter = 1000;
    task.vertex_position_format        = hl2ss.sm_vertex_position_format.R32G32B32A32Float;
    task.triangle_index_format         = hl2ss.sm_triangle_index_format.R32Uint;
    task.vertex_normal_format          = hl2ss.sm_vertex_normal_format.R32G32B32A32Float;
    task.include_vertex_normals        = true;
    task.include_bounds                = true;
    tasks = [tasks; task];
end

meshes = client.get_meshes(tasks, threads);
catch ME
    disp(ME.message);
end

client.close();
