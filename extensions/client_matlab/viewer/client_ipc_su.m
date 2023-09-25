%%
% This script downloads Scene Understanding data from the HoloLens.

%% Settings

% HoloLens address
host = '192.168.1.7';

% Task parameters
task = struct();
task.enable_quads         = true;
task.enable_meshes        = true;
task.enable_only_observed = false;
task.enable_world_mesh    = true;
task.mesh_lod             = hl2ss.su_mesh_lod.Fine;
task.query_radius         = 5;
task.create_mode          = hl2ss.su_create.New;
task.kind_flags           = bitor(hl2ss.su_kind_flag.Wall, hl2ss.su_kind_flag.Floor);
task.get_orientation      = true;
task.get_position         = true;
task.get_location_matrix  = true;
task.get_quad             = true;
task.get_meshes           = true;
task.get_collider_meshes  = true;
task.guid_list            = [];

%%

client = hl2ss.mt.ipc_su(host, hl2ss.ipc_port.SCENE_UNDERSTANDING);
client.open();

try
result = client.query(task);
catch ME
    disp(ME.message);
end

client.close();
