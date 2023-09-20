
hl2ss_matlab('open', '192.168.1.7', uint16(3814));

task = struct();
task.enable_quads = true;
task.enable_meshes = true;
task.enable_only_observed = false;
task.enable_world_mesh = true;
task.mesh_lod = uint32(2);
task.query_radius = single(5);
task.create_mode = uint8(0);
task.kind_flags = uint8(255);
task.get_orientation = true;
task.get_position = true;
task.get_location_matrix = true;
task.get_quad = true;
task.get_meshes = true;
task.get_collider_meshes = true;
task.guid_list = uint64([]);

result = hl2ss_matlab('ipc_call', uint16(3814), 'query', task);

hl2ss_matlab('close', uint16(3814));
