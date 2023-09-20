
hl2ss_matlab('open', '192.168.1.7', uint16(3813));

hl2ss_matlab('ipc_call', uint16(3813), 'create_observer');

volumes = [];
volumes.type = uint32(3);
volumes.center = single([0, 0, 0]);
volumes.radius = single(5);

hl2ss_matlab('ipc_call', uint16(3813), 'set_volumes', volumes);

surface_infos = hl2ss_matlab('ipc_call', uint16(3813), 'get_observed_surfaces');

tasks = struct([]);
for k = 1:numel(surface_infos)
    task = [];
    task.id = surface_infos(k).id;
    task.max_triangles_per_cubic_meter = double(1000);
    task.vertex_position_format = uint32(13); %uint32(2); 
    task.triangle_index_format = uint32(57); %uint32(42);
    task.vertex_normal_format = uint32(31); %uint32(2);
    task.include_vertex_normals = logical(true);
    task.include_bounds = logical(true);
    tasks = [tasks; task];
end

meshes = hl2ss_matlab('ipc_call', uint16(3813), 'get_meshes', tasks, uint32(2));

hl2ss_matlab('close', uint16(3813));
