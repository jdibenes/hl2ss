
import multiprocessing as mp
import threading
import numpy as np
import open3d as o3d
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv


#------------------------------------------------------------------------------
# Open3D Interop
#------------------------------------------------------------------------------

def sm_mesh_to_open3d_triangle_mesh(mesh):
    open3d_mesh = o3d.geometry.TriangleMesh()

    open3d_mesh.vertices       = o3d.utility.Vector3dVector(mesh.vertex_positions[:, 0:3])
    open3d_mesh.vertex_normals = o3d.utility.Vector3dVector(mesh.vertex_normals[:, 0:3])
    open3d_mesh.triangles      = o3d.utility.Vector3iVector(mesh.triangle_indices)

    return open3d_mesh


def su_mesh_to_open3d_triangle_mesh(mesh):
    open3d_mesh = o3d.geometry.TriangleMesh()

    open3d_mesh.vertices  = o3d.utility.Vector3dVector(mesh.vertex_positions)
    open3d_mesh.triangles = o3d.utility.Vector3iVector(mesh.triangle_indices)

    return open3d_mesh


def open3d_triangle_mesh_swap_winding(open3d_mesh):
    open3d_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(open3d_mesh.triangles)[:, ::-1])
    return open3d_mesh


#------------------------------------------------------------------------------
# Spatial Mapping Data Manager
#------------------------------------------------------------------------------

class _sm_manager_entry:
    def __init__(self, update_time, mesh, rcs):
        self.update_time = update_time
        self.mesh = mesh
        self.rcs = rcs


class sm_manager:
    def __init__(self, host, triangles_per_cubic_meter, threads):
        self._tpcm = triangles_per_cubic_meter
        self._vpf = hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized
        self._tif = hl2ss.SM_TriangleIndexFormat.R16UInt
        self._vnf = hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized
        self._ipc = hl2ss_lnm.ipc_sm(host, hl2ss.IPCPort.SPATIAL_MAPPING)
        self._surfaces = {}
        self._volumes = None

    def open(self):
        self._ipc.open()

    def set_volumes(self, volumes):
        self._volumes = volumes

    def _load_updated_surfaces(self):
        self._surfaces = self._updated_surfaces

    def _get_surfaces(self):
        return self._surfaces.values()

    def get_observed_surfaces(self):
        self._updated_surfaces = {}
        tasks = hl2ss.sm_mesh_task()        
        updated_surfaces = []

        if (self._volumes is not None):
            self._ipc.set_volumes(self._volumes)
            self._volumes = None
        
        for surface_info in self._ipc.get_observed_surfaces():
            id = surface_info.id
            surface_info.id = surface_info.id.hex()
            if (surface_info.id in self._surfaces):
                previous_entry = self._surfaces[surface_info.id]
                if (surface_info.update_time <= previous_entry.update_time):
                    self._updated_surfaces[surface_info.id] = previous_entry
                    continue
            tasks.add_task(id, self._tpcm, self._vpf, self._tif, self._vnf)
            updated_surfaces.append(surface_info)

        count = len(updated_surfaces)
        if (count <= 0):
            return

        for index, mesh in self._ipc.get_meshes(tasks).items():
            if (mesh is None):
                continue
            mesh.unpack(self._vpf, self._tif, self._vnf)
            hl2ss_3dcv.sm_mesh_cast(mesh, np.float64, np.uint32, np.float64)
            hl2ss_3dcv.sm_mesh_normalize(mesh)
            rcs = o3d.t.geometry.RaycastingScene()
            rcs.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(sm_mesh_to_open3d_triangle_mesh(mesh)))
            surface_info = updated_surfaces[index]
            self._updated_surfaces[surface_info.id] = _sm_manager_entry(surface_info.update_time, mesh, rcs)
            
        self._load_updated_surfaces()
    
    def close(self):
        self._ipc.close()

    def get_meshes(self):
        surfaces = self._get_surfaces()
        return [surface.mesh for surface in surfaces]

    def cast_rays(self, rays):
        surfaces = self._get_surfaces()
        n = len(surfaces)
        distances = np.ones(rays.shape[0:-1] + (n if (n > 0) else 1,)) * np.inf
        for index, entry in enumerate(surfaces):
            distances[..., index] = entry.rcs.cast_rays(rays)['t_hit'].numpy()
        distances = np.min(distances, axis=-1)
        return distances


class sm_mt_manager(sm_manager):
    def open(self):
        self._lock = threading.Lock()
        self._task = None
        super().open()

    def _load_updated_surfaces(self):
        self._lock.acquire()
        super()._load_updated_surfaces()
        self._lock.release()

    def _get_surfaces(self):
        self._lock.acquire()
        surfaces = super()._get_surfaces()
        self._lock.release()
        return surfaces
    
    def get_observed_surfaces(self):
        if (self._task is not None):
            if (self._task.is_alive()):
                return
            self._task.join()
        self._task = threading.Thread(target=super().get_observed_surfaces)
        self._task.start()
    
    def close(self):
        if (self._task is not None):
            self._task.join()
        super().close()


class sm_mp_manager(mp.Process):
    IPC_STOP = 0
    IPC_SET_VOLUMES = 1
    IPC_GET_OBSERVED_SURFACES = 2
    IPC_CAST_RAYS = 3

    def __init__(self, host, triangles_per_cubic_meter, threads):
        super().__init__()
        self._semaphore = mp.Semaphore(0)
        self._din = mp.Queue()
        self._dout = mp.Queue()        
        self._ipc = sm_mt_manager(host, triangles_per_cubic_meter, threads)

    def open(self):
        self.start()

    def close(self):
        self._din.put(sm_mp_manager.IPC_STOP)
        self._semaphore.release()
        self.join()

    def set_volumes(self, volumes):
        self._din.put(sm_mp_manager.IPC_SET_VOLUMES)
        self._din.put(volumes)
        self._semaphore.release()

    def get_observed_surfaces(self):
        self._din.put(sm_mp_manager.IPC_GET_OBSERVED_SURFACES)
        self._semaphore.release()

    def cast_rays(self, rays):
        self._din.put(sm_mp_manager.IPC_CAST_RAYS)
        self._din.put(rays)
        self._semaphore.release()
        d = self._dout.get()
        return d
    
    def _set_volumes(self):
        volumes = self._din.get()
        self._ipc.set_volumes(volumes)

    def _get_observed_surfaces(self):
        self._ipc.get_observed_surfaces()

    def _cast_rays(self):
        rays = self._din.get()
        d = self._ipc.cast_rays(rays)
        self._dout.put(d)
    
    def run(self):
        self._ipc.open()

        while (True):
            self._semaphore.acquire()
            message = self._din.get()

            if (message == sm_mp_manager.IPC_STOP):
                break
            elif (message == sm_mp_manager.IPC_SET_VOLUMES):
                self._set_volumes()
            elif (message == sm_mp_manager.IPC_GET_OBSERVED_SURFACES):
                self._get_observed_surfaces()
            elif (message == sm_mp_manager.IPC_CAST_RAYS):
                self._cast_rays()

        self._ipc.close()


#------------------------------------------------------------------------------
# Scene Understanding Data Manager
#------------------------------------------------------------------------------

class su_manager:
    def __init__(self, host):
        self._enable_scene_object_quads = False
        self._enable_scene_object_meshes = True
        self._enable_only_observed_scene_objects = False
        self._create_mode = hl2ss.SU_Create.NewFromPrevious
        self._get_orientation = False
        self._get_position = False
        self._get_location_matrix = True
        self._get_quad = False
        self._get_meshes = True
        self._get_collider_meshes = False
        self._guid_list = []
        self._ipc = hl2ss_lnm.ipc_su(host, hl2ss.IPCPort.SCENE_UNDERSTANDING)

    def open(self):
        self._ipc.open()

    def configure(self, enable_world_mesh, mesh_lod, query_radius, kind_flags):
        self._enable_world_mesh = enable_world_mesh
        self._requested_mesh_level_of_detail = mesh_lod
        self._query_radius = query_radius
        self._kinds = kind_flags

    def update(self):
        self._items = {}

        task = hl2ss.su_task(
            self._enable_scene_object_quads,
            self._enable_scene_object_meshes, 
            self._enable_only_observed_scene_objects, 
            self._enable_world_mesh, 
            self._requested_mesh_level_of_detail, 
            self._query_radius, 
            self._create_mode, 
            self._kinds, 
            self._get_orientation, 
            self._get_position, 
            self._get_location_matrix, 
            self._get_quad, 
            self._get_meshes, 
            self._get_collider_meshes, 
            self._guid_list
        )
        task.pack()

        result = self._ipc.query(task)
        result.unpack()

        for item in result.items:
            item.unpack()
            for mesh in item.meshes:
                mesh.unpack()
                hl2ss_3dcv.su_normalize(mesh, item.location @ result.pose)
            self._items[item.id.hex()] = item

    def close(self):
        self._ipc.close()

    def get_items(self):
        return self._items


#------------------------------------------------------------------------------
# Custom Open3D Integrator
#------------------------------------------------------------------------------

class integrator:
    def __init__(self, voxel_size=3/512, block_resolution=16, block_count=10000, device='cpu:0'):
        self._voxel_size = float(voxel_size)
        self._block_resolution = int(block_resolution)
        self._block_count = int(block_count)
        self._device = o3d.core.Device(device)
        self._bin_size = voxel_size * block_resolution
        attr_names = ('tsdf', 'weight', 'color')
        attr_dtypes = (o3d.core.float32, o3d.core.uint16, o3d.core.uint16)
        attr_channels = ((1), (1), (3))
        self._vbg = o3d.t.geometry.VoxelBlockGrid(attr_names=attr_names, attr_dtypes=attr_dtypes, attr_channels=attr_channels, voxel_size=voxel_size, block_resolution=block_resolution, block_count=block_count, device=self._device)
        self.set_trunc()
        self.set_trunc_voxel_multiplier()

    def set_trunc(self, value=None):
        self._trunc = 4*self._voxel_size if (value is None) else float(value)

    def set_trunc_voxel_multiplier(self, value=8.0):
        self._trunc_voxel_multiplier = float(value)

    def set_depth_parameters(self, depth_scale, depth_max):
        self._depth_scale = float(depth_scale)
        self._depth_max = float(depth_max)

    def set_intrinsics(self, intrinsics):
        t = o3d.core.Tensor(intrinsics.transpose())
        self._intrinsics32 = t.to(self._device, o3d.core.float32)
        self._intrinsics64 = t.to(self._device, o3d.core.float64)

    def set_extrinsics(self, extrinsics):
        t = o3d.core.Tensor(extrinsics.transpose())
        self._extrinsics32 = t.to(self._device, o3d.core.float32)
        self._extrinsics64 = t.to(self._device, o3d.core.float64)

    def set_projection(self, projection):
        self._projection32 = o3d.core.Tensor(projection).to(self._device, o3d.core.float32)

    def set_depth(self, depth):
        self._depth = o3d.t.geometry.Image(depth).to(self._device)

    def set_color(self, color):
        self._color = o3d.t.geometry.Image(color).to(self._device)

    def extract_point_cloud(self, weight_threshold=3.0, estimated_point_number=-1):
        return self._vbg.extract_point_cloud(float(weight_threshold), int(estimated_point_number))

    def integrate(self):
        frustum_block_coords = self._vbg.compute_unique_block_coordinates(self._depth, self._intrinsics64, self._extrinsics64, self._depth_scale, self._depth_max, self._trunc_voxel_multiplier)
        self._vbg.integrate(frustum_block_coords, self._depth, self._color, self._intrinsics64, self._intrinsics64, self._extrinsics64, self._depth_scale, self._depth_max)

    def reset_weights(self, value):
        weight = self._vbg.attribute('weight').reshape((-1, 1))
        weight[:, 0] = (weight[:, 0] | (weight[:, 0] >> 1)) & 1
        #o3d.core.cuda.synchronize()

    def erase_full(self):
        buf_indices = self._vbg.hashmap().active_buf_indices()
        #o3d.core.cuda.synchronize()
        voxel_coords, voxel_indices = self._vbg.voxel_coordinates_and_flattened_indices(buf_indices)
        #o3d.core.cuda.synchronize()
        uvd = voxel_coords @ self._projection32[:3, :3] + self._projection32[3:, :3]
        d = uvd[:, 2]
        u = (uvd[:, 0] / d).round().to(o3d.core.int64)
        v = (uvd[:, 1] / d).round().to(o3d.core.int64)
        #o3d.core.cuda.synchronize()
        mask_proj = (d > 0) & (u >= 0) & (v >= 0) & (u < self._depth.columns) & (v < self._depth.rows)
        v_proj = v[mask_proj]
        u_proj = u[mask_proj]
        d_proj = d[mask_proj]
        depth_readings = self._depth.as_tensor()[v_proj, u_proj, 0]
        sdf = depth_readings - d_proj
        mask_inlier = sdf > self._trunc
        #o3d.core.cuda.synchronize()
        weight = self._vbg.attribute('weight').reshape((-1, 1))
        valid_voxel_indices = voxel_indices[mask_proj][mask_inlier]
        weight[valid_voxel_indices] = 0
        #o3d.core.cuda.synchronize()

    def erase_approximate(self):
        active_keys = self._vbg.hashmap().key_tensor()
        voxel_coords = active_keys.to(o3d.core.float32) * self._bin_size
        uvd = voxel_coords @ self._projection32[:3, :3] + self._projection32[3:, :3]
        d = uvd[:, 2]
        u = (uvd[:, 0] / d).round().to(o3d.core.int64)
        v = (uvd[:, 1] / d).round().to(o3d.core.int64)
        #o3d.core.cuda.synchronize()
        mask_proj = (d > 0) & (u >= 0) & (v >= 0) & (u < self._depth.columns) & (v < self._depth.rows)
        erase_keys = active_keys[mask_proj]
        buf_indices, masks = self._vbg.hashmap().find(erase_keys)
        #o3d.core.cuda.synchronize()
        voxel_coords, voxel_indices = self._vbg.voxel_coordinates_and_flattened_indices(buf_indices)
        #o3d.core.cuda.synchronize()
        uvd = voxel_coords @ self._projection32[:3, :3] + self._projection32[3:, :3]
        d = uvd[:, 2]
        u = (uvd[:, 0] / d).round().to(o3d.core.int64)
        v = (uvd[:, 1] / d).round().to(o3d.core.int64)
        #o3d.core.cuda.synchronize()
        mask_proj = (d > 0) & (u >= 0) & (v >= 0) & (u < self._depth.columns) & (v < self._depth.rows)
        v_proj = v[mask_proj]
        u_proj = u[mask_proj]
        d_proj = d[mask_proj]
        depth_readings = self._depth.as_tensor()[v_proj, u_proj, 0]
        sdf = depth_readings - d_proj
        mask_inlier = sdf > self._trunc
        #o3d.core.cuda.synchronize()
        weight = self._vbg.attribute('weight').reshape((-1, 1))
        valid_voxel_indices = voxel_indices[mask_proj][mask_inlier]
        weight[valid_voxel_indices] = 0
        #o3d.core.cuda.synchronize()

    def update(self):
        frustum_block_coords = self._vbg.compute_unique_block_coordinates(self._depth, self._intrinsics64, self._extrinsics64, self._depth_scale, self._depth_max)
        self._vbg.hashmap().activate(frustum_block_coords)
        buf_indices = self._vbg.hashmap().active_buf_indices()
        #o3d.core.cuda.synchronize()
        voxel_coords, voxel_indices = self._vbg.voxel_coordinates_and_flattened_indices(buf_indices)
        #o3d.core.cuda.synchronize()        
        uvd = voxel_coords @ self._projection32[:3, :3] + self._projection32[3:, :3]
        d = uvd[:, 2]
        u = (uvd[:, 0] / d).round().to(o3d.core.int64)
        v = (uvd[:, 1] / d).round().to(o3d.core.int64)
        #o3d.core.cuda.synchronize()
        mask_proj = (d > 0) & (u >= 0) & (v >= 0) & (u < self._depth.columns) & (v < self._depth.rows)
        v_proj = v[mask_proj]
        u_proj = u[mask_proj]
        d_proj = d[mask_proj]
        depth_readings = self._depth.as_tensor()[v_proj, u_proj, 0]
        color_readings = self._color.as_tensor()[v_proj, u_proj]
        sdf = depth_readings - d_proj
        mask_base   = (depth_readings > 0) & (sdf >= -self._trunc)
        mask_erase  = mask_base & (sdf > self._trunc)
        mask_update = mask_base & (sdf <= self._trunc)
        #sdf = sdf / self._trunc
        #o3d.core.cuda.synchronize()
        weight = self._vbg.attribute('weight').reshape((-1, 1))
        tsdf = self._vbg.attribute('tsdf').reshape((-1, 1))
        rgb = self._vbg.attribute('color').reshape((-1, 3))
        valid_voxel_indices = voxel_indices[mask_proj][mask_update]
        tsdf[valid_voxel_indices] = sdf[mask_update].reshape((-1, 1))
        rgb[valid_voxel_indices] = color_readings[mask_update]
        weight[valid_voxel_indices] = 1
        #o3d.core.cuda.synchronize()
        valid_voxel_indices = voxel_indices[mask_proj][mask_erase]
        weight[valid_voxel_indices] = 0
        #o3d.core.cuda.synchronize()

    def update_full(self):
        frustum_block_coords = self._vbg.compute_unique_block_coordinates(self._depth, self._intrinsics64, self._extrinsics64, self._depth_scale, self._depth_max, self._trunc_voxel_multiplier)
        self._vbg.hashmap().activate(frustum_block_coords)
        buf_indices, masks = self._vbg.hashmap().find(frustum_block_coords)
        #o3d.core.cuda.synchronize()
        voxel_coords, voxel_indices = self._vbg.voxel_coordinates_and_flattened_indices(buf_indices)
        #o3d.core.cuda.synchronize()
        uvd = voxel_coords @ self._projection32[:3, :3] + self._projection32[3:, :3]
        d = uvd[:, 2]
        u = (uvd[:, 0] / d).round().to(o3d.core.int64)
        v = (uvd[:, 1] / d).round().to(o3d.core.int64)
        #o3d.core.cuda.synchronize()
        mask_proj = (d > 0) & (u >= 0) & (v >= 0) & (u < self._depth.columns) & (v < self._depth.rows)
        v_proj = v[mask_proj]
        u_proj = u[mask_proj]
        d_proj = d[mask_proj]
        depth_readings = self._depth.as_tensor()[v_proj, u_proj, 0].to(o3d.core.float32)
        color_readings = self._color.as_tensor()[v_proj, u_proj].to(o3d.core.float32)
        sdf = depth_readings - d_proj
        mask_inlier = (depth_readings > 0) & (sdf >= -self._trunc) & (depth_readings < self._depth_max) 
        sdf[sdf >= self._trunc] = self._trunc
        sdf = sdf / self._trunc
        #o3d.core.cuda.synchronize()
        weight = self._vbg.attribute('weight').reshape((-1, 1))
        tsdf = self._vbg.attribute('tsdf').reshape((-1, 1))
        rgb = self._vbg.attribute('color').reshape((-1, 3))
        valid_voxel_indices = voxel_indices[mask_proj][mask_inlier]
        tsdf[valid_voxel_indices] = sdf[mask_inlier].reshape((-1, 1))
        rgb[valid_voxel_indices] = color_readings[mask_inlier]
        weight[valid_voxel_indices] = 1
        #o3d.core.cuda.synchronize()
        active_keys = self._vbg.hashmap().key_tensor()
        voxel_coords = active_keys.to(o3d.core.float32) * self._bin_size
        uvd = voxel_coords @ self._projection32[:3, :3] + self._projection32[3:, :3]
        d = uvd[:, 2]
        u = (uvd[:, 0] / d).round().to(o3d.core.int64)
        v = (uvd[:, 1] / d).round().to(o3d.core.int64)
        #o3d.core.cuda.synchronize()
        mask_proj = (d > 0) & (u >= 0) & (v >= 0) & (u < self._depth.columns) & (v < self._depth.rows)
        erase_keys = active_keys[mask_proj]
        buf_indices, masks = self._vbg.hashmap().find(erase_keys)
        #o3d.core.cuda.synchronize()
        voxel_coords, voxel_indices = self._vbg.voxel_coordinates_and_flattened_indices(buf_indices)
        #o3d.core.cuda.synchronize()
        uvd = voxel_coords @ self._projection32[:3, :3] + self._projection32[3:, :3]
        d = uvd[:, 2]
        u = (uvd[:, 0] / d).round().to(o3d.core.int64)
        v = (uvd[:, 1] / d).round().to(o3d.core.int64)
        #o3d.core.cuda.synchronize()
        mask_proj = (d > 0) & (u >= 0) & (v >= 0) & (u < self._depth.columns) & (v < self._depth.rows)
        v_proj = v[mask_proj]
        u_proj = u[mask_proj]
        d_proj = d[mask_proj]
        depth_readings = self._depth.as_tensor()[v_proj, u_proj, 0]
        sdf = depth_readings - d_proj
        mask_inlier = sdf > self._trunc
        #o3d.core.cuda.synchronize()
        weight = self._vbg.attribute('weight').reshape((-1, 1))
        valid_voxel_indices = voxel_indices[mask_proj][mask_inlier]
        weight[valid_voxel_indices] = 0
        #o3d.core.cuda.synchronize()

