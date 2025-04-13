
import multiprocessing as mp
import threading as mt
import numpy as np
import open3d as o3d
import traceback
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


class sm_manager(hl2ss._context_manager):
    def __init__(self, host, port, sockopt=None, triangles_per_cubic_meter=1000, vpf=hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized, tif=hl2ss.SM_TriangleIndexFormat.R16UInt, vnf=hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized):
        self._tpcm = triangles_per_cubic_meter
        self._vpf = vpf
        self._tif = tif
        self._vnf = vnf
        self._ipc = hl2ss_lnm.ipc_sm(host, port, sockopt)
        self._surfaces = {}
        self._volumes = None
        self._updated = False

    def open(self):
        self._ipc.open()

    def set_volumes(self, volumes):
        self._set_volumes(volumes)

    def _set_volumes(self, volumes):
        self._volumes = volumes

    def _get_volumes(self):
        v, self._volumes = self._volumes, None
        return v

    def _set_surfaces(self, surfaces):
        self._surfaces = surfaces
        self._updated = True

    def _get_surfaces(self):
        return self._surfaces.values()

    def _get_updated_flag(self):
        f, self._updated = self._updated, False
        return f

    def get_observed_surfaces(self):
        next_surfaces = {}
        tasks = hl2ss.sm_mesh_task()        
        updated_surfaces = []

        next_volumes = self._get_volumes()
        if (next_volumes is not None):
            self._ipc.set_volumes(next_volumes)
        
        for surface_info in self._ipc.get_observed_surfaces():
            id = surface_info.id
            surface_info.id = surface_info.id.hex()
            if (surface_info.id in self._surfaces):
                previous_entry = self._surfaces[surface_info.id]
                if (surface_info.update_time <= previous_entry.update_time):
                    next_surfaces[surface_info.id] = previous_entry
                    continue
            tasks.add_task(id, self._tpcm, self._vpf, self._tif, self._vnf)
            updated_surfaces.append(surface_info)

        count = len(updated_surfaces)
        if (count <= 0):
            return

        for index, mesh in self._ipc.get_meshes(tasks).items():
            if (mesh is None):
                continue
            hl2ss_3dcv.sm_mesh_cast(mesh, np.float64, np.uint32, np.float64)
            hl2ss_3dcv.sm_mesh_normalize(mesh)
            rcs = o3d.t.geometry.RaycastingScene()
            rcs.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(sm_mesh_to_open3d_triangle_mesh(mesh)))
            surface_info = updated_surfaces[index]
            next_surfaces[surface_info.id] = _sm_manager_entry(surface_info.update_time, mesh, rcs)
            
        self._set_surfaces(next_surfaces)
    
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
    
    def get_updated_flag(self):
        return self._get_updated_flag()


class sm_manager_mt(sm_manager):
    def open(self):
        self._lock = mt.Lock()
        self._ipc_status = mt.Event()
        self._ipc_string = None
        self._task = None
        super().open()

    def _set_volumes(self, volumes):
        with self._lock:
            super()._set_volumes(volumes)

    def _get_volumes(self):
        with self._lock:
            return super()._get_volumes()

    def _set_surfaces(self, surfaces):
        with self._lock:
            super()._set_surfaces(surfaces)

    def _get_surfaces(self):
        with self._lock:
            return super()._get_surfaces()

    def _get_updated_flag(self):
        with self._lock:
            return super()._get_updated_flag()
    
    def _get_observed_surfaces(self):
        try:
            super().get_observed_surfaces()
        except:
            self._ipc_string = traceback.format_exc()
            self._ipc_status.set()

    def _cleanup_task(self, wait):
        if (self._task is not None):
            if ((not wait) and self._task.is_alive()):
                return False
            self._task.join()
            self._task = None
        return True

    def get_observed_surfaces(self):
        if (self._cleanup_task(False) and self.get_ipc_status()):
            self._task = mt.Thread(target=self._get_observed_surfaces)
            self._task.start()

    def get_ipc_status(self):
        return not self._ipc_status.is_set()
    
    def get_ipc_string(self):
        return self._ipc_string
    
    def close(self):
        self._cleanup_task(True)
        super().close()


class _sm_manager_stub:
    def __init__(self, host=None, port=None, sockopt=None, triangles_per_cubic_meter=1000, vpf=hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized, tif=hl2ss.SM_TriangleIndexFormat.R16UInt, vnf=hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized):
        pass

    def open(self):
        pass

    def set_volumes(self, volumes):
        pass

    def get_observed_surfaces(self):
        pass

    def close(self):
        pass

    def get_meshes(self):
        return []

    def cast_rays(self, rays):
        n = 0
        distances = np.ones(rays.shape[0:-1] + (n if (n > 0) else 1,)) * np.inf
        distances = np.min(distances, axis=-1)
        return distances
    
    def get_updated_flag(self):
        return False
    
    def get_ipc_status(self):
        return self._ipc_status

    def get_ipc_string(self):
        return self._ipc_string
    
    def set_ipc_status(self, status, string):
        self._ipc_status = status
        self._ipc_string = string


class _sm_manager_mp(mp.Process):
    IPC_STOP = 0
    IPC_SET_VOLUMES = 1
    IPC_GET_OBSERVED_SURFACES = 2
    IPC_GET_MESHES = 3
    IPC_CAST_RAYS = 4
    IPC_GET_IPC_STRING = 5
    IPC_GET_UPDATED_FLAG = 6

    def __init__(self, host, port, sockopt=None, triangles_per_cubic_meter=1000, vpf=hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized, tif=hl2ss.SM_TriangleIndexFormat.R16UInt, vnf=hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized):
        super().__init__()
        self._din = mp.Queue()
        self._dout = mp.Queue()        
        self._event = mp.Event()
        self._host = host
        self._port = port
        self._sockopt = sockopt
        self._tpcm = triangles_per_cubic_meter
        self._vpf = vpf
        self._tif = tif
        self._vnf = vnf

    def stop(self):
        self._din.put((_sm_manager_mp.IPC_STOP,))

    def set_volumes(self, volumes):
        self._din.put((_sm_manager_mp.IPC_SET_VOLUMES, volumes))

    def get_observed_surfaces(self):
        self._din.put((_sm_manager_mp.IPC_GET_OBSERVED_SURFACES,))

    def get_meshes(self):
        self._din.put((_sm_manager_mp.IPC_GET_MESHES,))
        return self._dout.get()

    def cast_rays(self, rays):
        self._din.put((_sm_manager_mp.IPC_CAST_RAYS, rays))
        return self._dout.get()
    
    def get_updated_flag(self):
        self._din.put((_sm_manager_mp.IPC_GET_UPDATED_FLAG,))
        return self._dout.get()
    
    def get_ipc_status(self):
        return not self._event.is_set()
    
    def get_ipc_string(self):
        self._din.put((_sm_manager_mp.IPC_GET_IPC_STRING,))
        return self._dout.get()
    
    def _set_volumes(self, volumes):
        self._ipc.set_volumes(volumes)

    def _get_observed_surfaces(self):
        self._ipc.get_observed_surfaces()
        if (self._event.is_set() or self._ipc.get_ipc_status()):
            return
        self._event.set()

    def _get_meshes(self):
        return self._ipc.get_meshes()

    def _cast_rays(self, rays):
        return self._ipc.cast_rays(rays)        
    
    def _get_updated_flag(self):
        return self._ipc.get_updated_flag()

    def _get_ipc_string(self):
        return self._ipc.get_ipc_string()
    
    def _process_ipc(self):
        message = self._din.get()

        if   (message[0] == _sm_manager_mp.IPC_STOP):
            return False
        elif (message[0] == _sm_manager_mp.IPC_SET_VOLUMES):
            self._set_volumes(*message[1:])
        elif (message[0] == _sm_manager_mp.IPC_GET_OBSERVED_SURFACES):
            self._get_observed_surfaces()
        elif (message[0] == _sm_manager_mp.IPC_GET_MESHES):
            self._dout.put(self._get_meshes())
        elif (message[0] == _sm_manager_mp.IPC_CAST_RAYS):
            self._dout.put(self._cast_rays(*message[1:]))
        elif (message[0] == _sm_manager_mp.IPC_GET_UPDATED_FLAG):
            self._dout.put(self._get_updated_flag())
        elif (message[0] == _sm_manager_mp.IPC_GET_IPC_STRING):
            self._dout.put(self._get_ipc_string())

        return True
        
    def run(self):
        self._ipc = sm_manager_mt(self._host, self._port, self._sockopt, self._tpcm, self._vpf, self._tif, self._vnf)

        try:
            self._ipc.open()
        except:
            self._ipc.close()
            self._ipc = _sm_manager_stub()
            self._ipc.set_ipc_status(False, traceback.format_exc())
            self._ipc.open()

        while (self._process_ipc()):
            pass

        self._ipc.close()


class sm_manager_mp:
    def __init__(self, host, port, sockopt=None, triangles_per_cubic_meter=1000, vpf=hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized, tif=hl2ss.SM_TriangleIndexFormat.R16UInt, vnf=hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized):
        self._ipc = _sm_manager_mp(host, port, sockopt, triangles_per_cubic_meter, vpf, tif, vnf)

    def open(self):
        self._ipc.start()

    def set_volumes(self, volumes):
        self._ipc.set_volumes(volumes)

    def get_observed_surfaces(self):
        self._ipc.get_observed_surfaces()

    def get_meshes(self):
        return self._ipc.get_meshes()

    def cast_rays(self, rays):
        return self._ipc.cast_rays(rays)
    
    def get_updated_flag(self):
        return self._ipc.get_updated_flag()
    
    def get_ipc_status(self):
        return self._ipc.get_ipc_status()
    
    def get_ipc_string(self):
        return self._ipc.get_ipc_string()

    def close(self):
        self._ipc.stop()
        self._ipc.join()


#------------------------------------------------------------------------------
# Scene Understanding Data Manager
#------------------------------------------------------------------------------

class su_manager:
    def __init__(self, host, port, sockopt=None):
        self._ipc = hl2ss_lnm.ipc_su(host, port, sockopt)

    def open(self):
        self._ipc.open()

    def update(self, task):
        self._items = {}
        result = self._ipc.query(task)
        for item in result.items:
            for mesh in (item.meshes + item.collider_meshes):
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

