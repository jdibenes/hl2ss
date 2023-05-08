
import multiprocessing as mp
import threading
import numpy as np
import open3d as o3d
import hl2ss
import hl2ss_3dcv


class _sm_manager_entry:
    def __init__(self, update_time, mesh, rcs):
        self.update_time = update_time
        self.mesh = mesh
        self.rcs = rcs


class sm_manager:
    def __init__(self, host, triangles_per_cubic_meter, threads):
        self._tpcm = triangles_per_cubic_meter
        self._threads = threads
        self._vpf = hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized
        self._tif = hl2ss.SM_TriangleIndexFormat.R16UInt
        self._vnf = hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized
        self._normals = False
        self._bounds = False
        self._ipc = hl2ss.ipc_sm(host, hl2ss.IPCPort.SPATIAL_MAPPING)
        self._surfaces = {}
        self._volumes = None

    def open(self):
        self._ipc.open()
        self._ipc.create_observer()

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
            tasks.add_task(id, self._tpcm, self._vpf, self._tif, self._vnf, self._normals, self._bounds)
            updated_surfaces.append(surface_info)

        count = len(updated_surfaces)
        if (count <= 0):
            return

        for index, mesh in self._ipc.get_meshes(tasks, self._threads).items():
            if (mesh is None):
                continue
            mesh.unpack(self._vpf, self._tif, self._vnf)
            hl2ss_3dcv.sm_mesh_cast(mesh, np.float64, np.uint32, np.float64)
            hl2ss_3dcv.sm_mesh_normalize(mesh)
            rcs = o3d.t.geometry.RaycastingScene()
            rcs.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(hl2ss_3dcv.sm_mesh_to_open3d_triangle_mesh(mesh)))
            surface_info = updated_surfaces[index]
            self._updated_surfaces[surface_info.id] = _sm_manager_entry(surface_info.update_time, mesh, rcs)
            
        self._load_updated_surfaces()
    
    def close(self):
        self._ipc.close()

    def cast_rays(self, rays):
        surfaces = self._get_surfaces()
        distances = np.ones(rays.shape[0:-1] + (len(surfaces),)) * np.inf
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

