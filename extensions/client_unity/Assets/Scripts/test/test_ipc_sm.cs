
using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class test_ipc_sm : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.sm_bounding_volume volumes = new hl2ss.sm_bounding_volume();
        hl2ss.sm_mesh_task tasks = new hl2ss.sm_mesh_task();

        hl2ss.sm_sphere sphere = new hl2ss.sm_sphere();

        sphere.center.x = 0.0f;
        sphere.center.y = 0.0f;
        sphere.center.z = 0.0f;
        sphere.radius = 8.0f;

        volumes.add_sphere(sphere);

        Debug.Log(string.Format("{0}, {1}, {2}", volumes.get_count(), volumes.get_data().Length, volumes.get_size()));

        hl2ss.svc.open_ipc(host, hl2ss.ipc_port.SPATIAL_MAPPING, out hl2ss.svc.ipc_sm ipc);

        ipc.set_volumes(volumes);

        using (var surfaces = ipc.get_observed_surfaces())
        {
            Debug.Log(string.Format("got {0} surfaces", surfaces.size));

            for (ulong i = 0; i < surfaces.size; ++i)
            {
                hl2ss.sm_surface_info info = Marshal.PtrToStructure<hl2ss.sm_surface_info>(IntPtr.Add(surfaces.data, (int)i * Marshal.SizeOf<hl2ss.sm_surface_info>()));
                tasks.add_task(info.id, 1000.0f, hl2ss.sm_vertex_position_format.R32G32B32A32Float, hl2ss.sm_triangle_index_format.R32Uint, hl2ss.sm_vertex_normal_format.R32G32B32A32Float, true, false);
            }

            using (var result = ipc.get_meshes(tasks))
            {
                Debug.Log(string.Format("got {0} meshes", result.meshes.Length));

                for (ulong i = 0; i < surfaces.size; ++i)
                {
                    Debug.Log(string.Format("mesh index: {0}", i));
                    Debug.Log(string.Format("mesh status: {0}", result.meshes[i].status));
                    Debug.Log(string.Format("mesh vertices: {0}", result.meshes[i].vertex_positions_size / (4 * sizeof(float))));
                    Debug.Log(string.Format("mesh triangles: {0}", result.meshes[i].triangle_indices_size / (3 * sizeof(uint))));
                    Debug.Log(string.Format("mesh normals: {0}", result.meshes[i].vertex_normals_size / (4 * sizeof(float))));
                    Debug.Log(string.Format("mesh scale: [{0}, {1}, {2}]", result.meshes[i].vertex_position_scale.x, result.meshes[i].vertex_position_scale.y, result.meshes[i].vertex_position_scale.z));
                }
            }
        }

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
