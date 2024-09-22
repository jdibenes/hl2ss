
using System;
using UnityEngine;

public class test_ipc_su : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.open_ipc(host, hl2ss.ipc_port.SCENE_UNDERSTANDING, out hl2ss.svc.ipc_su ipc);

        hl2ss.su_task task = new hl2ss.su_task();

        task.enable_quads = true;
        task.enable_meshes = true;
        task.enable_only_observed = true;
        task.enable_world_mesh = true;
        task.mesh_lod = hl2ss.su_mesh_lod.Medium;
        task.query_radius = 5.0f;
        task.create_mode = hl2ss.su_create.New;
        task.kind_flags = hl2ss.su_kind_flag.Ceiling | hl2ss.su_kind_flag.Floor | hl2ss.su_kind_flag.Platform | hl2ss.su_kind_flag.Wall | hl2ss.su_kind_flag.World;
        task.get_orientation = true;
        task.get_position = true;
        task.get_location_matrix = true;
        task.get_quad = true;
        task.get_meshes = true;
        task.get_collider_meshes = true;
        task.guid_list = Array.Empty<hl2ss.guid>();

        using (var result = ipc.query(task))
        {
            if (result.header.status == 0)
            {
                Debug.Log(string.Format("got {0}/{1} objects", result.items.Length, result.header.count));
                for (uint i = 0; i < result.header.count; ++i)
                {
                    Debug.Log(string.Format("item_kind: {0}", result.items[i].content.kind));
                    Debug.Log(string.Format("item_alignment: {0}", result.items[i].content.alignment));
                    Debug.Log(string.Format("meshes_count: {0}", result.items[i].content.meshes_count));
                    Debug.Log(string.Format("collider_meshes_count: {0}", result.items[i].content.collider_meshes_count));

                    for (uint j = 0; j < result.items[i].content.meshes_count; ++j)
                    {
                        Debug.Log(string.Format("mesh_vertices {0}", result.items[i].unpacked_meshes[j].vertex_positions_size / (3 * sizeof(uint))));
                        Debug.Log(string.Format("mesh_triangles {0}", result.items[i].unpacked_meshes[j].triangle_indices_size / (3 * sizeof(uint))));
                    }
                }
            }
            else
            {
                Debug.Log("su query failed");
            }
        }

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
