
using System;
using UnityEngine;

public class test_ipc_su : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        hl2ss.svc.open_ipc(run_once.host_address, hl2ss.ipc_port.SCENE_UNDERSTANDING, out hl2ss.shared.ipc_su ipc);

        var task = new hl2ss.su_task();

        task.enable_quads = true;
        task.enable_meshes = true;
        task.enable_only_observed = true;
        task.enable_world_mesh = true;
        task.mesh_lod = hl2ss.su_mesh_lod.Fine;
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

        using var result = ipc.query(task);

        if (result.status == 0)
        {
            Debug.Log(string.Format("extrinsics [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", result.extrinsics.m_00, result.extrinsics.m_01, result.extrinsics.m_02, result.extrinsics.m_03, result.extrinsics.m_10, result.extrinsics.m_11, result.extrinsics.m_12, result.extrinsics.m_13, result.extrinsics.m_20, result.extrinsics.m_21, result.extrinsics.m_22, result.extrinsics.m_23, result.extrinsics.m_30, result.extrinsics.m_31, result.extrinsics.m_32, result.extrinsics.m_33));
            Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", result.pose.m_00, result.pose.m_01, result.pose.m_02, result.pose.m_03, result.pose.m_10, result.pose.m_11, result.pose.m_12, result.pose.m_13, result.pose.m_20, result.pose.m_21, result.pose.m_22, result.pose.m_23, result.pose.m_30, result.pose.m_31, result.pose.m_32, result.pose.m_33));

            Debug.Log(string.Format("got {0}/{1} objects", result.items.Length, result.items_count));

            for (uint i = 0; i < result.items_count; ++i)
            {
                Debug.Log(string.Format("item_id: {0:X16}{1:X16}", result.items[i].id.h, result.items[i].id.l));
                Debug.Log(string.Format("item_kind: {0}", result.items[i].kind));
                Debug.Log(string.Format("item_alignment: {0}", result.items[i].alignment));
                Debug.Log(string.Format("item_extents: [{0}, {1}]", result.items[i].extents.x, result.items[i].extents.y));
                Debug.Log(string.Format("item_location: [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", result.items[i].location.m_00, result.items[i].location.m_01, result.items[i].location.m_02, result.items[i].location.m_03, result.items[i].location.m_10, result.items[i].location.m_11, result.items[i].location.m_12, result.items[i].location.m_13, result.items[i].location.m_20, result.items[i].location.m_21, result.items[i].location.m_22, result.items[i].location.m_23, result.items[i].location.m_30, result.items[i].location.m_31, result.items[i].location.m_32, result.items[i].location.m_33));
                Debug.Log(string.Format("item_orientation: [{0}, {1}, {2}, {3}]", result.items[i].orientation.x, result.items[i].orientation.y, result.items[i].orientation.z, result.items[i].orientation.w));
                Debug.Log(string.Format("item_position: [{0}, {1}, {2}]", result.items[i].position.x, result.items[i].position.y, result.items[i].position.z));
                Debug.Log(string.Format("meshes_count: {0}", result.items[i].meshes_count));
                Debug.Log(string.Format("collider_meshes_count: {0}", result.items[i].collider_meshes_count));

                for (uint j = 0; j < result.items[i].meshes_count; ++j)
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

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
