
#include <Windows.h>
#include "server.h"
#include "ports.h"
#include "scene_understanding.h"
#include "log.h"

#define URI_STATIC_BUILD
#include "uriparser/Uri.h"

#include "hl2ss_network.h"

#include "pcpd_msgs/rpc/Hololens2SceneUnderstanding.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <Microsoft.MixedReality.SceneUnderstanding.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace Microsoft::MixedReality::SceneUnderstanding;

struct Task
{
    SceneQuerySettings sqs;
    float query_radius;
    bool use_previous;
    uint8_t kind_flags;
    bool get_orientation;
    bool get_position;
    bool get_location_matrix;
    bool get_quad;
    bool get_meshes;
    bool get_collider_meshes;
    uint32_t guid_count;
};

struct SU_Context {
    std::string client_id;
    z_session_t session;
    bool valid{ false };
};


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL;
static HANDLE g_event_quit = NULL;
static HANDLE g_event_client = NULL;

static SU_Context* g_zenoh_context = NULL;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------


// OK
static void SU_TransferMeshes(
    std::vector<pcpd_msgs::rpc::HL2SUSceneUnderstanding_Mesh>& output, 
    std::vector<std::shared_ptr<SceneMesh>> const& meshes)
{
    uint32_t const vertex_fields = 3;
    uint32_t count = (uint32_t)meshes.size();
    uint32_t count_vertices;
    uint32_t count_vertices_fields;
    uint32_t count_triangle_indices;
    uint32_t* data_vertices;
    uint32_t* data_triangle_indices;
    std::vector<uint32_t> buffer_vertices;
    std::vector<uint32_t> buffer_triangles;

    output.resize(count);
    for (unsigned i=0; i < count; ++i)
    {
        auto& mesh = meshes.at(i);
        auto& t = output.at(i);
        t.vertex_fields(vertex_fields);

        count_vertices = mesh->GetVertexCount();
        t.count_vertices(count_vertices);

        count_triangle_indices = mesh->GetTriangleIndexCount();
        t.count_triangle_indices(count_triangle_indices);

        count_vertices_fields = count_vertices * vertex_fields;

        buffer_vertices.resize(count_vertices_fields);
        buffer_triangles.resize(count_triangle_indices);

        data_vertices = buffer_vertices.data();
        data_triangle_indices = buffer_triangles.data();

        mesh->GetVertexPositions(data_vertices, count_vertices);
        mesh->GetTriangleIndices({ data_triangle_indices, count_triangle_indices });

        t.vertices(buffer_vertices);
        t.triangle_indices(buffer_triangles);
    }
}

// OK
struct SU_SpatialQuery {
    using RequestT = pcpd_msgs::rpc::HL2SURequest_SpatialQuery;
    using ResponseT = pcpd_msgs::rpc::HL2SUResponse_SpatialQuery;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {

        Task task;
        std::vector<GUID> guids;
        SceneUnderstanding_Result const* result;
        std::shared_ptr<SceneObject> item;
        std::shared_ptr<SceneQuad> quad;
        GUID item_id;
        SceneObjectKind item_kind;
        Quaternion item_orientation;
        Matrix4x4 item_location;
        Vector3 item_position;
        SceneQuadAlignment item_alignment;
        Vector2 item_extents;

        task.sqs.EnableSceneObjectQuads = request.sqs().enable_scene_object_quads();
        task.sqs.EnableSceneObjectMeshes = request.sqs().enable_scene_object_meshes();
        task.sqs.EnableOnlyObservedSceneObjects = request.sqs().enable_only_observed_scene_objects();
        task.sqs.EnableWorldMesh = request.sqs().enable_world_mesh();
        switch (request.sqs().requested_mesh_level_of_detail()) {
        case pcpd_msgs::rpc::SMLOD_Coarse:
            task.sqs.RequestedMeshLevelOfDetail = Microsoft::MixedReality::SceneUnderstanding::SceneMeshLevelOfDetail::Coarse;
            break;
        case pcpd_msgs::rpc::SMLOD_Medium:
            task.sqs.RequestedMeshLevelOfDetail = Microsoft::MixedReality::SceneUnderstanding::SceneMeshLevelOfDetail::Medium;
            break;
        case pcpd_msgs::rpc::SMLOD_Fine:
            task.sqs.RequestedMeshLevelOfDetail = Microsoft::MixedReality::SceneUnderstanding::SceneMeshLevelOfDetail::Fine;
            break;
        case pcpd_msgs::rpc::SMLOD_Unlimited:
            task.sqs.RequestedMeshLevelOfDetail = Microsoft::MixedReality::SceneUnderstanding::SceneMeshLevelOfDetail::Unlimited;
            break;
        }

        task.query_radius = request.query_radius();
        task.use_previous = request.use_previous();
        task.kind_flags = request.kind_flags();
        task.get_orientation = request.get_orientation();
        task.get_position = request.get_position();
        task.get_location_matrix = request.get_location_matrix();
        task.get_quad = request.get_quad();
        task.get_meshes = request.get_meshes();
        task.get_collider_meshes = request.get_collider_meshes();
        
        for (auto& s : request.guids()) {
            GUID t{};
            t.Data1 = s.Data1();
            t.Data2 = s.Data2();
            t.Data3 = s.Data3();
            t.Data4[0] = s.Data4()[0];
            t.Data4[1] = s.Data4()[1];
            t.Data4[2] = s.Data4()[2];
            t.Data4[3] = s.Data4()[3];
            guids.push_back(std::move(t));
        }

        SceneUnderstanding_Query(task.sqs, task.query_radius, task.use_previous, guids.data(), guids.size(), task.kind_flags);
        result = SceneUnderstanding_WaitForResult();

        {
            auto& t = response.result();
            auto& s = *result;

            switch (s.status) {
            case Microsoft::MixedReality::SceneUnderstanding::Status::OK:
                t.status(pcpd_msgs::rpc::SUS_OK);
                break;
            case Microsoft::MixedReality::SceneUnderstanding::Status::Failed:
                t.status(pcpd_msgs::rpc::SUS_Failed);
                break;
            case Microsoft::MixedReality::SceneUnderstanding::Status::ObjectDisposed:
                t.status(pcpd_msgs::rpc::SUS_ObjectDisposed);
                break;
            case Microsoft::MixedReality::SceneUnderstanding::Status::OutOfMemory:
                t.status(pcpd_msgs::rpc::SUS_OutOfMemory);
                break;
            case Microsoft::MixedReality::SceneUnderstanding::Status::InvalidArgument:
                t.status(pcpd_msgs::rpc::SUS_InvalidArgument);
                break;
            case Microsoft::MixedReality::SceneUnderstanding::Status::OutOfRange:
                t.status(pcpd_msgs::rpc::SUS_OutOfRange);
                break;
            case Microsoft::MixedReality::SceneUnderstanding::Status::NotImplemented:
                t.status(pcpd_msgs::rpc::SUS_NotImplemented);
                break;
            case Microsoft::MixedReality::SceneUnderstanding::Status::KeyNotFound:
                t.status(pcpd_msgs::rpc::SUS_KeyNotFound);
                break;
            }

            pcpd_msgs::rpc::float__16 extrinsics;
            extrinsics[0] = s.extrinsics.M11;
            extrinsics[1] = s.extrinsics.M21;
            extrinsics[2] = s.extrinsics.M31;
            extrinsics[3] = s.extrinsics.M41;

            extrinsics[4] = s.extrinsics.M12;
            extrinsics[5] = s.extrinsics.M22;
            extrinsics[6] = s.extrinsics.M32;
            extrinsics[7] = s.extrinsics.M42;

            extrinsics[8] = s.extrinsics.M13;
            extrinsics[9] = s.extrinsics.M23;
            extrinsics[10] = s.extrinsics.M33;
            extrinsics[11] = s.extrinsics.M43;

            extrinsics[12] = s.extrinsics.M14;
            extrinsics[13] = s.extrinsics.M24;
            extrinsics[14] = s.extrinsics.M34;
            extrinsics[15] = s.extrinsics.M44;

            t.extrinsics(std::move(extrinsics));

            {
                float3 scale;
                quaternion rotation;
                float3 translation;
                // here we have another scale - probably identity ..
                if (decompose(s.pose, &scale, &rotation, &translation)) {
                    t.position().x(translation.x);
                    t.position().y(translation.y);
                    t.position().z(translation.z);

                    t.orientation().x(rotation.x);
                    t.orientation().y(rotation.y);
                    t.orientation().z(rotation.z);
                    t.orientation().w(rotation.w);
                }
            }

        }

        std::vector<pcpd_msgs::rpc::HL2SUSceneUnderstanding_SpatialQueryResult> sqrs(result->selected.size());
        for (int i = 0; i < result->selected.size(); ++i)
        {
            auto& t = sqrs.at(i);
            item = result->selected[i];
            if (!item) {
                continue;
            }

            item_id = item->GetId();
            t.item_id().Data1(item_id.Data1);
            t.item_id().Data2(item_id.Data2);
            t.item_id().Data3(item_id.Data3);
            t.item_id().Data4({ item_id.Data4[0], item_id.Data4[1], item_id.Data4[2], item_id.Data4[3] });

            item_kind = item->GetKind();
            switch (item_kind) {
            case SceneObjectKind::Background:
                t.item_kind(pcpd_msgs::rpc::SKO_Background);
                break;
            case SceneObjectKind::Wall:
                t.item_kind(pcpd_msgs::rpc::SKO_Wall);
                break;
            case SceneObjectKind::Floor:
                t.item_kind(pcpd_msgs::rpc::SKO_Floor);
                break;
            case SceneObjectKind::Ceiling:
                t.item_kind(pcpd_msgs::rpc::SKO_Ceiling);
                break;
            case SceneObjectKind::Platform:
                t.item_kind(pcpd_msgs::rpc::SKO_Platform);
                break;
            case SceneObjectKind::Unknown:
                t.item_kind(pcpd_msgs::rpc::SKO_Unknown);
                break;
            case SceneObjectKind::World:
                t.item_kind(pcpd_msgs::rpc::SKO_World);
                break;
            case SceneObjectKind::CompletelyInferred:
                t.item_kind(pcpd_msgs::rpc::SKO_CompletelyInferred);
                break;
            }

            if (task.get_orientation) { 
                item_orientation = item->GetOrientation(); 
                t.orientation().x(item_orientation.X);
                t.orientation().y(item_orientation.Y);
                t.orientation().z(item_orientation.Z);
                t.orientation().w(item_orientation.W);
            }
            if (task.get_position) { 
                item_position = item->GetPosition(); 
                t.position().x(item_position.X);
                t.position().y(item_position.Y);
                t.position().z(item_position.Z);
            }
            if (task.get_location_matrix) { 
                item_location = item->GetLocationAsMatrix();
                pcpd_msgs::rpc::float__16 location;
                location[0] = item_location.M11;
                location[1] = item_location.M21;
                location[2] = item_location.M31;
                location[3] = item_location.M41;

                location[4] = item_location.M12;
                location[5] = item_location.M22;
                location[6] = item_location.M32;
                location[7] = item_location.M42;

                location[8] =  item_location.M13;
                location[9] =  item_location.M23;
                location[10] = item_location.M33;
                location[11] = item_location.M43;

                location[12] = item_location.M14;
                location[13] = item_location.M24;
                location[14] = item_location.M34;
                location[15] = item_location.M44;

                t.location_matrix(std::move(location));

            }
            if (task.get_quad) {
                quad = item->GetQuad();
                item_alignment = quad ? quad->GetAlignment() : SceneQuadAlignment::NonOrthogonal;
                switch (item_alignment) {
                case SceneQuadAlignment::NonOrthogonal:
                    t.item_alignment(pcpd_msgs::rpc::HL2SUSceneUnderstanding_SceneQuadAlignment::SQA_NonOrthogonal);
                    break;
                case SceneQuadAlignment::Horizontal:
                    t.item_alignment(pcpd_msgs::rpc::HL2SUSceneUnderstanding_SceneQuadAlignment::SQA_Horizontal);
                    break;
                case SceneQuadAlignment::Vertical:
                    t.item_alignment(pcpd_msgs::rpc::HL2SUSceneUnderstanding_SceneQuadAlignment::SQA_Vertical);
                    break;
                }

                item_extents = quad ? quad->GetExtents() : Vector2{ 0, 0 };
                t.item_extents({ item_extents.X, item_extents.Y });
            }

            if (task.get_meshes) { 
                SU_TransferMeshes(t.meshes(), item->GetMeshes());
            }
            if (task.get_collider_meshes) { 
                SU_TransferMeshes(t.collider_meshes(), item->GetColliderMeshes());
            }
        }

        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};


template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2SURequest_SpatialQuery> {
    bool parse(const std::map<std::string, std::string>& /*args*/, pcpd_msgs::rpc::HL2SURequest_SpatialQuery& /*request*/) {
        ShowMessage("SU: SpatialQuery without payload is not suported.");
        return false;
    }
};

void SU_QueryHandler(const z_query_t* query, void* context) {
    z_owned_str_t keystr = z_keyexpr_to_string(z_query_keyexpr(query));
    z_bytes_t pred = z_query_parameters(query);
    z_value_t payload_value = z_query_value(query);

    std::map<std::string, std::string> arguments;

    // parse query parameters
    UriQueryListA* queryList = NULL;
    int itemCount = 0;
    const int res = uriDissectQueryMallocA(&queryList, &itemCount,
        reinterpret_cast<const char*>(pred.start), reinterpret_cast<const char*>(pred.start + pred.len));
    if (res == URI_SUCCESS) {
        UriQueryListA* current = queryList;
        while (current != nullptr) {
            if (current->key != nullptr && current->value != nullptr) {
                arguments.insert(std::pair(current->key, current->value));
                ShowMessage("Received argument: %s -> %s", current->key, current->value);
            }
            current = current->next;
        }
    }
    uriFreeQueryListA(queryList);

    eprosima::fastcdr::FastBuffer result_buffer{};
    std::size_t result_bytes{0};
    bool call_success{ false };

    call_success = forward_rpc_call(SU_SpatialQuery{}, nullptr, payload_value, arguments, result_buffer, result_bytes);

    if (!call_success) {
        if (payload_value.payload.len > 0) {
            ShowMessage(">> [Queryable ] Received unhandled Query '%s?%.*s' with value '%.*s'\n", z_loan(keystr), (int)pred.len,
                pred.start, (int)payload_value.payload.len, payload_value.payload.start);
        }
        else {
            ShowMessage(">> [Queryable ] Received unhandled Query '%s?%.*s'\n", z_loan(keystr), (int)pred.len, pred.start);
        }
    }

    z_query_reply_options_t options = z_query_reply_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
    z_query_reply(query, z_keyexpr((const char*)context), (const uint8_t*)result_buffer.getBuffer(), result_bytes, &options);
    z_drop(z_move(keystr));
}


// OK
static DWORD WINAPI SU_EntryPoint(void *param)
{
    if (g_zenoh_context == NULL || !g_zenoh_context->valid) {
        ShowMessage("SU: Invalid Zenoh Context");
        return 1;
    }

    (void)param;

    ShowMessage("SU: Waiting for consent");

    SceneUnderstanding_WaitForConsent();

    std::string keyexpr_str = "hl2/rpc/su/" + g_zenoh_context->client_id;
    ShowMessage("SU: endpoint: %s", keyexpr_str.c_str());

    z_keyexpr_t keyexpr = z_keyexpr(keyexpr_str.c_str());
    if (!z_check(keyexpr)) {
        ShowMessage("SU: %s is not a valid key expression", keyexpr_str.c_str());
        return 1;
    }
    const char* expr = keyexpr_str.c_str();

    // zclosure macro does not work with c++17
    z_owned_closure_query_t callback{};
    callback.call = SU_QueryHandler;
    callback.context = const_cast<void*>(static_cast<const void*>(expr));
    callback.drop = nullptr;

    z_owned_queryable_t qable = z_declare_queryable(g_zenoh_context->session, keyexpr, z_move(callback), NULL);
    if (!z_check(qable)) {
        ShowMessage("SU: Unable to create queryable.");
        return 1;
    }

    do
    {
        // heartbeat occassionally ??   
    } while (WaitForSingleObject(g_event_quit, 100) == WAIT_TIMEOUT);

    z_undeclare_queryable(z_move(qable));
    ShowMessage("SU: Closed");

    return 0;
}

// OK
void SU_Initialize(const char* client_id, z_session_t session)
{
    g_zenoh_context = new SU_Context(); // release
    g_zenoh_context->client_id = std::string(client_id);
    g_zenoh_context->session = session;
    g_zenoh_context->valid = true;

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SU_EntryPoint, NULL, 0, NULL);
}

// OK
void SU_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void SU_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);
    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);

    free(g_zenoh_context);
    g_zenoh_context = NULL;
}
