
#include <Windows.h>
#include "server.h"
#include "ports.h"
#include "spatial_mapping.h"
#include "log.h"

#define URI_STATIC_BUILD
#include "uriparser/Uri.h"

#include "hl2ss_network.h"

#include "pcpd_msgs/rpc/Hololens2SpatialMapping.h"

#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Foundation.Numerics.h>

using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Foundation::Numerics;


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL;
static HANDLE g_event_quit = NULL;
static HANDLE g_event_client = NULL;

static HC_Context_Ptr  g_zenoh_context{};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------


// OK
struct SM_CreateObserver {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SpatialMapping_CreateObserver();
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct SM_SetVolumes {
    using RequestT = pcpd_msgs::rpc::HL2SMRequest_SetVolumes;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {

        std::vector<SpatialMapping_VolumeDescription> vd;

        auto& volumes = request.volumes();
        auto count = static_cast<uint8_t>(volumes.size());

        vd.resize(count);
        SpatialMapping_VolumeDescription* data = vd.data();
        for (int i = 0; i < count; ++i)
        {
            auto& vol = volumes.at(i);
            switch (vol._d()) {
            case pcpd_msgs::rpc::SM_VT_BOX:
            {
                vd[i].type = SpatialMapping_VolumeType::VolumeType_Box;
                auto& s = vol.bounding_box();
                auto& t = data[i].data.box;
                t.Center.x = static_cast<float>(s.center().x());
                t.Center.y = static_cast<float>(s.center().y());
                t.Center.z = static_cast<float>(s.center().z());
                t.Extents.x = static_cast<float>(s.extents().x());
                t.Extents.y = static_cast<float>(s.extents().y());
                t.Extents.z = static_cast<float>(s.extents().z());
                break;
            }
            case pcpd_msgs::rpc::SM_VT_FRUSTUM:
            {
                vd[i].type = SpatialMapping_VolumeType::VolumeType_Frustum;
                auto& s = vol.frustum();
                auto& t = data[i].data.frustum;

                t.Near.normal.x = static_cast<float>(s.plane_near().normal().x());
                t.Near.normal.y = static_cast<float>(s.plane_near().normal().y());
                t.Near.normal.z = static_cast<float>(s.plane_near().normal().z());
                t.Near.d = s.plane_near().d();

                t.Far.normal.x = static_cast<float>(s.plane_far().normal().x());
                t.Far.normal.y = static_cast<float>(s.plane_far().normal().y());
                t.Far.normal.z = static_cast<float>(s.plane_far().normal().z());
                t.Far.d = s.plane_far().d();

                t.Top.normal.x = static_cast<float>(s.plane_far().normal().x());
                t.Top.normal.y = static_cast<float>(s.plane_far().normal().y());
                t.Top.normal.z = static_cast<float>(s.plane_far().normal().z());
                t.Top.d = s.plane_far().d();

                t.Bottom.normal.x = static_cast<float>(s.plane_far().normal().x());
                t.Bottom.normal.y = static_cast<float>(s.plane_far().normal().y());
                t.Bottom.normal.z = static_cast<float>(s.plane_far().normal().z());
                t.Bottom.d = s.plane_far().d();

                t.Left.normal.x = static_cast<float>(s.plane_far().normal().x());
                t.Left.normal.y = static_cast<float>(s.plane_far().normal().y());
                t.Left.normal.z = static_cast<float>(s.plane_far().normal().z());
                t.Left.d = s.plane_far().d();

                t.Right.normal.x = static_cast<float>(s.plane_far().normal().x());
                t.Right.normal.y = static_cast<float>(s.plane_far().normal().y());
                t.Right.normal.z = static_cast<float>(s.plane_far().normal().z());
                t.Right.d = s.plane_far().d();

                break;
            }
            case pcpd_msgs::rpc::SM_VT_ORIENTED_BOX:
            {
                vd[i].type = SpatialMapping_VolumeType::VolumeType_OrientedBox;
                auto& s = vol.oriented_box();
                auto& t = data[i].data.oriented_box;
                t.Center.x = static_cast<float>(s.center().x());
                t.Center.y = static_cast<float>(s.center().y());
                t.Center.z = static_cast<float>(s.center().z());
                t.Extents.x = static_cast<float>(s.extents().x());
                t.Extents.y = static_cast<float>(s.extents().y());
                t.Extents.z = static_cast<float>(s.extents().z());
                t.Orientation.x = static_cast<float>(s.orientation().x());
                t.Orientation.y = static_cast<float>(s.orientation().y());
                t.Orientation.z = static_cast<float>(s.orientation().z());
                t.Orientation.w = static_cast<float>(s.orientation().w());
                break;
            }
            case pcpd_msgs::rpc::SM_VT_SPHERE:
            {
                vd[i].type = SpatialMapping_VolumeType::VolumeType_OrientedBox;
                auto& s = vol.sphere();
                auto& t = data[i].data.sphere;
                t.Center.x = static_cast<float>(s.center().x());
                t.Center.y = static_cast<float>(s.center().y());
                t.Center.z = static_cast<float>(s.center().z());
                t.Radius = s.radius();
                break;
            }
            default:
                // should not happen...
                break;
            }
        }

        SpatialMapping_SetVolumes(data, count);

        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct SM_GetObservedSurfaces {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::HL2SMResponse_GetObservedSurfaces;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {

        SpatialMapping_SurfaceInfo const* ids;
        size_t count;

        SpatialMapping_GetObservedSurfaces(ids, count);

        std::vector<pcpd_msgs::rpc::HL2SpatialMapping_SurfaceInfo> surfaces(count);
        for (int i = 0; i < count; ++i) {
            auto& s = ids[i];
            auto& t = surfaces.at(i);

            t.id().Data1(s.id.Data1);
            t.id().Data2(s.id.Data2);
            t.id().Data3(s.id.Data3);
            t.id().Data4({ s.id.Data4[0], s.id.Data4[1] , s.id.Data4[2] , s.id.Data4[3] });
            t.update_time(s.update_time);
        }

        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct SM_GetMeshes {
    using RequestT = pcpd_msgs::rpc::HL2SMRequest_GetMeshes;
    using ResponseT = pcpd_msgs::rpc::HL2SMResponse_GetMeshes;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {


        std::vector<SpatialMapping_MeshTask> md;
        uint32_t count = static_cast<uint32_t>(request.description().size());
        uint32_t maxtasks = request.max_tasks();

        SpatialMapping_MeshTask* task;
        SpatialMapping_MeshInfo* info;

        md.resize(count);
        task = md.data();

        for (uint32_t i = 0; i < count; ++i)
        {
            auto& s = request.description().at(i);
            SpatialMapping_MeshDescription d{};
            d.id.Data1 = s.id().Data1();
            d.id.Data2 = s.id().Data2();
            d.id.Data3 = s.id().Data3();
            d.id.Data4[0] = s.id().Data4()[0];
            d.id.Data4[1] = s.id().Data4()[1];
            d.id.Data4[2] = s.id().Data4()[2];
            d.id.Data4[3] = s.id().Data4()[3];
            memcpy(&(task[i]), (const char*)(&d), sizeof(SpatialMapping_MeshDescription));
        }

        SpatialMapping_BeginComputeMeshes(task, count, maxtasks);

        std::vector<pcpd_msgs::rpc::HL2SpatialMapping_MeshInfo> meshes(count);

        for (; count > 0; --count)
        {
            info = SpatialMapping_GetNextMesh();
            if (info == nullptr) {
                continue;
            }
            auto& s = *info;
            auto& t = meshes.at(count);

            t.index(s.index);
            t.status(s.status);
            t.vpl(s.vpl);
            t.til(s.til);
            t.vnl(s.vnl);

            t.scale().x(s.scale.x);
            t.scale().y(s.scale.y);
            t.scale().z(s.scale.z);

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

            t.bsz(s.bsz);

            t.bounds().center().x(static_cast<double>(s.bounds.Center.x));
            t.bounds().center().y(static_cast<double>(s.bounds.Center.y));
            t.bounds().center().z(static_cast<double>(s.bounds.Center.z));
            t.bounds().extents().x(static_cast<double>(s.bounds.Extents.x));
            t.bounds().extents().y(static_cast<double>(s.bounds.Extents.y));
            t.bounds().extents().z(static_cast<double>(s.bounds.Extents.z));
            t.bounds().orientation().x(static_cast<double>(s.bounds.Orientation.x));
            t.bounds().orientation().y(static_cast<double>(s.bounds.Orientation.y));
            t.bounds().orientation().z(static_cast<double>(s.bounds.Orientation.z));
            t.bounds().orientation().w(static_cast<double>(s.bounds.Orientation.w ));

            // copying buffers .. can it be avoided ??
            std::vector<uint8_t> vpd;
            vpd.assign(s.vpd, s.vpd + s.vpl);
            t.vpd(std::move(vpd));

            std::vector<uint8_t> tid;
            tid.assign(s.tid, s.tid + s.til);
            t.tid(std::move(tid));

            std::vector<uint8_t> vnd;
            vnd.assign(s.vnd, s.vnd + s.vnl);
            t.vnd(std::move(vnd));
        }

        SpatialMapping_EndComputeMeshes();

        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};


template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2SMRequest_SetVolumes> {
    bool parse(const std::map<std::string, std::string>& /*args*/, pcpd_msgs::rpc::HL2SMRequest_SetVolumes& /*request*/) {
        SPDLOG_INFO("SM: SetVolumes without payload is not suported.");
        return false;
    }
};

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2SMRequest_GetMeshes> {
    bool parse(const std::map<std::string, std::string>& /*args*/, pcpd_msgs::rpc::HL2SMRequest_GetMeshes& /*request*/) {
        SPDLOG_INFO("SM: GetMeshes without payload is not suported.");
        return false;
    }
};



void SM_QueryHandler(const z_query_t* query, void* context) {
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
                SPDLOG_INFO("Received argument: {0} -> {1}", current->key, current->value);
            }
            current = current->next;
        }
    }
    uriFreeQueryListA(queryList);

    eprosima::fastcdr::FastBuffer result_buffer{};
    std::size_t result_bytes{0};
    bool call_success{ false };

    if (arguments.find("cmd") != arguments.end()) {
        auto cmd = arguments.extract("cmd");

        if (cmd.mapped() == "CreateObserver") {
            call_success = forward_rpc_call(SM_CreateObserver{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetVolumes") {
            call_success = forward_rpc_call(SM_SetVolumes{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "GetObservedSurfaces") {
            call_success = forward_rpc_call(SM_GetObservedSurfaces{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "GetMeshes") {
            call_success = forward_rpc_call(SM_GetMeshes{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else {
            call_success = false;
        }
    }

    if (!call_success) {
        std::string pred_str((const char*)pred.start, pred.len);
        if (payload_value.payload.len > 0) {
            SPDLOG_WARN("[Queryable ] Received unhandled Query '{0}?{1}' with payload length '{2}'",
                z_loan(keystr), pred_str, (int)payload_value.payload.len);
        }
        else {
            SPDLOG_WARN(">> [Queryable ] Received unhandled Query '{0}?{1}'", z_loan(keystr), pred_str);
        }
    }

    z_query_reply_options_t options = z_query_reply_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
    z_query_reply(query, z_keyexpr((const char*)context), (const uint8_t*)result_buffer.getBuffer(), result_bytes, &options);
    z_drop(z_move(keystr));
}



// OK
static DWORD WINAPI SM_EntryPoint(void* param)
{
    if (!g_zenoh_context || !g_zenoh_context->valid) {
        SPDLOG_INFO("SM: Invalid Zenoh Context");
        return 1;
    }

    (void)param;

    SPDLOG_INFO("SM: Waiting for consent");

    SpatialMapping_WaitForConsent();

    std::string keyexpr_str = g_zenoh_context->topic_prefix + "/rpc/sm";
    SPDLOG_INFO("SM: endpoint: {0}", keyexpr_str.c_str());

    z_keyexpr_t keyexpr = z_keyexpr(keyexpr_str.c_str());
    if (!z_check(keyexpr)) {
        SPDLOG_INFO("SM: {0} is not a valid key expression", keyexpr_str.c_str());
        return 1;
    }
    const char* expr = keyexpr_str.c_str();

    // zclosure macro does not work with c++17
    z_owned_closure_query_t callback{};
    callback.call = SM_QueryHandler;
    callback.context = const_cast<void*>(static_cast<const void*>(expr));
    callback.drop = nullptr;

    z_owned_queryable_t qable = z_declare_queryable(z_loan(g_zenoh_context->session), keyexpr, z_move(callback), NULL);
    if (!z_check(qable)) {
        SPDLOG_INFO("SM: Unable to create queryable.");
        return 1;
    }

    do
    {
        // heartbeat occassionally ??   
    } while (WaitForSingleObject(g_event_quit, 100) == WAIT_TIMEOUT && !g_zenoh_context->should_exit);

    z_undeclare_queryable(z_move(qable));
    SPDLOG_INFO("SM: Closed");

    return 0;
}

// OK
void SM_Initialize(HC_Context_Ptr& context)
{
    g_zenoh_context = context;

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SM_EntryPoint, NULL, 0, NULL);
}

// OK
void SM_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void SM_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);

    g_thread = NULL;
    g_event_client = NULL;
    g_event_quit = NULL;

    g_zenoh_context.reset();
}
