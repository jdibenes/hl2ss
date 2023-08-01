#pragma once

#include "hl2ss_network.h"

#include "stream_eet.h"
#include "stream_mc.h"
#include "stream_pv.h"
#include "stream_rm.h"
#include "stream_si.h"

#include "ipc_rc.h"
#include "ipc_sm.h"
#include "ipc_su.h"
#include "ipc_vi.h"

#define HL2SS_ENABLE_RM    1
#define HL2SS_ENABLE_PV    2
#define HL2SS_ENABLE_MC    4
#define HL2SS_ENABLE_SI    8
#define HL2SS_ENABLE_RC   16
#define HL2SS_ENABLE_SM   32
#define HL2SS_ENABLE_SU   64
#define HL2SS_ENABLE_VI  128
#define HL2SS_ENABLE_MQ  256
#define HL2SS_ENABLE_EET 512

struct HC_Context {
    std::string client_id;
    z_owned_session_t session;
    uint32_t streams_enabled;
    uint32_t streams_started;
    bool valid{ false };
};

bool StartManager(HC_Context* context);

bool StopManager(HC_Context* context);

bool StartRM(HC_Context* context,
    bool enable_location,
    bool enable_left_front, bool enable_left_left,
    bool enable_right_front, bool enable_right_right,
    H26xFormat vlc_format,
    bool enable_depth_ahat, bool enable_depth_long_throw,
    H26xFormat depth_format,
    bool enable_imu_accel, bool enable_imu_gyro, bool enable_imu_mag);

bool StopRM(HC_Context* context);

bool StartPV(HC_Context* context,
    bool enable_location,
    H26xFormat pv_format);

bool StopPV(HC_Context* context);

bool StartMC(HC_Context* context, AACFormat aac_format);

bool StopMC(HC_Context* context);

bool StartSI(HC_Context* context);

bool StopSI(HC_Context* context);

bool StartRC(HC_Context* context);

bool StopRC(HC_Context* context);

bool StartSM(HC_Context* context);

bool StopSM(HC_Context* context);

bool StartSU(HC_Context* context);

bool StopSU(HC_Context* context);

bool StartVI(HC_Context* context);

bool StopVI(HC_Context* context);

bool StartEET(HC_Context* context, uint8_t eye_fps);

bool StopEET(HC_Context* context);
