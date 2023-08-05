#pragma once

#include "hl2ss_network.h"

#ifdef WINDOWS_UWP

#include "stream_eet.h"
#include "stream_mc.h"
#include "stream_pv.h"
#include "stream_rm.h"
#include "stream_si.h"

#include "ipc_rc.h"
#include "ipc_sm.h"
#include "ipc_su.h"
#include "ipc_vi.h"

#endif // WINDOWS_UWP


bool StartManager(HC_Context_Ptr& context);

bool StopManager(HC_Context_Ptr& context);

#ifdef WINDOWS_UWP

bool StartRM(HC_Context_Ptr& context,
    bool enable_location,
    bool enable_left_front, bool enable_left_left,
    bool enable_right_front, bool enable_right_right,
    H26xFormat vlc_format,
    bool enable_depth_ahat, bool enable_depth_long_throw,
    H26xFormat depth_format,
    bool enable_imu_accel, bool enable_imu_gyro, bool enable_imu_mag);

bool StopRM(HC_Context_Ptr& context);

bool StartPV(HC_Context_Ptr& context,
    bool enable_location,
    H26xFormat pv_format);

bool StopPV(HC_Context_Ptr& context);

bool StartMC(HC_Context_Ptr& context, AACFormat aac_format);

bool StopMC(HC_Context_Ptr& context);

bool StartSI(HC_Context_Ptr& context);

bool StopSI(HC_Context_Ptr& context);

bool StartRC(HC_Context_Ptr& context);

bool StopRC(HC_Context_Ptr& context);

bool StartSM(HC_Context_Ptr& context);

bool StopSM(HC_Context_Ptr& context);

bool StartSU(HC_Context_Ptr& context);

bool StopSU(HC_Context_Ptr& context);

bool StartVI(HC_Context_Ptr& context);

bool StopVI(HC_Context_Ptr& context);

bool StartEET(HC_Context_Ptr& context, uint8_t eye_fps);

bool StopEET(HC_Context_Ptr& context);

#endif // WINDOWS_UWP

