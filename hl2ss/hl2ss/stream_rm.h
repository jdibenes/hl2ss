
#pragma once

#include "hl2ss_network.h"
#include "custom_sink_writers.h"

struct RMStreamConfig {

	H26xFormat vlc_format;
	bool enable_left_front{ false };
	bool enable_left_left{ false };
	bool enable_right_front{ false };
	bool enable_right_right{ false };

	H26xFormat depth_format;
	bool enable_depth_ahat{ false };
	bool enable_depth_long_throw{ false };

	bool enable_imu_accel{ false };
	bool enable_imu_gyro{ false };
	bool enable_imu_mag{ false };

	bool enable_location{ true };


};

void RM_Initialize(HC_Context_Ptr& context, RMStreamConfig config);
void RM_Quit();
void RM_Cleanup();
