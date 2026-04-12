
#ifndef IMU_H
#define IMU_H

#include "../arduino.h"
#include "../global.h"
#include "../drv/mpu6050/mpu.hpp"
#include "MadgwickAHRS.hpp"
#include "../drv/comms/config.pb.h"

#define MADGWICK

class IMU {
public:
	IMU(const Config* config)
		:mw_(&config->balance_settings.imu_beta), config_(config) {
		mw_.begin(1000);
	}
	void compute(const MpuUpdate& update, bool init = false);

	volatile float angles[2];
	volatile float rates[3];

	MpuUpdate last_update_;

private:
	Madgwick mw_;

	const Config* config_;
	DISALLOW_COPY_AND_ASSIGN(IMU);
};

#endif
