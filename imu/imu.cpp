#include <math.h>
#include "imu.hpp"

// Madgwick takes about 220us to compute
void IMU::compute(const MpuUpdate& update, bool init) {
	last_update_  = update;

	const float MW_GYRO_SCALE = (4 / 65.5);   //MPU6050 and MPU3050   65.5 LSB/(deg/s) and we ignore the last 2 bits
	for (int i = 0; i < 3; i++) {
    rates[i] = update.gyro[i] * MW_GYRO_SCALE;
  }

	if (init) {
		// While gyro is getting initialized its data is invalid - ignore gyro.
		// Take all data from ACC with much higher weight - assuming board is stationary (otherwise gyro would not calibrate)
		mw_.updateIMU(0, 0, 0, update.acc[0] / (float)ACC_1G, update.acc[1] / (float)ACC_1G, update.acc[2] / (float)ACC_1G, true);
	}
	else {
		mw_.updateIMU(rates[0], rates[1], rates[2], update.acc[0] / (float)ACC_1G, update.acc[1] / (float)ACC_1G, update.acc[2] / (float)ACC_1G, false);
	}

	angles[0] = mw_.getRoll();
	angles[1] = mw_.getPitch();
}
