#pragma once
#include <math.h>

#include <limits>

#include "balanceController.hpp"
#include "cmsis_boot/stm32f10x.h"
#include "drv/vesc/vesc.hpp"
#include "global.h"
#include "imu/imu.hpp"
#include "io/genericOut.hpp"
#include "io/pwm_out.hpp"
#include "io/rx.h"
#include "lpf.hpp"
#include "pid.hpp"
#include "stateTracker.hpp"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/stm32f10x_tim.h"

class ConstrainedOut {
 public:
  ConstrainedOut(VescComm* motor_out, Config_BalancingConfig* balance_settings)
      : settings_(balance_settings),
        motor_out_(motor_out),
        motor_out_lpf_(&(balance_settings->output_lpf_rc)) {
    reset();
  }

  void set(float new_out) {
    float prev_val = motor_out_lpf_.getVal();

    new_out = constrain(new_out, prev_val - settings_->max_update_limiter,
                        prev_val + settings_->max_update_limiter);
    new_out = motor_out_lpf_.compute(new_out);

    motor_out_->setCurrent(new_out);
  }

  void setDuty(float value) {
    motor_out_lpf_.reset(0);
    motor_out_->setDuty(value);
  }

  float get() { return motor_out_lpf_.getVal(); }

  void reset() {
    motor_out_lpf_.reset(0);
    motor_out_->setCurrent(0);
  }

 private:
  Config_BalancingConfig* settings_;
  VescComm* motor_out_;
  // This lpf is to smooth out motor output so stepper does not get spikes and
  // does not skip steps.
  BiQuadLpf motor_out_lpf_;
};

class BoardController : public UpdateListener {
 public:
  BoardController(Config* settings, IMU& imu, GenericOut& status_led,
                  GenericOut& beeper, Guard** guards, int guards_count,
                  GenericOut& green_led);

  // Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise
  // controller will freeze.
  void processUpdate(const MpuUpdate& update);

  float batteryVoltage() const { return out[0].mc_values_.v_in; }

 public:
  float fwd;
  float right;

 private:
  Config* settings_;
  IMU& imu_;
  StateTracker state_;
  BalanceController pitch_balancer_;
  BalanceController roll_balancer_;
  PidController yaw_pid_controler_;

  VescComm out[3] = {1, 2, 3};
  ConstrainedOut motor1_;
  ConstrainedOut motor2_;
  ConstrainedOut motor3_;

  GenericOut& status_led_;
  GenericOut& beeper_;

  GenericOut& green_led_;

  int vesc_update_cycle_ctr_ = 0;
};
