#pragma once
#include <math.h>

#include <limits>

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


#define BRAKE_VIA_USART

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
                  GenericOut& green_led)
      : settings_(settings),
        imu_(imu),
        state_(guards, guards_count),
        pitch_balancer_(settings_, &(settings_->angle_pid)),
        roll_balancer_(settings_, &(settings_->angle_pid)),
        yaw_pid_controler_(&(settings_->yaw_pid)),
        motor1_(&out[0], &settings->balance_settings),
        motor2_(&out[1], &settings->balance_settings),
        motor3_(&out[2], &settings->balance_settings),
        status_led_(status_led),
        beeper_(beeper),
        green_led_(green_led),
        fwd_lpf_(&settings->misc.throttle_rc),
        right_lpf_(&settings->misc.throttle_rc) {}

  float mapRcInput(uint16_t input) {
    if (input < MIN_MOTOR_CMD || input > MAX_MOTOR_CMD) {
      return 0;
    }

    return fmap(input, MIN_MOTOR_CMD, MAX_MOTOR_CMD, -1, 1);
  }

  // Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise
  // controller will freeze.
  void processUpdate(const MpuUpdate& update) {
    imu_.compute(update);
    State current_state = state_.update();

    switch (current_state) {
      case State::Stopped:
        motor1_.reset();
        motor2_.reset();
        motor3_.reset();

        status_led_.setState(0);
        beeper_.setState(0);
        break;

      case State::FirstIteration:
        motor1_.reset();
        motor2_.reset();
        motor3_.reset();

        fwd_lpf_.reset();
        right_lpf_.reset();

        pitch_balancer_.reset();
        roll_balancer_.reset();
        yaw_pid_controler_.reset();

        status_led_.setState(1);
        // intentional fall through
      case State::Starting:
      case State::Running:

        float fwdTargetAngle = mapRcInput(rxVals[1]) * 5;
        float rightTargetAngle = mapRcInput(rxVals[0]) * 5;

        // float yaw_target = mapRcInput(rxVals[3]) * 1500;
        float yaw_target = 0;
        float yaw = yaw_pid_controler_.compute(update.gyro[2] - yaw_target) *
                    state_.start_progress();

        if (current_state == State::Starting) {
          fwd = pitch_balancer_.computeStarting(imu_.angles[1] - fwdTargetAngle,
                                                update.gyro[1],
                                                state_.start_progress());
          right = roll_balancer_.computeStarting(
              imu_.angles[0] - rightTargetAngle, -update.gyro[0],
              state_.start_progress());
        } else {
          fwd = pitch_balancer_.compute(imu_.angles[1] - fwdTargetAngle,
                                        update.gyro[1]);
          right = roll_balancer_.compute(imu_.angles[0] - rightTargetAngle,
                                         -update.gyro[0]);
        }

        fwd *= settings_->balance_settings.usart_control_scaling;
        right *= settings_->balance_settings.usart_control_scaling;

#ifdef SPEED_CTRL
        if (current_state != State::Starting) {
          fwd += fwd_lpf_.getVal();
          right += right_lpf_.getVal();

          fwd_lpf_.compute(fwd);
          right_lpf_.compute(right);
        }
#endif

        right = -right;
        float speed1 = yaw + right;
        float speed2 =
            yaw + cos(deg_to_rad(120)) * right - sin(deg_to_rad(120)) * fwd;
        float speed3 =
            yaw + cos(deg_to_rad(120)) * right + sin(deg_to_rad(120)) * fwd;

        motor1_.set(speed1);
        motor2_.set(speed2);
        motor3_.set(speed3);

        break;
    }
  }

 public:
  float fwd;
  float right;

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

  // These lpfs compensate for body inertia.
  BiQuadLpf fwd_lpf_;
  BiQuadLpf right_lpf_;

  int vesc_update_cycle_ctr_ = 0;
};
