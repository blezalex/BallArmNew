#pragma once
#include <math.h>

#include <limits>
#include <tuple>

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

  void setDuty(float value) { motor_out_->setDuty(value); }

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
        pitch_balancer_(settings_, &(settings_->pitch_angle_pid), &(settings_->pitch_rate_pid)),
        roll_balancer_(settings_, &(settings_->roll_angle_pid), &(settings_->roll_rate_pid)),
        yaw_pid_controler_(&(settings_->yaw_pid)),
        motor1_(&out[0], &settings->balance_settings),
        motor2_(&out[1], &settings->balance_settings),
        motor3_(&out[2], &settings->balance_settings),
        status_led_(status_led),
        beeper_(beeper),
        green_led_(green_led) {}

  float mapRcInput(uint16_t input) {
    if (input < MIN_MOTOR_CMD || input > MAX_MOTOR_CMD) {
      return 0;
    }

    return fmap(input, MIN_MOTOR_CMD, MAX_MOTOR_CMD, -1, 1);
  }

  bool NoCmd(float cmd) { return fabsf(cmd) < 0.01; }

  // Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise
  // controller will freeze.
  void processUpdate(const MpuUpdate& update) {
    imu_.compute(update);
    State current_state = state_.update();

    switch (current_state) {
      case State::Stopped:
        if (NoCmd(settings_->direct_cmd.fwd) &&
            NoCmd(settings_->direct_cmd.right) &&
            NoCmd(settings_->direct_cmd.yaw)) {
          motor1_.reset();
          motor2_.reset();
          motor3_.reset();
        } else {
          auto [speed1, speed2, speed3] =
              Mix(settings_->direct_cmd.right, settings_->direct_cmd.fwd,
                  settings_->direct_cmd.yaw, 1.0f);

          motor1_.setDuty(speed1);
          motor2_.setDuty(speed2);
          motor3_.setDuty(speed3);
        }

        status_led_.setState(0);
        beeper_.setState(0);
        break;

      case State::FirstIteration:
        motor1_.reset();
        motor2_.reset();
        motor3_.reset();

        pitch_balancer_.reset();
        roll_balancer_.reset();
        yaw_pid_controler_.reset();

        status_led_.setState(1);
        // intentional fall through
      case State::Starting:
      case State::Running:

        float fwdTargetAngle = 0; // mapRcInput(rxVals[1]) * 5;
        float rightTargetAngle = 0; // mapRcInput(rxVals[0]) * 5;
        float yaw_target = 0; //  mapRcInput(rxVals[3]) * 1500;
        float yaw = yaw_pid_controler_.compute(yaw_target - update.gyro[2]) *
                    state_.start_progress();

        const float fwd_error = fwdTargetAngle - imu_.angles[1];
        const float right_error = rightTargetAngle - imu_.angles[0];
        if (current_state == State::Starting) {
          fwd = pitch_balancer_.computeStarting(fwd_error, update.gyro[1],
                                                state_.start_progress());
          right = roll_balancer_.computeStarting(right_error, update.gyro[0],
                                                 state_.start_progress());
        } else {
          fwd = pitch_balancer_.compute(fwd_error, update.gyro[1]);
          right = roll_balancer_.compute(right_error, update.gyro[0]);
        }

        fwd *= settings_->balance_settings.pid_to_current_mult;
        right *= settings_->balance_settings.pid_to_current_mult;

        auto [speed1, speed2, speed3] = Mix(
            right, fwd, yaw, settings_->balance_settings.pid_to_current_mult);
        motor1_.set(speed1);
        motor2_.set(speed2);
        motor3_.set(speed3);

        break;
    }
  }

  constexpr float ApplyRotated(float angle_rad, float right, float fwd,
                               float yaw) {
    return yaw + cos(angle_rad) * fwd - sin(angle_rad) * right;
  }

  std::tuple<float, float, float> Mix(float right, float fwd, float yaw,
                                      float max_output) {
    constexpr float m1_angle = deg_to_rad(90);
    constexpr float m2_angle = m1_angle + deg_to_rad(120);
    constexpr float m3_angle = m2_angle + deg_to_rad(120);

    float speed1 = ApplyRotated(m1_angle, right, fwd, yaw);
    float speed2 = ApplyRotated(m2_angle, right, fwd, yaw);
    float speed3 = ApplyRotated(m3_angle, right, fwd, yaw);

    if (max_output > 0.0f) {
      const float max_abs =
          fmaxf(fabsf(speed1), fmaxf(fabsf(speed2), fabsf(speed3)));
      if (max_abs > max_output) {
        const float scale = max_output / max_abs;
        speed1 *= scale;
        speed2 *= scale;
        speed3 *= scale;
      }
    }

    return {speed1, speed2, speed3};
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

  int vesc_update_cycle_ctr_ = 0;
};
