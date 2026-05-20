#include "boardController.hpp"

#include <tuple>

namespace {

bool NoCmd(float cmd) { return fabsf(cmd) < 0.01; }

float ApplyRotated(float angle_rad, float right, float fwd, float yaw) {
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
    const float max_abs = fmaxf(fabsf(speed1), fmaxf(fabsf(speed2), fabsf(speed3)));
    if (max_abs > max_output) {
      const float scale = max_output / max_abs;
      speed1 *= scale;
      speed2 *= scale;
      speed3 *= scale;
    }
  }

  return {speed1, speed2, speed3};
}

}  // namespace

BoardController::BoardController(Config* settings, IMU& imu,
                                 GenericOut& status_led, GenericOut& beeper,
                                 Guard** guards, int guards_count,
                                 GenericOut& green_led)
    : settings_(settings),
      imu_(imu),
      state_(guards, guards_count),
      pitch_balancer_(settings_, &(settings_->pitch_angle_pid),
                      &(settings_->pitch_rate_pid)),
      roll_balancer_(settings_, &(settings_->roll_angle_pid),
                     &(settings_->roll_rate_pid)),
      yaw_pid_controler_(&(settings_->yaw_pid)),
      motor1_(&out[0], &settings->balance_settings),
      motor2_(&out[1], &settings->balance_settings),
      motor3_(&out[2], &settings->balance_settings),
      status_led_(status_led),
      beeper_(beeper),
      green_led_(green_led) {}

void BoardController::processUpdate(const MpuUpdate& update) {
  imu_.compute(update);
  State current_state = state_.update();

  switch (current_state) {
    case State::Stopped:
      if (NoCmd(settings_->direct_cmd.fwd) && NoCmd(settings_->direct_cmd.right) &&
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
      float fwdTargetAngle = 0;    // mapRcInput(rxVals[1]) * 5;
      float rightTargetAngle = 0;  // mapRcInput(rxVals[0]) * 5;
      float yaw_target = 0;        //  mapRcInput(rxVals[3]) * 1500;
      float yaw =
          yaw_pid_controler_.compute(yaw_target - update.gyro[2]) * state_.start_progress();

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

      auto [speed1, speed2, speed3] =
          Mix(right, fwd, yaw, settings_->balance_settings.pid_to_current_mult);
      motor1_.set(speed1);
      motor2_.set(speed2);
      motor3_.set(speed3);

      break;
  }
}