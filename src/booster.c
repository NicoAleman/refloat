// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "booster.h"

#include <math.h>

void booster_init(Booster *b) {
    ema_init(&b->forward);
    ema_init(&b->braking);
    booster_reset(b);
}

void booster_reset(Booster *b) {
    ema_reset(&b->forward, 1.0f);
    ema_reset(&b->braking, 0.0f);
}

void booster_configure(Booster *b, float frequency) {
    ema_configure(&b->forward, 1.0f, frequency);
    ema_configure(&b->braking, 1.0f, frequency);
}

void booster_update(
    Booster *b,
    const MotorData *md,
    const RefloatConfig *config,
    float proportional,
    float *kp_pitch
) {
    ema_update(&b->braking, md->braking ? 1.0f : 0.0f);
    float brake = b->braking.value;

    // Riding Direction, or Torque Direction when using positive current
    // toward opposite direction (e.g. transition from braking to accelerating
    // backwards).
    bool forward_target = md->forward;
    if (!md->braking && md->erpm * md->dir_current < 0.0f) {
        forward_target = md->dir_current > 0.0f;
    }
    ema_update(&b->forward, forward_target ? 1.0f : 0.0f);
    float forward = b->forward.value;

    // Weight controller_down vs battery_down by whether that side is the
    // directional-nose (accel) or directional-tail (brake).
    float controller_down = fmaxf(proportional, 0.0f);
    float battery_down = fmaxf(-proportional, 0.0f);
    float controller_match = 1.0f - fabsf(forward - (1.0f - brake));
    float battery_match = 1.0f - fabsf(forward - brake);
    float error = controller_down * controller_match + battery_down * battery_match;

    float angle =
        config->booster_angle + (config->brkbooster_angle - config->booster_angle) * brake;
    float ramp = config->booster_ramp + (config->brkbooster_ramp - config->booster_ramp) * brake;

    // 0 disables that side of Booster; keep the current Pitch KP.
    float accel_target = config->booster_mahony_kp > 0.0f ? config->booster_mahony_kp : *kp_pitch;
    float brake_target =
        config->brkbooster_mahony_kp > 0.0f ? config->brkbooster_mahony_kp : *kp_pitch;
    float target_kp = accel_target + (brake_target - accel_target) * brake;

    // Only tighten (lower KP), higher than current Pitch KP is ignored.
    if (target_kp <= 0.0f || *kp_pitch <= target_kp) {
        return;
    }

    if (error > angle) {
        if (error - angle < ramp) {
            float ratio = (error - angle) / ramp;
            *kp_pitch = *kp_pitch + (target_kp - *kp_pitch) * ratio;
        } else {
            *kp_pitch = target_kp;
        }
    }
}
