#pragma once
#include <cmath>
#include "sixdof.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct ControllerGains {
    // Position -> Velocity
    double kp_pos_xy = 1.0;
    double kp_pos_z  = 1.5;
    // Velocity -> Acceleration
    double kp_vel_xy = 2.0;
    double kp_vel_z  = 2.5;
    // Attitude PD -> Moment
    double kp_att_rp  = 8.0;
    double kd_att_rp  = 1.5;
    double kp_att_yaw = 4.0;
    double kd_att_yaw = 0.8;
    // 최대 기울기 각도 제한 (degree)
    double max_tilt_deg = 25.0;
};

struct Ref {
    // 목표 위치(월드)와 목표 yaw
    Vec3 p_ref;
    double yaw_ref = 0.0;
};

// 현재 상태 s를 보고, 목표 ref를 따라가도록 입력 u를 계산.
Input controller_update(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains);