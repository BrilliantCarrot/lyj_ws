// controller.h
#pragma once
#include <cmath>
#include "sixdof_types.h"

// sixdof.cpp에서 쓰는 Vec3, Quat, State, Input, Params를 그대로 include 하거나
// 공용 헤더로 빼는 것이 가장 좋습니다.
// 우선은 "동일한 정의를 공유한다"는 가정으로 진행합니다.

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

    double max_tilt_deg = 25.0;
};

struct Ref {
    // 목표 위치(월드) + 목표 yaw
    Vec3 p_ref;
    double yaw_ref = 0.0;
};

// 핵심: 현재 상태 s를 보고, 목표 ref를 따라가도록 입력 u를 계산한다.
Input controller_update(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains);