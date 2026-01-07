#include "controller.h"

static double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

Input controller_update(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains) {
    Input u;

    // 1) z 위치 오차 -> z 속도 명령 (P)
    const double e_z = ref.p_ref.z - s.p.z;
    const double vz_cmd = gains.kp_pos_z * e_z;

    // 2) z 속도 오차 -> z 가속도 명령 (P)
    const double e_vz = vz_cmd - s.v.z;
    const double az_cmd = gains.kp_vel_z * e_vz;

    // 3) thrust 계산 (현재는 "기울기 보상" 없이 단순히 T = m*(g + az_cmd))
    // 좌표계가 z-up 이면 g는 아래 방향이므로 thrust는 +z로 들어가야 보통 상승합니다.
    const double T = params.mass * (params.g + az_cmd);

    u.thrust_body = {0.0, 0.0, T};
    u.moment_body = {0.0, 0.0, 0.0}; // 아직 자세 제어는 안 함

    return u;
}