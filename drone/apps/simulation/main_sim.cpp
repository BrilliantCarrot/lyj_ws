#include <fstream>
#include <iostream>
#include <direct.h>
#include <iomanip>
#include "sixdof.h"
#include "controller.h"

int main(int argc, char** argv) {
    char cwd[_MAX_PATH];
    _getcwd(cwd, sizeof(cwd));
    std::cout << "CWD = " << cwd << std::endl;
    Params params;
    params.mass = 2;
    params.inertia = {0.03, 0.03, 0.06};
    params.use_drag = true;
    params.k1 = 0.10;
    params.k2 = 0.03;
    State s;
    s.p = {0,0,0};       // m
    s.v = {0,0,0};       // m/s
    s.q = {1,0,0,0};     // level
    s.w = {0,0,0};       // rad/s
    const double dt = 0.002;  // 500 Hz
    const double T  = 10.0;
    std::ofstream ofs("C:/Users/leeyj/master/drone/px4/traj.csv");
    // ofs << "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz\n";
    ofs << "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,refx,refy,refz\n";
    ofs << std::fixed << std::setprecision(6);
    // Controller 설정, 루프 밖(초기화)
    ControllerGains gains; // 게인 설정(기본값)
    Ref ref;
    // ref.p_ref = {0.0, 0.0, 1.0}; // 목표: z=1m hover
    // ref.yaw_ref = 0.0;
    ref.p_ref = {0.0, 0.0, 0.0}; // 시작 목표는 0m
    ref.yaw_ref = 0.0;
    // 루프 안(매 스텝마다)
    for (int i = 0; i <= static_cast<int>(T/dt); ++i) {
        const double t = i * dt;

        // 로그용
        ofs << t << ","
            << s.p.x << "," << s.p.y << "," << s.p.z << ","
            << s.v.x << "," << s.v.y << "," << s.v.z << ","
            << s.q.w << "," << s.q.x << "," << s.q.y << "," << s.q.z << ","
            << s.w.x << "," << s.w.y << "," << s.w.z << ","
            << ref.p_ref.x << "," << ref.p_ref.y << "," << ref.p_ref.z << "\n";

        // 스탭마다 실행
        // const Input u = input_schedule(t, params);
        // s = rk4_step(s, u, params, dt);
        // 2초 정지, 그 후 1m 상승 목표(x,y는 유지)
        if (t < 2.0) ref.p_ref.z = 0.0;
        else         ref.p_ref.z = 1.0;
        ref.p_ref.x = 0.0;
        ref.p_ref.y = 0.0;
        const Input u = controller_update(s, ref, params, gains);
        s = rk4_step(s, u, params, dt);
    }

    std::cout << "traj.csv 작성됨 \n";
    std::cout << "Final position: (" << s.p.x << ", " << s.p.y << ", " << s.p.z << ")\n";
    return 0;
}

// =====================
// 입력 생성기(임시)
// 시간 t에 따라 기체에 가해줄 추력, 모멘트 산출 입력 생성기, 제어기가 없는 상태에서 테스트용 조종 입력 시나리오
// =====================
// static Input input_schedule(double t, const Params& p) {
//     Input u;
//     // Hover-ish thrust: mass*g upward in body z axis (assuming body z-up).
//     // If you prefer body z-down (aircraft convention), flip sign accordingly.
//     const double hover = p.mass * p.g;
//     // 데모 시나리오:
//     // 시작~2s: 호버링, 자세 유지
//     // 2~6s: 피치 토크 발생 -> 각속도 w 생성 -> 자세 q 변화 -> 전진 가속
//     // 6~종료: 모멘트를 0으로, 관성에 의해 계속 전진
//     u.thrust_body = {0.0, 0.0, hover};

//     if (t > 2.0 && t < 6.0) {
//         u.moment_body = {0.0, 0.03, 0.0}; // N*m about body y
//     } else {
//         u.moment_body = {0.0, 0.0, 0.0};
//     }
//     if (t > 6.0 && t < 8.0) {
//         // u.moment_body.z = 0.02;
//         u.moment_body = {0.0, -0.04, 0.0}; // 다시 호버링
//     }

//     return u;
// }