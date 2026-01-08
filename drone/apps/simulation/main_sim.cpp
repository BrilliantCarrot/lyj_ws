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
    std::ofstream ofs("C:/Users/leeyj/master/drone/csv/traj.csv");
    ofs << "t,"
    << "px,py,pz,"
    << "vx,vy,vz,"
    // << "qw,qx,qy,qz,"
    << "roll,pitch,yaw,"
    << "wx,wy,wz,"
    << "refx,refy,refz,"
    << "roll_ref,pitch_ref,yaw_ref,"
    << "thrust,Mx,My,Mz\n";
    ofs << std::fixed << std::setprecision(6);
    // Controller 설정, 루프 밖(초기화)
    ControllerGains gains; // 게인 설정(기본값)
    Ref ref;
    // ref.p_ref = {0.0, 0.0, 1.0}; // 목표: z=1m hover
    // ref.yaw_ref = 0.0;
    ref.p_ref = {0.0, 0.0, 0.0}; // 시작 목표는 0m
    ref.yaw_ref = 0.0;
    // 반복문을 통한 목표 시나리오 생성
    for (int i = 0; i <= static_cast<int>(T/dt); ++i) {
        const double t = i * dt;

        // 스탭마다 실행
        if (t < 3.0) {
            ref.p_ref.x = 0.0;
            ref.p_ref.y = 0.0;
            ref.p_ref.z = 1.0;
        }
        else {
            ref.p_ref.x = 1.0;
            ref.p_ref.y = 0.0;
            ref.p_ref.z = 1.0;
        }
        ref.yaw_ref = 0.0;

        // const Input u = controller_update(s, ref, params, gains);

        // 3) 현재 Euler 계산 (로그용)
        double roll=0.0, pitch=0.0, yaw=0.0;
        quat_to_euler_zyx(s.q, roll, pitch, yaw);
        // controller (Debug 포함)
        const ControllerOutput out = controller_update_dbg(s, ref, params, gains);
        const Input u = out.u;
        // 로그 (헤더와 동일 열/순서)
        ofs << t << ","
        << s.p.x << "," << s.p.y << "," << s.p.z << ","
        << s.v.x << "," << s.v.y << "," << s.v.z << ","
        << out.dbg.roll << "," << out.dbg.pitch << "," << out.dbg.yaw << ","
        << s.w.x << "," << s.w.y << "," << s.w.z << ","
        << ref.p_ref.x << "," << ref.p_ref.y << "," << ref.p_ref.z << ","
        << out.dbg.roll_ref << "," << out.dbg.pitch_ref << "," << out.dbg.yaw_ref << ","
        << out.u.thrust_body.z << ","
        << out.u.moment_body.x << "," << out.u.moment_body.y << "," << out.u.moment_body.z
        << "\n";

        s = rk4_step(s, u, params, dt);
    }
    std::cout << "traj.csv 작성됨 \n";
    std::cout << "Final position: (" << s.p.x << ", " << s.p.y << ", " << s.p.z << ")\n";
    return 0;
}