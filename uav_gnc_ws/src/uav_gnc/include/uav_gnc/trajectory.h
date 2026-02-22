#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::Vector3d;

// ==========================================
// 5차 다항식 (Minimum Jerk) 궤적 생성기
// ==========================================
class MinJerkTrajectory {
public:
    MinJerkTrajectory() {}

    void generate(double p0, double v0, double a0, 
                  double pf, double vf, double af, double T) {
        T_ = T;
        c_[0] = p0;
        c_[1] = v0;
        c_[2] = 0.5 * a0;

        Matrix3d A;
        A << pow(T, 3),   pow(T, 4),    pow(T, 5),
             3*pow(T, 2), 4*pow(T, 3),  5*pow(T, 4),
             6*T,         12*pow(T, 2), 20*pow(T, 3);

        Vector3d B;
        B << pf - p0 - v0*T - 0.5*a0*pow(T, 2),
             vf - v0 - a0*T,
             af - a0;

        Vector3d x = A.colPivHouseholderQr().solve(B);
        c_[3] = x(0);
        c_[4] = x(1);
        c_[5] = x(2);
    }

    double getPosition(double t) const {
        if (t < 0.0) t = 0.0;
        if (t > T_) t = T_;
        return c_[0] + c_[1]*t + c_[2]*pow(t, 2) + c_[3]*pow(t, 3) + c_[4]*pow(t, 4) + c_[5]*pow(t, 5);
    }

    double getVelocity(double t) const {
        if (t < 0.0) t = 0.0;
        if (t > T_) t = T_;
        return c_[1] + 2*c_[2]*t + 3*c_[3]*pow(t, 2) + 4*c_[4]*pow(t, 3) + 5*c_[5]*pow(t, 4);
    }

private:
    double c_[6]{0.0};
    double T_{0.0};
};

// ==========================================
// 7차 다항식 (Minimum Snap) 궤적 생성기
// ==========================================
class MinSnapTrajectory {
public:
    MinSnapTrajectory() {}

    void generate(double p0, double v0, double a0, double j0,
                  double pf, double vf, double af, double jf, double T) {
        T_ = T;

        c_[0] = p0;
        c_[1] = v0;
        c_[2] = 0.5 * a0;
        c_[3] = (1.0 / 6.0) * j0;

        Eigen::Matrix4d A;
        A << pow(T, 4),    pow(T, 5),     pow(T, 6),      pow(T, 7),
             4*pow(T, 3),  5*pow(T, 4),   6*pow(T, 5),    7*pow(T, 6),
             12*pow(T, 2), 20*pow(T, 3),  30*pow(T, 4),   42*pow(T, 5),
             24*T,         60*pow(T, 2),  120*pow(T, 3),  210*pow(T, 4);

        Eigen::Vector4d B;
        B << pf - (c_[0] + c_[1]*T + c_[2]*pow(T, 2) + c_[3]*pow(T, 3)),
             vf - (c_[1] + 2*c_[2]*T + 3*c_[3]*pow(T, 2)),
             af - (2*c_[2] + 6*c_[3]*T),
             jf - (6*c_[3]);

        Eigen::Vector4d x = A.colPivHouseholderQr().solve(B);
        c_[4] = x(0);
        c_[5] = x(1);
        c_[6] = x(2);
        c_[7] = x(3);
    }

    double getPosition(double t) const {
        if (t < 0.0) t = 0.0;
        if (t > T_) t = T_;
        return c_[0] + c_[1]*t + c_[2]*pow(t, 2) + c_[3]*pow(t, 3) + 
               c_[4]*pow(t, 4) + c_[5]*pow(t, 5) + c_[6]*pow(t, 6) + c_[7]*pow(t, 7);
    }

    double getVelocity(double t) const {
        if (t < 0.0) t = 0.0;
        if (t > T_) t = T_;
        return c_[1] + 2*c_[2]*t + 3*c_[3]*pow(t, 2) + 4*c_[4]*pow(t, 3) + 
               5*c_[5]*pow(t, 4) + 6*c_[6]*pow(t, 5) + 7*c_[7]*pow(t, 6);
    }

private:
    double c_[8]{0.0};
    double T_{0.0};
};

// ==========================================
// 다중 구간 7차 다항식 (Multi-segment Minimum Snap) 생성기
// ==========================================
class MultiMinSnapTrajectory {
public:
    MultiMinSnapTrajectory() {}

    // 여러 개의 Waypoint와 각 구간별 소요 시간(times)을 받아서 전체 궤적 계수를 한 번에 계산
    void generate(const std::vector<double>& waypoints, const std::vector<double>& times) {
        int N = times.size(); // 구간의 수
        if (N == 0 || waypoints.size() != static_cast<size_t>(N + 1)) return;

        int num_vars = 8 * N; // 각 구간별로 8개의 계수 (7차 다항식)
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_vars, num_vars);
        Eigen::VectorXd B = Eigen::VectorXd::Zero(num_vars);

        int row = 0;

        // 1. 출발점 조건 (t = 0일 때 위치는 wp[0], 속도=가속도=Jerk=0)
        A(row, 0) = 1.0; B(row++) = waypoints[0];
        A(row, 1) = 1.0; B(row++) = 0.0;
        A(row, 2) = 2.0; B(row++) = 0.0;
        A(row, 3) = 6.0; B(row++) = 0.0;

        // 2. 중간 경유지 조건 (위치 지정 및 미분값 연속성 보장)
        for (int i = 0; i < N - 1; ++i) {
            double T = times[i];
            int col_i = i * 8;          // 현재 구간의 계수 시작 열
            int col_next = (i + 1) * 8; // 다음 구간의 계수 시작 열

            // i번째 구간의 끝점 위치 = 다음 Waypoint
            A(row, col_i+0)=1.0; A(row, col_i+1)=T; A(row, col_i+2)=pow(T,2); A(row, col_i+3)=pow(T,3);
            A(row, col_i+4)=pow(T,4); A(row, col_i+5)=pow(T,5); A(row, col_i+6)=pow(T,6); A(row, col_i+7)=pow(T,7);
            B(row++) = waypoints[i+1];

            // i+1번째 구간의 시작점 위치 = 다음 Waypoint
            A(row, col_next+0) = 1.0;
            B(row++) = waypoints[i+1];

            // 연속성 보장: 속도, 가속도, Jerk, Snap, Snap의 1/2차 미분 (총 6개)
            // (1) 속도 연속 (v_i(T) - v_{i+1}(0) = 0)
            A(row, col_i+1)=1.0; A(row, col_i+2)=2*T; A(row, col_i+3)=3*pow(T,2); A(row, col_i+4)=4*pow(T,3);
            A(row, col_i+5)=5*pow(T,4); A(row, col_i+6)=6*pow(T,5); A(row, col_i+7)=7*pow(T,6);
            A(row, col_next+1) = -1.0; B(row++) = 0.0;

            // (2) 가속도 연속 (a_i(T) - a_{i+1}(0) = 0)
            A(row, col_i+2)=2.0; A(row, col_i+3)=6*T; A(row, col_i+4)=12*pow(T,2); A(row, col_i+5)=20*pow(T,3);
            A(row, col_i+6)=30*pow(T,4); A(row, col_i+7)=42*pow(T,5);
            A(row, col_next+2) = -2.0; B(row++) = 0.0;

            // (3) Jerk 연속
            A(row, col_i+3)=6.0; A(row, col_i+4)=24*T; A(row, col_i+5)=60*pow(T,2); A(row, col_i+6)=120*pow(T,3); A(row, col_i+7)=210*pow(T,4);
            A(row, col_next+3) = -6.0; B(row++) = 0.0;

            // (4) Snap 연속
            A(row, col_i+4)=24.0; A(row, col_i+5)=120*T; A(row, col_i+6)=360*pow(T,2); A(row, col_i+7)=840*pow(T,3);
            A(row, col_next+4) = -24.0; B(row++) = 0.0;

            // (5) Snap 1차 미분 연속 (Crackle)
            A(row, col_i+5)=120.0; A(row, col_i+6)=720*T; A(row, col_i+7)=2520*pow(T,2);
            A(row, col_next+5) = -120.0; B(row++) = 0.0;

            // (6) Snap 2차 미분 연속 (Pop)
            A(row, col_i+6)=720.0; A(row, col_i+7)=5040*T;
            A(row, col_next+6) = -720.0; B(row++) = 0.0;
        }

        // 3. 도착점 조건 (t = T_end일 때 위치는 wp[N], 속도=가속도=Jerk=0)
        double T_end = times[N - 1];
        int col_last = (N - 1) * 8;

        A(row, col_last+0)=1.0; A(row, col_last+1)=T_end; A(row, col_last+2)=pow(T_end,2); A(row, col_last+3)=pow(T_end,3);
        A(row, col_last+4)=pow(T_end,4); A(row, col_last+5)=pow(T_end,5); A(row, col_last+6)=pow(T_end,6); A(row, col_last+7)=pow(T_end,7);
        B(row++) = waypoints[N];

        A(row, col_last+1)=1.0; A(row, col_last+2)=2*T_end; A(row, col_last+3)=3*pow(T_end,2); A(row, col_last+4)=4*pow(T_end,3);
        A(row, col_last+5)=5*pow(T_end,4); A(row, col_last+6)=6*pow(T_end,5); A(row, col_last+7)=7*pow(T_end,6);
        B(row++) = 0.0;

        A(row, col_last+2)=2.0; A(row, col_last+3)=6*T_end; A(row, col_last+4)=12*pow(T_end,2); A(row, col_last+5)=20*pow(T_end,3);
        A(row, col_last+6)=30*pow(T_end,4); A(row, col_last+7)=42*pow(T_end,5);
        B(row++) = 0.0;

        A(row, col_last+3)=6.0; A(row, col_last+4)=24*T_end; A(row, col_last+5)=60*pow(T_end,2); A(row, col_last+6)=120*pow(T_end,3); A(row, col_last+7)=210*pow(T_end,4);
        B(row++) = 0.0;

        // 거대한 행렬 풀기 (Eigen의 강력함)
        coeffs_ = A.colPivHouseholderQr().solve(B);
        times_ = times;
    }

    double getPosition(double t) const {
        if (times_.empty()) return 0.0;
        int idx = getSegmentIndex(t);
        double t_local = t - getStartTime(idx);
        int c = idx * 8;
        return coeffs_(c) + coeffs_(c+1)*t_local + coeffs_(c+2)*pow(t_local,2) + coeffs_(c+3)*pow(t_local,3) +
               coeffs_(c+4)*pow(t_local,4) + coeffs_(c+5)*pow(t_local,5) + coeffs_(c+6)*pow(t_local,6) + coeffs_(c+7)*pow(t_local,7);
    }

    double getVelocity(double t) const {
        if (times_.empty()) return 0.0;
        int idx = getSegmentIndex(t);
        double t_local = t - getStartTime(idx);
        int c = idx * 8;
        return coeffs_(c+1) + 2*coeffs_(c+2)*t_local + 3*coeffs_(c+3)*pow(t_local,2) + 4*coeffs_(c+4)*pow(t_local,3) +
               5*coeffs_(c+5)*pow(t_local,4) + 6*coeffs_(c+6)*pow(t_local,5) + 7*coeffs_(c+7)*pow(t_local,6);
    }

    double getTotalTime() const {
        double t = 0;
        for (double dt : times_) t += dt;
        return t;
    }

private:
    int getSegmentIndex(double t) const {
        double accumulated_t = 0.0;
        for (size_t i = 0; i < times_.size(); ++i) {
            accumulated_t += times_[i];
            if (t <= accumulated_t) return i;
        }
        return times_.size() - 1;
    }

    double getStartTime(int idx) const {
        double start_t = 0.0;
        for (int i = 0; i < idx; ++i) start_t += times_[i];
        return start_t;
    }

    Eigen::VectorXd coeffs_;
    std::vector<double> times_;
};