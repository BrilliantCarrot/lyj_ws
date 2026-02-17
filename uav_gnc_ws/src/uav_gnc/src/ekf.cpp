#include "uav_gnc/ekf.h"

EKF::EKF() {
    // 상태 벡터 15차원 초기화
    x_ = VectorXd::Zero(15);
    P_ = MatrixXd::Identity(15, 15);
    q_ = Eigen::Quaterniond(1, 0, 0, 0); // Identity quaternion

    // 초기 공분산 설정 (초기값에 대한 불확실성)
    P_.block<3, 3>(0, 0) *= 1.0;   // Pos
    P_.block<3, 3>(3, 3) *= 0.1;   // Vel
    P_.block<3, 3>(6, 6) *= 0.01;  // Att
    P_.block<3, 3>(9, 9) *= 0.01;  // Acc Bias
    P_.block<3, 3>(12, 12) *= 0.001; // Gyro Bias

    // Process Noise (Q) 튜닝 파라미터
    // 시스템이 얼마나 흔들리는지, 모델이 얼마나 불확실한지
    Q_ = MatrixXd::Identity(15, 15);
    Q_.block<3, 3>(0, 0) *= 0.0;     // Pos (직접 적분 안함)
    Q_.block<3, 3>(3, 3) *= 0.01;    // Vel noise (from accel noise)
    Q_.block<3, 3>(6, 6) *= 0.001;   // Att noise (from gyro noise)
    Q_.block<3, 3>(9, 9) *= 1e-5;    // Accel Bias Random Walk
    Q_.block<3, 3>(12, 12) *= 1e-6;  // Gyro Bias Random Walk

    // Measurement Noise (R)
    R_gps_ = MatrixXd::Identity(3, 3) * 0.25; // GPS 오차 약 0.5m -> 분산 0.25
}

EKF::~EKF() {}

void EKF::init(const Vector3d& p, const Vector3d& v, const Eigen::Quaterniond& q) {
    x_.setZero();
    x_.segment<3>(0) = p;
    x_.segment<3>(3) = v;
    // bias는 0으로 시작한다고 가정 (혹은 초기 캘리브레이션 값)
    q_ = q;
    q_.normalize();
}

void EKF::predict(const Vector3d& acc_meas, const Vector3d& gyro_meas, double dt) {
    // [DEBUG] dt가 튀는지, P 행렬이 터지는지 확인
    // std::cout을 쓰려면 맨 위에 #include <iostream> 확인
    if (dt > 1.0 || dt < 0.0) std::cout << "[Warning] Bad dt: " << dt << std::endl;
    // 1. 현재 상태 가져오기
    Vector3d pos = x_.segment<3>(0);
    Vector3d vel = x_.segment<3>(3);
    Vector3d ba  = x_.segment<3>(9);
    Vector3d bg  = x_.segment<3>(12);

    // 2. 입력 보정 (Measurement - Bias)
    Vector3d acc_unbiased = acc_meas - ba;
    Vector3d gyro_unbiased = gyro_meas - bg;

    // 3. Nominal State 적분 (비선형 예측)
    // Position Update
    pos += vel * dt + 0.5 * (q_ * acc_unbiased - Vector3d(0, 0, g_)) * dt * dt;
    
    // Velocity Update
    // Body 가속도를 World로 회전 후 중력 뺌
    Vector3d acc_world = q_ * acc_unbiased - Vector3d(0, 0, g_);
    vel += acc_world * dt;

    // Attitude Update (Quaternion kinematics)
    // q_dot = 0.5 * q * omega
    Eigen::Quaterniond dq;
    Vector3d omega = gyro_unbiased * dt * 0.5;
    dq.w() = 1.0;
    dq.vec() = omega; // 소각 근사
    q_ = q_ * dq;
    q_.normalize();

    // 4. Jacobian F 계산 (15x15)
    // Error State Kinematics의 선형화 행렬
    MatrixXd F = MatrixXd::Identity(15, 15);
    Matrix3d R = q_.toRotationMatrix();

    // dp/dv = I * dt
    F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;

    // dv/dtheta (skew symmetric of acc)
    // F_v_q = -R * [a_body]x * dt
    Matrix3d acc_skew;
    acc_skew << 0, -acc_unbiased.z(), acc_unbiased.y(),
                acc_unbiased.z(), 0, -acc_unbiased.x(),
                -acc_unbiased.y(), acc_unbiased.x(), 0;
    F.block<3, 3>(3, 6) = -R * acc_skew * dt;

    // dv/dba = -R * dt
    F.block<3, 3>(3, 9) = -R * dt;

    // dq/dq (attitude error dynamics)
    // F_q_q = I - [omega]x * dt
    Matrix3d gyro_skew;
    gyro_skew << 0, -gyro_unbiased.z(), gyro_unbiased.y(),
                 gyro_unbiased.z(), 0, -gyro_unbiased.x(),
                 -gyro_unbiased.y(), gyro_unbiased.x(), 0;
    F.block<3, 3>(6, 6) = Matrix3d::Identity() - gyro_skew * dt; // 단순화된 형태

    // dq/dbg = -I * dt
    F.block<3, 3>(6, 12) = -Matrix3d::Identity() * dt;

    // 5. 공분산 예측 (Time Update): P = F*P*F' + Q
    P_ = F * P_ * F.transpose() + Q_;

    // 6. 상태 벡터 업데이트 (Nominal State에 반영했으므로 x_ 자체는 다시 0 근처로 유지하거나 값 저장)
    // 여기서는 x_에 nominal state 값을 저장해두는 방식 사용
    x_.segment<3>(0) = pos;
    x_.segment<3>(3) = vel;
    // Attitude Error는 항상 0으로 리셋 (Error-State 방식의 특징)
    // 하지만 P 행렬에는 오차가 누적됨.

    // 함수 맨 마지막 줄 (P_ 계산 후)에 추가
    if (std::isnan(x_(0))) {
        std::cout << "[EKF FAIL] State is NaN at dt=" << dt << std::endl;
        std::cout << "P diagonal: " << P_.diagonal().transpose() << std::endl; }
}

void EKF::update_gps(const Vector3d& meas_pos) {
    // 1. 측정 모델 행렬 H (GPS는 위치만 측정하므로 15x15 중 앞 3x3만 Identity)
    // z = Hx + v
    MatrixXd H = MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 0) = Matrix3d::Identity();

    // 2. Kalman Gain K = P * H' * (H * P * H' + R)^-1
    MatrixXd PHt = P_ * H.transpose();
    MatrixXd S = H * PHt + R_gps_; // Innovation Covariance
    MatrixXd K = PHt * S.inverse();

    // 3. Residual (Innovation) y = z - Hx
    Vector3d pred_pos = x_.segment<3>(0);
    Vector3d y = meas_pos - pred_pos;

    // 4. State Update: x = x + K * y
    VectorXd dx = K * y;
    
    // 명목 상태 업데이트 (Injection)
    x_.segment<3>(0) += dx.segment<3>(0); // Pos
    x_.segment<3>(3) += dx.segment<3>(3); // Vel
    x_.segment<3>(9) += dx.segment<3>(9); // Acc Bias
    x_.segment<3>(12) += dx.segment<3>(12); // Gyro Bias

    // Attitude Update (Error Quaternion Injection)
    // dq = [1, 0.5*dtheta]
    Vector3d dtheta = dx.segment<3>(6);
    Eigen::Quaterniond dq;
    dq.w() = 1.0;
    dq.vec() = 0.5 * dtheta;
    q_ = q_ * dq; // 보정 적용
    q_.normalize();

    // 5. Covariance Update: P = (I - K*H) * P
    MatrixXd I = MatrixXd::Identity(15, 15);
    P_ = (I - K * H) * P_;
}