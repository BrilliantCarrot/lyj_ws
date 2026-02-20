#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace uav_gnc
{

struct TrajSample
{
  double x{0.0}, y{0.0}, z{0.0};
  double vx{0.0}, vy{0.0}, vz{0.0};
  double ax{0.0}, ay{0.0}, az{0.0};
};

struct Waypoint
{
  double x{0.0}, y{0.0}, z{0.0};
};

class QuinticSegment
{
public:
  QuinticSegment() = default;

  QuinticSegment(double p0, double p1, double T)
  {
    set(p0, p1, T);
  }

  void set(double p0, double p1, double T)
  {
    if (T <= 1e-6) {
      throw std::runtime_error("QuinticSegment: T must be > 0");
    }
    T_ = T;

    // Boundary conditions:
    // p(0)=p0, v(0)=0, a(0)=0
    // p(T)=p1, v(T)=0, a(T)=0
    // Closed-form solution for zero vel/acc endpoints:
    // p(t) = p0 + dp*(10*s^3 - 15*s^4 + 6*s^5), s=t/T
    // We store coefficients in time-domain polynomial:
    // p(t)=a0+a1 t+a2 t^2+a3 t^3+a4 t^4+a5 t^5
    const double dp = (p1 - p0);
    a0_ = p0;
    a1_ = 0.0;
    a2_ = 0.0;
    a3_ = 10.0 * dp / (T_*T_*T_);
    a4_ = -15.0 * dp / (T_*T_*T_*T_);
    a5_ = 6.0 * dp / (T_*T_*T_*T_*T_);
  }

  double T() const { return T_; }

  inline double pos(double t) const
  {
    t = clampTime(t);
    return a0_ + a1_*t + a2_*t*t + a3_*t*t*t + a4_*t*t*t*t + a5_*t*t*t*t*t;
  }

  inline double vel(double t) const
  {
    t = clampTime(t);
    return a1_ + 2.0*a2_*t + 3.0*a3_*t*t + 4.0*a4_*t*t*t + 5.0*a5_*t*t*t*t;
  }

  inline double acc(double t) const
  {
    t = clampTime(t);
    return 2.0*a2_ + 6.0*a3_*t + 12.0*a4_*t*t + 20.0*a5_*t*t*t;
  }

private:
  inline double clampTime(double t) const
  {
    if (t < 0.0) return 0.0;
    if (t > T_) return T_;
    return t;
  }

  double T_{1.0};
  double a0_{0.0}, a1_{0.0}, a2_{0.0}, a3_{0.0}, a4_{0.0}, a5_{0.0};
};

class PiecewiseQuinticTrajectory
{
public:
  void reset()
  {
    wps_.clear();
    segT_.clear();
    x_.clear(); y_.clear(); z_.clear();
    total_T_ = 0.0;
  }

  // 자동 시간 할당: dist/max_speed, 최소 seg_dt_min 보장
  void buildAutoTime(const std::vector<Waypoint>& wps, double max_speed, double seg_dt_min)
  {
    if (wps.size() < 2) {
      throw std::runtime_error("Trajectory: need at least 2 waypoints");
    }
    if (max_speed <= 1e-6) {
      throw std::runtime_error("Trajectory: max_speed must be > 0");
    }
    if (seg_dt_min <= 1e-6) seg_dt_min = 0.1;

    // ===== [FIX] self-reference 버그 방지: segT_를 인자로 넘기지 말고 로컬 벡터 사용 =====
    std::vector<double> seg_times;
    seg_times.reserve(wps.size() - 1);

    for (size_t i = 0; i + 1 < wps.size(); ++i) {
      const double dx = wps[i+1].x - wps[i].x;
      const double dy = wps[i+1].y - wps[i].y;
      const double dz = wps[i+1].z - wps[i].z;
      const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      double T = dist / max_speed;
      T = std::max(T, seg_dt_min);
      seg_times.push_back(T);
    }

    // buildFromSegTimes 내부에서 reset()하고 segT_에 복사해줌
    buildFromSegTimes(wps, seg_times);
  }

  // 수동 시간 할당
  void buildFromSegTimes(const std::vector<Waypoint>& wps, const std::vector<double>& seg_times)
  {
    if (wps.size() < 2) {
      throw std::runtime_error("Trajectory: need at least 2 waypoints");
    }
    if (seg_times.size() != wps.size() - 1) {
      throw std::runtime_error("Trajectory: seg_times size mismatch");
    }

    reset();
    wps_ = wps;
    segT_ = seg_times;

    const size_t N = wps.size() - 1;
    x_.reserve(N); y_.reserve(N); z_.reserve(N);

    total_T_ = 0.0;
    for (size_t i = 0; i < N; ++i) {
      const double T = segT_[i];
      if (T <= 1e-6) {
        throw std::runtime_error("Trajectory: segment time must be > 0");
      }
      x_.emplace_back(wps[i].x, wps[i+1].x, T);
      y_.emplace_back(wps[i].y, wps[i+1].y, T);
      z_.emplace_back(wps[i].z, wps[i+1].z, T);
      total_T_ += T;
    }
  }

  double totalTime() const { return total_T_; }

  TrajSample sample(double t) const
  {
    TrajSample s;
    if (x_.empty()) return s;

    // clamp to [0, total_T_]
    t = std::max(0.0, std::min(t, total_T_));

    // find segment
    size_t idx = 0;
    double t_local = t;
    for (; idx < segT_.size(); ++idx) {
      if (t_local <= segT_[idx]) break;
      t_local -= segT_[idx];
    }
    if (idx >= x_.size()) {
      idx = x_.size() - 1;
      t_local = segT_.back();
    }

    s.x  = x_[idx].pos(t_local);
    s.y  = y_[idx].pos(t_local);
    s.z  = z_[idx].pos(t_local);

    s.vx = x_[idx].vel(t_local);
    s.vy = y_[idx].vel(t_local);
    s.vz = z_[idx].vel(t_local);

    s.ax = x_[idx].acc(t_local);
    s.ay = y_[idx].acc(t_local);
    s.az = z_[idx].acc(t_local);

    return s;
  }

private:
  std::vector<Waypoint> wps_;
  std::vector<double> segT_;
  std::vector<QuinticSegment> x_, y_, z_;
  double total_T_{0.0};
};

} // namespace uav_gnc