#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

// =====================
// Minimal 3D vector
// =====================
struct Vec3 {
    double x{0}, y{0}, z{0};

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }
    static double dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// =====================
// Quaternion (w, x, y, z)
// Represents rotation from body to world when used as q * v_body * q_conj.
// =====================
struct Quat {
    double w{1}, x{0}, y{0}, z{0};

    Quat() = default;
    Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator+(const Quat& o) const { return {w + o.w, x + o.x, y + o.y, z + o.z}; }
    Quat operator*(double s) const { return {w*s, x*s, y*s, z*s}; }

    static Quat multiply(const Quat& a, const Quat& b) {
        // Hamilton product a*b
        return {
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
            a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
        };
    }

    Quat conj() const { return {w, -x, -y, -z}; }

    void normalize() {
        const double n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 0) { w /= n; x /= n; y /= n; z /= n; }
        else { w = 1; x = y = z = 0; }
    }

    Vec3 rotateBodyToWorld(const Vec3& v_body) const {
        // v_world = q * [0,v] * q_conj
        Quat vq{0, v_body.x, v_body.y, v_body.z};
        Quat out = multiply(multiply(*this, vq), this->conj());
        return {out.x, out.y, out.z};
    }

    Vec3 rotateWorldToBody(const Vec3& v_world) const {
        // v_body = q_conj * [0,v] * q
        Quat vq{0, v_world.x, v_world.y, v_world.z};
        Quat out = multiply(multiply(this->conj(), vq), *this);
        return {out.x, out.y, out.z};
    }
};

// =====================
// State and derivative
// =====================
struct State {
    Vec3 p;   // world position (m)
    Vec3 v;   // world velocity (m/s)
    Quat q;   // body->world attitude
    Vec3 w;   // body angular rate (rad/s), expressed in body frame
};

struct Deriv {
    Vec3 dp;
    Vec3 dv;
    Quat dq;
    Vec3 dw;
};

struct Params {
    double mass{1.5};          // kg
    Vec3 inertia{0.02, 0.02, 0.04}; // diagonal inertia (kg*m^2) [Ix,Iy,Iz]
    double g{9.80665};         // m/s^2 (z-up world: gravity is -z)
    // Optional drag in world frame: F_drag = - (k1*v + k2*|v|*v)
    bool use_drag{true};
    double k1{0.15};
    double k2{0.02};
};

struct Input {
    Vec3 thrust_body;  // N (body frame)
    Vec3 moment_body;  // N*m (body frame)
};

// =====================
// Dynamics
// =====================
static Vec3 invI_times(const Vec3& I, const Vec3& tau) {
    return { tau.x / I.x, tau.y / I.y, tau.z / I.z };
}

static Deriv dynamics(const State& s, const Input& u, const Params& p) {
    Deriv d;

    // Position derivative
    d.dp = s.v;

    // Forces
    // Thrust in world frame
    const Vec3 thrust_world = s.q.rotateBodyToWorld(u.thrust_body);

    // Gravity in world frame (z-up): a_g = (0,0,-g)
    const Vec3 gravity{0.0, 0.0, -p.g};

    // Drag (world frame)
    Vec3 a_drag{0,0,0};
    if (p.use_drag) {
        const double vnorm = s.v.norm();
        const Vec3 Fd = (-p.k1)*s.v + (-p.k2)*vnorm*s.v; // N (assuming coefficients already incorporate mass if you want)
        a_drag = Fd / p.mass;
    }

    // Translational acceleration
    d.dv = (thrust_world / p.mass) + gravity + a_drag;

    // Quaternion derivative: q_dot = 0.5 * q ⊗ omega_quat
    // omega_quat = [0, w_body]
    Quat omega{0.0, s.w.x, s.w.y, s.w.z};
    d.dq = Quat::multiply(s.q, omega) * 0.5;

    // Rotational dynamics (body frame):
    // I*w_dot + w x (I*w) = tau  => w_dot = I^{-1} ( tau - w x (I*w) )
    const Vec3 Iw{ p.inertia.x * s.w.x, p.inertia.y * s.w.y, p.inertia.z * s.w.z };
    const Vec3 wxIw = Vec3::cross(s.w, Iw);
    const Vec3 rhs = u.moment_body - wxIw;
    d.dw = invI_times(p.inertia, rhs);

    return d;
}

// =====================
// RK4 Integrator
// =====================
static State add_scaled(const State& s, const Deriv& k, double h) {
    State out = s;
    out.p += k.dp * h;
    out.v += k.dv * h;
    out.q = out.q + (k.dq * h);
    out.w += k.dw * h;
    return out;
}

static State rk4_step(const State& s, const Input& u, const Params& p, double dt) {
    const Deriv k1 = dynamics(s, u, p);
    const Deriv k2 = dynamics(add_scaled(s, k1, dt*0.5), u, p);
    const Deriv k3 = dynamics(add_scaled(s, k2, dt*0.5), u, p);
    const Deriv k4 = dynamics(add_scaled(s, k3, dt), u, p);

    State out = s;
    out.p += (k1.dp + 2.0*k2.dp + 2.0*k3.dp + k4.dp) * (dt/6.0);
    out.v += (k1.dv + 2.0*k2.dv + 2.0*k3.dv + k4.dv) * (dt/6.0);
    out.q = out.q + (k1.dq + (k2.dq*2.0) + (k3.dq*2.0) + k4.dq) * (dt/6.0);
    out.w += (k1.dw + 2.0*k2.dw + 2.0*k3.dw + k4.dw) * (dt/6.0);

    // Normalize quaternion to prevent drift
    out.q.normalize();
    return out;
}

// =====================
// Example input schedule
// =====================
static Input input_schedule(double t, const Params& p) {
    Input u;
    // Hover-ish thrust: mass*g upward in body z axis (assuming body z-up).
    // If you prefer body z-down (aircraft convention), flip sign accordingly.
    const double hover = p.mass * p.g;

    // Simple demo:
    // 0~2s: hover thrust
    // 2~6s: add small pitch moment to start moving
    // 6~10s: keep thrust, remove moment
    u.thrust_body = {0.0, 0.0, hover};

    if (t > 2.0 && t < 6.0) {
        u.moment_body = {0.0, 0.03, 0.0}; // N*m about body y
    } else {
        u.moment_body = {0.0, 0.0, 0.0};
    }

    // Add slight yaw moment later
    if (t > 6.0 && t < 8.0) {
        u.moment_body.z = 0.02;
    }

    return u;
}

int main(int argc, char** argv) {
    Params params;
    // You can tune these
    params.mass = 1.8;
    params.inertia = {0.03, 0.03, 0.06};
    params.use_drag = true;
    params.k1 = 0.10;
    params.k2 = 0.03;

    // Initial state
    State s;
    s.p = {0,0,0};       // m
    s.v = {0,0,0};       // m/s
    s.q = {1,0,0,0};     // level
    s.w = {0,0,0};       // rad/s

    const double dt = 0.002;  // 500 Hz
    const double T  = 10.0;

    std::ofstream ofs("traj.csv");
    ofs << "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz\n";
    ofs << std::fixed << std::setprecision(6);

    for (int i = 0; i <= static_cast<int>(T/dt); ++i) {
        const double t = i * dt;

        // log
        ofs << t << ","
            << s.p.x << "," << s.p.y << "," << s.p.z << ","
            << s.v.x << "," << s.v.y << "," << s.v.z << ","
            << s.q.w << "," << s.q.x << "," << s.q.y << "," << s.q.z << ","
            << s.w.x << "," << s.w.y << "," << s.w.z << "\n";

        // step
        const Input u = input_schedule(t, params);
        s = rk4_step(s, u, params, dt);
    }

    std::cout << "Wrote traj.csv\n";
    std::cout << "Final position: (" << s.p.x << ", " << s.p.y << ", " << s.p.z << ")\n";
    return 0;
}
