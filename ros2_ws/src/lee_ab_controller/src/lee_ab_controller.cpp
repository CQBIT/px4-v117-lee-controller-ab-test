#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleRatesSetpoint;
using px4_msgs::msg::VehicleThrustSetpoint;
using px4_msgs::msg::VehicleTorqueSetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleControlMode;

namespace {
constexpr double kPi = 3.14159265358979323846;

double clampd(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

Eigen::Matrix3d hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d H;
  H << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
      -v.y(), v.x(), 0.0;
  return H;
}

Eigen::Vector3d vee(const Eigen::Matrix3d &M) {
  return Eigen::Vector3d(M(2,1), M(0,2), M(1,0));
}

Eigen::Vector3d so3_log(const Eigen::Matrix3d &R) {
  Eigen::AngleAxisd aa(R);
  if (!std::isfinite(aa.angle()) || aa.angle() < 1e-9) return Eigen::Vector3d::Zero();
  return aa.angle() * aa.axis();
}

double so3_angle(const Eigen::Matrix3d &R) {
  return std::acos(clampd((R.trace() - 1.0) * 0.5, -1.0, 1.0));
}

double smoothstep01(double x) {
  x = clampd(x, 0.0, 1.0);
  return x*x*(3.0 - 2.0*x);
}

struct Reference {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};
} // namespace

class LeeABController final : public rclcpp::Node {
public:
  LeeABController() : Node("lee_ab_controller") {
    mode_ = declare_parameter<std::string>("mode", "rate");
    scenario_ = declare_parameter<std::string>("scenario", "figure8");
    output_csv_ = declare_parameter<std::string>("output_csv", "controller.csv");
    control_hz_ = declare_parameter<double>("control_hz", 100.0);
    experiment_duration_ = declare_parameter<double>("duration", 25.0);

    mass_ = declare_parameter<double>("mass", 2.0643076923076924);
    inertia_ << 0.023615184955393723, 0, 0,
                0, 0.023718109765741166, 0,
                0, 0, 0.04399995371400395;

    kp_pos_ = vec_param("kp_pos", {4.0, 4.0, 6.0});
    kd_vel_ = vec_param("kd_vel", {3.2, 3.2, 4.2});
    k_att_rate_ = vec_param("k_att_rate", {7.0, 7.0, 3.5});

    const Eigen::Vector3d wn = vec_param("lee_wn", {10.0, 10.0, 4.5});
    const Eigen::Vector3d zeta = vec_param("lee_zeta", {0.9, 0.9, 0.9});
    kR_ = inertia_.diagonal().cwiseProduct(wn.cwiseProduct(wn));
    kOmega_ = 2.0 * inertia_.diagonal().cwiseProduct(zeta.cwiseProduct(wn));

    torque_scale_nm_ = vec_param("torque_scale_nm", {3.0, 3.0, 0.35});
    max_torque_norm_ = declare_parameter<double>("max_torque_norm", 0.55);
    max_thrust_norm_ = declare_parameter<double>("max_thrust_norm", 0.90);

    constexpr double motor_constant = 8.54858e-6;
    constexpr double omega_max = 1000.0;
    max_total_thrust_n_ = 4.0 * motor_constant * omega_max * omega_max;

    if (mode_ != "rate" && mode_ != "torque") {
      throw std::runtime_error("mode must be 'rate' or 'torque'");
    }

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", pub_qos);
    rates_pub_ = create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", pub_qos);
    thrust_pub_ = create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", pub_qos);
    torque_pub_ = create_publisher<VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", pub_qos);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", pub_qos);

    odom_sub_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](const VehicleOdometry::SharedPtr msg) { on_odom(*msg); });
    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
      [this](const VehicleStatus::SharedPtr msg) { on_status(*msg); });
    control_mode_sub_ = create_subscription<VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", rclcpp::SensorDataQoS(),
      [this](const VehicleControlMode::SharedPtr msg) { on_control_mode(*msg); });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
      [this](const VehicleCommandAck::SharedPtr msg) { on_ack(*msg); });

    csv_.open(output_csv_);
    if (!csv_) throw std::runtime_error("Cannot open output CSV: " + output_csv_);
    csv_ << "t,flight_t,mode,scenario,x,y,z,xd,yd,zd,vx,vy,vz,vxd,vyd,vzd,"
            "pos_err,vel_err,att_err_deg,wx,wy,wz,wspx,wspy,wspz,"
            "thrust_norm,taux_norm,tauy_norm,tauz_norm,armed,nav_state,failsafe,preflight_ok\n";
    csv_ << std::setprecision(10);

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "mode=%s scenario=%s hz=%.1f output=%s",
                mode_.c_str(), scenario_.c_str(), control_hz_, output_csv_.c_str());
  }

private:
  Eigen::Vector3d vec_param(const std::string &name, const std::array<double,3> &def) {
    auto v = declare_parameter<std::vector<double>>(name, {def[0], def[1], def[2]});
    if (v.size() != 3) throw std::runtime_error(name + " must have exactly 3 values");
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  uint64_t timestamp_us() {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
  }

  void on_odom(const VehicleOdometry &m) {
    if (!std::isfinite(m.position[0]) || !std::isfinite(m.q[0])) return;
    p_ = Eigen::Vector3d(m.position[0], m.position[1], m.position[2]);
    v_ = Eigen::Vector3d(m.velocity[0], m.velocity[1], m.velocity[2]);
    omega_ = Eigen::Vector3d(m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]);
    Eigen::Quaterniond q(m.q[0], m.q[1], m.q[2], m.q[3]);
    q.normalize();
    R_ = q.toRotationMatrix();
    if (!yaw_initialized_) {
      yaw0_ = std::atan2(R_(1,0), R_(0,0));
      yaw_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Captured initial NED yaw %.3f rad (%.1f deg)", yaw0_, yaw0_*180.0/kPi);
    }
    odom_ready_ = true;
  }

  void on_status(const VehicleStatus &m) {
    status_ready_ = true;
    nav_state_ = m.nav_state;
    failsafe_ = m.failsafe;
    preflight_ok_ = m.pre_flight_checks_pass;
    if (!control_mode_ready_) {
      armed_ = (m.arming_state == VehicleStatus::ARMING_STATE_ARMED);
      offboard_active_ = (m.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    }
  }

  void on_control_mode(const VehicleControlMode &m) {
    const bool first = !control_mode_ready_;
    control_mode_ready_ = true;
    armed_ = m.flag_armed;
    offboard_active_ = m.flag_control_offboard_enabled;
    if (!status_ready_) nav_state_ = m.source_id;
    if (first) {
      RCLCPP_INFO(get_logger(), "VehicleControlMode received: armed=%d offboard=%d source_id=%u",
                  armed_ ? 1 : 0, offboard_active_ ? 1 : 0,
                  static_cast<unsigned>(m.source_id));
    }
  }

  void on_ack(const VehicleCommandAck &m) {
    if (m.command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
        m.command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
      RCLCPP_INFO(get_logger(), "VehicleCommandAck command=%u result=%u result_param1=%u result_param2=%d",
                  static_cast<unsigned>(m.command), static_cast<unsigned>(m.result),
                  static_cast<unsigned>(m.result_param1), static_cast<int>(m.result_param2));
    }
  }

  Reference reference(double flight_t) const {
    Reference r;
    r.yaw = yaw0_;
    const double z_ramp = smoothstep01(flight_t / 3.0);
    r.p.z() = -2.0 * z_ramp;
    r.v.z() = (flight_t < 3.0) ? -2.0 * (6.0*(flight_t/3.0)*(1.0-flight_t/3.0)) / 3.0 : 0.0;

    if (flight_t < 5.0 || scenario_ == "hover") return r;

    const double tt = flight_t - 5.0;
    const double T_ramp = 2.0;
    const double u = clampd(tt / T_ramp, 0.0, 1.0);
    const double amp = smoothstep01(u);
    const double amp_dot = (tt > 0.0 && tt < T_ramp) ? (6.0*u*(1.0-u)/T_ramp) : 0.0;
    const double amp_ddot = (tt > 0.0 && tt < T_ramp) ? (6.0*(1.0-2.0*u)/(T_ramp*T_ramp)) : 0.0;
    double w = 0.45;
    if (scenario_ == "aggressive") w = 0.90;
    if (scenario_ == "windy_figure8") w = 0.55;

    auto ramp_axis = [&](double q, double qd, double qdd, int axis) {
      r.p(axis) = amp*q;
      r.v(axis) = amp_dot*q + amp*qd;
      r.a(axis) = amp_ddot*q + 2.0*amp_dot*qd + amp*qdd;
    };

    if (scenario_ == "circle") {
      const double radius = 2.0;
      const double qx = radius*std::sin(w*tt);
      const double qy = radius*(1.0-std::cos(w*tt));
      ramp_axis(qx, radius*w*std::cos(w*tt), -radius*w*w*std::sin(w*tt), 0);
      ramp_axis(qy, radius*w*std::sin(w*tt), radius*w*w*std::cos(w*tt), 1);
    } else {
      const double A = 2.5, B = 1.5;
      const double qx = A*std::sin(w*tt);
      const double qy = B*std::sin(2.0*w*tt);
      ramp_axis(qx, A*w*std::cos(w*tt), -A*w*w*std::sin(w*tt), 0);
      ramp_axis(qy, 2.0*B*w*std::cos(2.0*w*tt), -4.0*B*w*w*std::sin(2.0*w*tt), 1);
    }
    return r;
  }

  Eigen::Matrix3d desired_attitude(const Eigen::Vector3d &b3, double yaw) const {
    Eigen::Vector3d b1c(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d b2 = b3.cross(b1c);
    if (b2.norm() < 1e-6) {
      b1c = Eigen::Vector3d(0.0, 1.0, 0.0);
      b2 = b3.cross(b1c);
    }
    b2.normalize();
    const Eigen::Vector3d b1 = b2.cross(b3).normalized();
    Eigen::Matrix3d Rd;
    Rd.col(0)=b1; Rd.col(1)=b2; Rd.col(2)=b3;
    return Rd;
  }

  void publish_offboard() {
    OffboardControlMode m{};
    m.timestamp = timestamp_us();
    m.position = false;
    m.velocity = false;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = (mode_ == "rate");
    m.thrust_and_torque = (mode_ == "torque");
    m.direct_actuator = false;
    offboard_pub_->publish(m);
  }

  void command(uint16_t cmd, float p1=0.f, float p2=0.f) {
    VehicleCommand m{};
    m.timestamp = timestamp_us();
    m.param1 = p1;
    m.param2 = p2;
    m.command = cmd;
    m.target_system = 1;
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 1;
    m.from_external = true;
    command_pub_->publish(m);
  }

  [[noreturn]] void fatal_exit(int code, const std::string &why) {
    csv_.flush();
    RCLCPP_ERROR(get_logger(), "%s", why.c_str());
    std::exit(code);
  }

  void tick() {
    publish_offboard();
    if (!odom_ready_) return;

    const double now = this->now().seconds();
    if (!started_) {
      start_time_ = now;
      started_ = true;
      prev_time_ = now;
      prev_Rd_ = R_;
    }
    const double t = now - start_time_;
    const double dt = clampd(now - prev_time_, 0.001, 0.05);
    prev_time_ = now;

    // PX4's official ROS 2 offboard example sends the mode-change and arm
    // commands in the same cycle after first streaming OffboardControlMode.
    // Do not gate the arm command on VehicleStatus: in headless CI that topic
    // can fail to reach a ROS subscriber even while command acks and odometry do.
    // VehicleControlMode is exported by PX4 v1.17 at 50 Hz and directly exposes
    // the armed/offboard control flags, so use it as the primary confirmation.
    if (t > 2.0 && !offboard_active_ && now - last_mode_command_time_ > 0.5) {
      command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
      last_mode_command_time_ = now;
    }
    if (t > 2.0 && !armed_ && now - last_arm_command_time_ > 0.5) {
      command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 21196.f);
      last_arm_command_time_ = now;
    }

    if (t > 18.0 && (!control_mode_ready_ || !offboard_active_ || !armed_)) {
      fatal_exit(30, "PX4 failed to confirm armed Offboard state via VehicleControlMode within 18 s");
    }

    if (armed_ && offboard_active_ && !flight_started_) {
      flight_started_ = true;
      flight_start_time_ = now;
      prev_time_ = now;
      prev_Rd_ = R_;
      desired_rate_initialized_ = false;
      RCLCPP_INFO(get_logger(), "PX4 confirmed ARMED + OFFBOARD; starting experiment clock");
    }

    if (flight_started_ && (!armed_ || !offboard_active_ || failsafe_)) {
      fatal_exit(31, "PX4 left armed Offboard state or entered failsafe during experiment");
    }

    const double flight_t = flight_started_ ? std::max(0.0, now - flight_start_time_) : 0.0;
    const Reference ref = reference(flight_t);

    const Eigen::Vector3d ep = p_ - ref.p;
    const Eigen::Vector3d ev = v_ - ref.v;
    const Eigen::Vector3d a_cmd = ref.a - kp_pos_.cwiseProduct(ep) - kd_vel_.cwiseProduct(ev);

    const Eigen::Vector3d e3(0.0,0.0,1.0);
    Eigen::Vector3d thrust_axis = 9.80665*e3 - a_cmd;
    if (thrust_axis.norm() < 1e-6) thrust_axis = e3;
    const Eigen::Vector3d b3d = thrust_axis.normalized();
    const Eigen::Matrix3d Rd = desired_attitude(b3d, ref.yaw);

    const double f_n = std::max(0.0, mass_ * thrust_axis.dot(R_*e3));
    const double thrust_norm = clampd(f_n / max_total_thrust_n_, 0.0, max_thrust_norm_);

    const Eigen::Vector3d eR = 0.5 * vee(Rd.transpose()*R_ - R_.transpose()*Rd);
    const Eigen::Matrix3d Rrel = R_.transpose()*Rd;

    Eigen::Vector3d omega_d_raw = so3_log(prev_Rd_.transpose()*Rd) / dt;
    if (!desired_rate_initialized_) {
      omega_d_filt_ = omega_d_raw;
      prev_omega_d_filt_ = omega_d_filt_;
      omega_dot_d_filt_.setZero();
      desired_rate_initialized_ = true;
    }
    omega_d_filt_ = 0.25*omega_d_raw + 0.75*omega_d_filt_;
    Eigen::Vector3d omega_dot_raw = (omega_d_filt_ - prev_omega_d_filt_) / dt;
    omega_dot_d_filt_ = 0.15*omega_dot_raw + 0.85*omega_dot_d_filt_;
    prev_omega_d_filt_ = omega_d_filt_;
    prev_Rd_ = Rd;

    Eigen::Vector3d wsp = Rrel * omega_d_filt_ - k_att_rate_.cwiseProduct(eR);
    Eigen::Vector3d tau_norm = Eigen::Vector3d::Zero();

    if (mode_ == "rate") {
      VehicleRatesSetpoint m{};
      m.timestamp = timestamp_us();
      m.roll = static_cast<float>(wsp.x());
      m.pitch = static_cast<float>(wsp.y());
      m.yaw = static_cast<float>(wsp.z());
      m.thrust_body = {0.f, 0.f, static_cast<float>(-thrust_norm)};
      m.reset_integral = false;
      rates_pub_->publish(m);
    } else {
      const Eigen::Vector3d eOmega = omega_ - Rrel*omega_d_filt_;
      const Eigen::Vector3d Jw = inertia_ * omega_;
      const Eigen::Vector3d feed = hat(omega_) * (Rrel*omega_d_filt_) - Rrel*omega_dot_d_filt_;
      const Eigen::Vector3d M_n_m =
          -kR_.cwiseProduct(eR)
          -kOmega_.cwiseProduct(eOmega)
          +omega_.cross(Jw)
          -inertia_*feed;

      tau_norm = M_n_m.cwiseQuotient(torque_scale_nm_);
      for (int i=0;i<3;i++) tau_norm(i)=clampd(tau_norm(i), -max_torque_norm_, max_torque_norm_);

      VehicleThrustSetpoint tm{};
      tm.timestamp = timestamp_us();
      tm.timestamp_sample = tm.timestamp;
      tm.xyz = {0.f, 0.f, static_cast<float>(-thrust_norm)};
      thrust_pub_->publish(tm);

      VehicleTorqueSetpoint mm{};
      mm.timestamp = timestamp_us();
      mm.timestamp_sample = mm.timestamp;
      mm.xyz = {static_cast<float>(tau_norm.x()), static_cast<float>(tau_norm.y()), static_cast<float>(tau_norm.z())};
      torque_pub_->publish(mm);
    }

    const double att_deg = so3_angle(Rd.transpose()*R_) * 180.0 / kPi;
    csv_ << t << ',' << flight_t << ',' << mode_ << ',' << scenario_ << ','
         << p_.x() << ',' << p_.y() << ',' << p_.z() << ','
         << ref.p.x() << ',' << ref.p.y() << ',' << ref.p.z() << ','
         << v_.x() << ',' << v_.y() << ',' << v_.z() << ','
         << ref.v.x() << ',' << ref.v.y() << ',' << ref.v.z() << ','
         << ep.norm() << ',' << ev.norm() << ',' << att_deg << ','
         << omega_.x() << ',' << omega_.y() << ',' << omega_.z() << ','
         << wsp.x() << ',' << wsp.y() << ',' << wsp.z() << ','
         << thrust_norm << ',' << tau_norm.x() << ',' << tau_norm.y() << ',' << tau_norm.z() << ','
         << (armed_ ? 1 : 0) << ',' << static_cast<unsigned>(nav_state_) << ','
         << (failsafe_ ? 1 : 0) << ',' << (preflight_ok_ ? 1 : 0) << '\n';

    if (flight_started_ && flight_t > experiment_duration_) {
      if (!disarm_commanded_) {
        command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 21196.f);
        disarm_commanded_ = true;
        finish_time_ = now;
      } else if (!armed_ || now - finish_time_ > 1.5) {
        csv_.flush();
        RCLCPP_INFO(get_logger(), "Experiment complete");
        rclcpp::shutdown();
      }
    }
  }

  std::string mode_, scenario_, output_csv_;
  double control_hz_{100.0}, experiment_duration_{25.0};
  double mass_{2.0643}, max_total_thrust_n_{34.19}, max_torque_norm_{0.55}, max_thrust_norm_{0.90};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d kp_pos_, kd_vel_, k_att_rate_, kR_, kOmega_, torque_scale_nm_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_pub_;
  rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::Vector3d p_{Eigen::Vector3d::Zero()}, v_{Eigen::Vector3d::Zero()}, omega_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_{Eigen::Matrix3d::Identity()}, prev_Rd_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d omega_d_filt_{Eigen::Vector3d::Zero()}, prev_omega_d_filt_{Eigen::Vector3d::Zero()}, omega_dot_d_filt_{Eigen::Vector3d::Zero()};
  bool odom_ready_{false}, status_ready_{false}, control_mode_ready_{false}, started_{false}, flight_started_{false};
  bool armed_{false}, offboard_active_{false}, failsafe_{false}, preflight_ok_{false};
  bool disarm_commanded_{false}, desired_rate_initialized_{false}, yaw_initialized_{false};
  uint8_t nav_state_{255};
  double yaw0_{0.0};
  double start_time_{0.0}, prev_time_{0.0}, flight_start_time_{0.0}, finish_time_{0.0};
  double last_mode_command_time_{-1e9}, last_arm_command_time_{-1e9};
  std::ofstream csv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeeABController>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
