#pragma once

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <imac_interfaces/msg/vehicle_status_mav.hpp>
#include <imac_interfaces/msg/vehicle_status_hils.hpp>
#include <imac_interfaces/msg/virtual_control_command.hpp>
#include <imac_interfaces/msg/global_path.hpp>

#include <Eigen/Dense>

#include <limits>
#include <mutex>
#include <vector>
#include <string>

// (추가) PWM 토픽용
#include <std_msgs/msg/int32_multi_array.hpp>

namespace imac_ctrl
{

class TrackingControllerNode : public rclcpp::Node
{
public:
  TrackingControllerNode();
  ~TrackingControllerNode() override = default;

private:
  // ============================================================
  // Callbacks
  // ============================================================
  void poseCallback(const imac_interfaces::msg::VehicleStatusMav::SharedPtr msg);
  void simposeCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void globalPathCallback(const imac_interfaces::msg::GlobalPath::SharedPtr msg);
  void controlLoop();

  // ============================================================
  // Helpers
  // ============================================================
  int nearestIndex(const std::vector<Eigen::Vector2d> & path,
                   const Eigen::Vector2d & p) const;

  std::vector<Eigen::Vector2d> quarticFit(
    const std::vector<Eigen::Vector2d> & path_ref,
    const Eigen::Vector2d & p,
    int maxPts) const;

  void publishPath(const std::vector<Eigen::Vector2d> & pts,
                   const std::string & frame_id);

  // ============================================================
  // Subscribers / Publishers
  // ============================================================
  rclcpp::Subscription<imac_interfaces::msg::VehicleStatusMav>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sim_sub_;
  rclcpp::Subscription<imac_interfaces::msg::GlobalPath>::SharedPtr global_path_sub_;

  rclcpp::Publisher<imac_interfaces::msg::VirtualControlCommand>::SharedPtr control_command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_curve_pub_;
  rclcpp::Publisher<imac_interfaces::msg::VehicleStatusHils>::SharedPtr status_hils_pub_;

  // (추가) PWM 전용 토픽 퍼블리셔
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  // ============================================================
  // State Flags
  // ============================================================
  bool have_pose_{false};        // 실제 차량 상태 (옵션)
  bool sim_have_pose_{false};    // 시뮬레이터 odom 수신 여부
  bool ref_path_ok_{false};
  bool global_ok_{false};
  bool sim_initialized_{false};

  // ============================================================
  // HILS State (integrated state)
  // ============================================================
  double cur_x_{0.0};
  double cur_y_{0.0};
  double cur_z_{0.0};
  double cur_yaw_{0.0};
  double cur_speed_{0.0};

  // ============================================================
  // Simulator state (from /odom)
  // ============================================================
  double sim_x_{0.0};
  double sim_y_{0.0};
  double sim_yaw_{0.0};

  // odom twist (child_frame_id = Vehicle 기준, body frame)
  double sim_vx_body_{0.0};     // twist.linear.x  [m/s]
  double sim_vy_body_{0.0};     // twist.linear.y  [m/s]
  double sim_yaw_rate_{0.0};    // twist.angular.z [rad/s]

  // speed magnitude (for PID etc.)
  double sim_speed_{0.0};       // hypot(sim_vx_body_, sim_vy_body_)

  int current_path_idx_{0};

  // ============================================================
  // Control / Model Parameters
  // ============================================================
  double v_goal_{0.8};               // 목표 속도
  double v_set_{0.8};                // Stanley에서 사용하는 속도
  double loop_rate_hz_ctrl_{100.0};  // controlLoop 주기
  double loop_rate_hz_trj_{20.0};    // (미사용) trajectory rate
  double stanley_k_{1.0};            // Stanley gain
  double max_steer_left_{0.52};      // [rad]
  double max_steer_right_{-0.52};    // [rad]
  double s_step_init_{0.20};         // quarticFit step

  // ============================================================
  // PID gains
  // ============================================================
  double kp_{1.0};
  double ki_{0.1};
  double kd_{0.1};

  // PID internal states
  double integral_{0.0};
  double prev_err_{0.0};

  struct Cmd
  {
    double steer{0.0};     // [-1,1]
    double throttle{0.0};  // [0,1]
    double brake{0.0};     // [0,1]
  };

  // ============================================================
  // Path storage
  // ============================================================
  std::mutex map_mtx_;
  std::vector<Eigen::Vector2d> global_path_;
  std::vector<Eigen::Vector2d> ref_path_;
  std::vector<Eigen::Vector2d> path_front_;   // 확장용

  // ============================================================
  // Extra params (현재 미사용)
  // ============================================================
  std::vector<double> dpsi_set_;
};

}  // namespace imac_ctrl

