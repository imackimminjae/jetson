#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <imac_interfaces/msg/fsm_mode.hpp>
#include <imac_interfaces/msg/vehicle_status_mav.hpp>
#include <imac_interfaces/msg/vehicle_status_hils.hpp>
#include <Eigen/Dense>
#include <atomic>

namespace imac_ctrl {

	class VehicleControllerNode : public rclcpp::Node {

	public:
		VehicleControllerNode();

	private:
		// callbacks
		void poseCallback(const imac_interfaces::msg::VehicleStatusMav::SharedPtr msg);      
		void hilsposeCallback(const imac_interfaces::msg::VehicleStatusHils::SharedPtr msg);  
		void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
		void fsmModeCallback(const imac_interfaces::msg::FsmMode::SharedPtr msg);
		void controlLoop();


		// subs
		rclcpp::Subscription<imac_interfaces::msg::VehicleStatusMav>::SharedPtr status_mav_sub_;
		rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
		rclcpp::Subscription<imac_interfaces::msg::FsmMode>::SharedPtr              fsm_mode_sub_;

		// pubs
		rclcpp::Publisher<imac_interfaces::msg::VehicleStatusHils>::SharedPtr status_hils_pub_;

		rclcpp::TimerBase::SharedPtr loop_timer_;

		std::atomic<int> hils_mode_{ true };
		std::atomic<bool> manual_mode_{ false };
		std::atomic<bool> obstacle_flag_{ false };
		std::atomic<bool> go_flag_{ true };
		std::atomic<bool> avoid_zone_{ true };

		// state from status_mav
		bool   have_pose_{ false };
		bool   have_path_{false}; // IMAC
		double cur_x_{ 0.0 }, cur_y_{ 0.0 }, cur_z_{ 0.0 }, cur_yaw_{ 0.0 };
		double cur_speed_{ 0.0 };


		// last path
		nav_msgs::msg::Path::SharedPtr path_;

		// ---- HILS 초기 파라미터 ----
		double init_x_{ std::numeric_limits<double>::quiet_NaN() };
		double init_y_{ std::numeric_limits<double>::quiet_NaN() };
		double init_yaw_rad_{ std::numeric_limits<double>::quiet_NaN() };
		double init_speed_{ std::numeric_limits<double>::quiet_NaN() };

		// ---- HILS 시뮬 상태 ----
		bool   sim_initialized_{ false };
		double sim_x_{ 0.0 }, sim_y_{ 0.0 }, sim_z_{ 0.0 }, sim_yaw_{ 0.0 };
		double sim_v_{ 0.0 };

		// ---- 모델 파라미터 ----
		double speed_tau_{ 0.6 };   // 속도 1차 지연 상수(s)
		double wheelbase_{ 0.7 };   // 자전거 모델 휠베이스

		// params
		double loop_rate_hz_{ 50.0 };
		double stanley_k_{ 1.0 };
		double max_steer_{ 0.35 }; // [rad]
		double v_set_{ 0.8 };      // [m/s]
	};

} // namespace imac_ctrl

