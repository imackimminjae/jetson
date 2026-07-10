#include "imac_control_system/application/vehicle_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <atomic>  
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/path.hpp>

#include <imac_interfaces/msg/fsm_mode.hpp>
#include <imac_interfaces/msg/vehicle_status_mav.hpp>
#include <imac_interfaces/msg/vehicle_status_hils.hpp>


using std::placeholders::_1;
using Eigen::Vector2d;

namespace imac_ctrl {

    constexpr int PWM_FREQUENCY = 1000;   // Hz

    // 메뉴얼 모드(조이스틱)
    bool throttle_on = false;
    bool button_pressed = false;
    float last_duty_steering_ = -1.0f;
    float last_duty_pedal_ = -1.0f;

    imac_interfaces::msg::VehicleStatusHils st;
    
    inline double deg2rad(double d){
        constexpr double PI = 3.14159265358979323846;
        return d * PI / 180.0;
    }

    VehicleControllerNode::VehicleControllerNode() : Node("vehicle_controller_node")
    {
        this->get_parameter("loop_rate_hz", loop_rate_hz_);
        this->get_parameter("stanley_k", stanley_k_);
        this->get_parameter("max_steer", max_steer_);
        this->get_parameter("v_set", v_set_);
        double yaw_deg = std::numeric_limits<double>::quiet_NaN();


        RCLCPP_INFO(get_logger(), "VehicleControllerNode @ %.1f Hz (v_set=%.2f)",
            loop_rate_hz_, v_set_);


        // subs
        status_mav_sub_ = this->create_subscription<imac_interfaces::msg::VehicleStatusMav>(
            "/vehicle/status_mav", 10, std::bind(&VehicleControllerNode::poseCallback, this, _1));
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/controller/local_curve", 10, std::bind(&VehicleControllerNode::pathCallback, this, _1));
        fsm_mode_sub_ = this->create_subscription<imac_interfaces::msg::FsmMode>(
            "/fsm_mode", 10,
            std::bind(&VehicleControllerNode::fsmModeCallback, this, std::placeholders::_1));

        // pubs

        status_hils_pub_ = this->create_publisher<imac_interfaces::msg::VehicleStatusHils>("/vehicle/status_hils", 10);

        // timer
        auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
        loop_timer_ = create_wall_timer(period, std::bind(&VehicleControllerNode::controlLoop, this));
    }

    void VehicleControllerNode::fsmModeCallback(const imac_interfaces::msg::FsmMode::SharedPtr msg)
    {
      RCLCPP_INFO(get_logger(), "Iam here0");
        manual_mode_ = 0;//(msg->manual != 0);
        obstacle_flag_ = 1;//(msg->obstacle != 0);
        go_flag_ = 1;//(msg->go != 0);sor
        hils_mode_ = 1;//(msg->hils != 0);
        avoid_zone_ = 1;//(msg->avoid != 0);

        if (1) {
        RCLCPP_INFO(get_logger(), "Iam here0-1");
            cur_x_ = 0.0; cur_y_ = 0.0; cur_yaw_ = 0.0; cur_speed_ = 0.0;
            sim_initialized_ = true;
            have_pose_ = true;
            RCLCPP_INFO(this->get_logger(),
            	"[FSM] Enter HILS -> forced init to zeros");
        }
    }

    void VehicleControllerNode::poseCallback(const imac_interfaces::msg::VehicleStatusMav::SharedPtr msg) {
          RCLCPP_INFO(get_logger(), "Iam here00");
        if (hils_mode_ == 0) {
            cur_x_ = msg->x;
            cur_y_ = msg->y;
            cur_z_ = msg->z;
            cur_speed_ = msg->speed;
            cur_yaw_ = msg->heading;

            have_pose_ = true;
        }
    }

    void VehicleControllerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
          RCLCPP_INFO(get_logger(), "Iam here000");
        path_ = msg;
        have_path_ = true; 
    }



        void VehicleControllerNode::controlLoop()
    {

    	status_hils_pub_->publish(st);
        if (1){
        float steer_neutral = 0.f; // [µs] 서보 중앙
        float steer_scale = 0.f;  // ±max_steer → ± µs
        float duty_pedal_ = 0.f; // 목표 속도용 스로틀 PWM
        float duty_steering_ = 0.f;


       //RCLCPP_INFO(get_logger(), "%d %d", have_pose_, path_);
        if (!have_path_) return; //path도 불 아님.
        		

        const auto& poses = path_->poses;

        if (poses.size() < 2) return;
                RCLCPP_INFO(get_logger(), "Iam here2");
        // 경로 포인트
        std::vector<Vector2d> pts; pts.reserve(poses.size());
        for (const auto& ps : poses)
            pts.emplace_back(ps.pose.position.x, ps.pose.position.y);

        // 현재 자세
        Vector2d pos{ cur_x_, cur_y_ };
        double   yaw = cur_yaw_;

        // 최근접 인덱스
        size_t i_near = 0; double dmin = 1e18;
        for (size_t i = 0; i < pts.size(); ++i) {
            double d2 = (pts[i] - pos).squaredNorm();
            if (d2 < dmin) { dmin = d2; i_near = i; }
        }
        const size_t i_prev = (i_near > 0) ? i_near - 1 : i_near;
        const size_t i_next = std::min(i_near + 1, pts.size() - 1);

        // 경로 헤딩(차분)
        double path_yaw;
        if (i_next != i_near) {
            path_yaw = std::atan2(pts[i_next].y() - pts[i_near].y(),
                pts[i_next].x() - pts[i_near].x());
        }
        else if (i_prev != i_near) {
            path_yaw = std::atan2(pts[i_near].y() - pts[i_prev].y(),
                pts[i_near].x() - pts[i_prev].x());
        }
        else {
               
            return; // 포인트 1개
	RCLCPP_INFO(get_logger(), "Iam here3");
        }

        auto normAng = [](double a) { return std::atan2(std::sin(a), std::cos(a));
        };
	RCLCPP_INFO(get_logger(), "Iam here4"); 
        // CTE
        double dx = pos.x() - pts[i_near].x(), dy = pos.y() - pts[i_near].y();
        double cte = dx * std::sin(path_yaw) - dy * std::cos(path_yaw);

        // 헤딩 에러
        double heading_err = normAng(path_yaw - yaw);

        // Stanley
        const double v_for = std::max(0.05, v_set_); // 0 나눗셈 방지
        double steer_cmd = heading_err + std::atan2(stanley_k_ * cte, v_for);
        steer_cmd = std::clamp(steer_cmd, -max_steer_, max_steer_);


        if (1) {
            const double dt = 1.0 / loop_rate_hz_;
            const double v_cmd = (go_flag_ ? v_set_ : 0.0);

            // sim_v_ 가 v_cmd 로 지연 수렴. speed_tau_는 파라미터(예: 0.6s)
            if (speed_tau_ > 1e-6) {
                sim_v_ += (v_cmd - sim_v_) * (dt / speed_tau_);
            }
            else {
                sim_v_ = v_cmd;
            }
            const double v_eff = sim_v_;

            // ── 모델 적분 ──────────────────────────────────
            const double L = 0.7;  // 휠베이스 (필요 시 파라미터화)
            const double yaw_rate = v_eff * std::tan(steer_cmd) / L;

            sim_x_ += v_eff * std::cos(sim_yaw_) * dt;
            sim_y_ += v_eff * std::sin(sim_yaw_) * dt;
            sim_yaw_ = normAng(sim_yaw_ + yaw_rate * dt);

            // ── 상태 퍼블리시: 초기 주입된 값이 그대로 반영됨 ──────

            cur_x_ = sim_x_;
            cur_y_ = sim_y_;
            cur_z_ = 0.0;
            cur_speed_ = v_eff;
            cur_yaw_ = sim_yaw_;
            
            st.x = sim_x_;
            st.y = sim_y_;
            st.z = 0.0;
            st.speed = v_eff;
            st.heading = sim_yaw_;

            RCLCPP_INFO(get_logger(), "Iam here");
        }
        }
    }








} // namespace imac_ctrl

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imac_ctrl::VehicleControllerNode>());
    rclcpp::shutdown();
    return 0;
}
