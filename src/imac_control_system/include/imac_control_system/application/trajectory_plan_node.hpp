#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <imac_interfaces/msg/global_path.hpp>
#include <imac_interfaces/msg/hils_obstacles.hpp>
#include <imac_interfaces/msg/obstacles.hpp>
#include <imac_interfaces/msg/fsm_mode.hpp>
#include <imac_interfaces/msg/vehicle_status_mav.hpp>
#include <imac_interfaces/msg/vehicle_status_hils.hpp>

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <mutex>
#include <string>
#include <cmath>

class TrajectoryPlanNode : public rclcpp::Node {
public:
    TrajectoryPlanNode();

private:
    // ===== ROS Callbacks =====
    void statusCallback(const imac_interfaces::msg::VehicleStatusMav::SharedPtr msg);
    void hilsstatusCallback(const imac_interfaces::msg::VehicleStatusHils::SharedPtr msg);
    void globalPathCallback(const imac_interfaces::msg::GlobalPath::SharedPtr msg);
    void obstaclesCallback(const imac_interfaces::msg::Obstacles::SharedPtr msg);
    void hilsobstaclesCallback(const imac_interfaces::msg::HilsObstacles::SharedPtr msg);
    void fsmModeCallback(const imac_interfaces::msg::FsmMode::SharedPtr msg);



    // ===== Main loop =====
    void planLoop();

    // ===== Utils =====
    static constexpr double kPi = 3.14159265358979323846;
    static double deg2rad(double d) { return d * kPi / 180.0; }
    static int    nearestIndex(const std::vector<Eigen::Vector2d>& path, const Eigen::Vector2d& p);
    static double minDistanceToPath(const std::vector<Eigen::Vector2d>& path, const Eigen::Vector2d& p);
    static double angleWrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    static std::tuple<Eigen::Vector2d, double, double, bool>
        nextMicroFront(const Eigen::Vector2d& p, double psi,
            const Eigen::Vector2d& p_obs,
            const std::vector<double>& dpsi_set,
            double s_step,
            const Eigen::Vector2d& goal,
            const std::vector<Eigen::Vector2d>& path_ref);

    std::vector<Eigen::Vector2d>
        findAvoid(const std::vector<Eigen::Vector2d>& ref_path, const Eigen::Vector2d& p_obs,
              int& i1, int& i2);

    static std::vector<Eigen::Vector2d>
        quarticFit(const std::vector<Eigen::Vector2d>& path_ref, const Eigen::Vector2d& p,
            int maxPts);

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
        const std::vector<Eigen::Vector2d>& pts, const std::string& frame_id);

    std::vector<double> obs_distances_;

    // ===== Parameters =====
    double loop_rate_hz_{ 20.0 };
    double s_step_init_{ 0.20 };
    std::vector<double> dpsi_set_;
    
    // ===== State from VehicleStatusMav =====
    bool   sim_initialized_{ false };
    bool         status_ok_{ false };
    rclcpp::Time status_stamp_;
    double       status_x_{ 0.0 }, status_y_{ 0.0 }, status_z_{ 0.0 };
    double       current_speed_{ 0.0 };
    double       status_yaw_{ 0.0 };

    // ===== Global path (from /vehicle/local_map, /global_path) =====
    std::mutex                        map_mtx_;

    std::vector<Eigen::Vector2d>      ref_path_;     // local path XY
    bool                              ref_path_ok_{ false };

    std::vector<Eigen::Vector2d>      global_path_;  // same frame as ref_path_ (e.g., "map")
    bool                              global_ok_{ false };

    // ===== Obstacles (from MarkerArray or LocalMap) =====
    std::vector<Eigen::Vector2d> p_obs_list_;
    int   total_obs_{ 0 };
    int   next_obs_ptr_{ 0 };
    Eigen::Vector2d p_obs_curr_{ 0,0 };
    bool has_curr_obs_{ false };


    // ===== FSM (from FsmMode) =====
    bool manual_mode_{ false };
    bool go_flag_{ true };
    bool obstacle_flag_{ false };
    bool hils_mode_{ true };
    bool avoid_zone_{ true };

    // ===== Planner runtime =====
    bool avoid_{ false };
    bool finished_{ false };
    int  current_path_idx_{ 0 };
    int  idx_e_cur_{ 0 };

    std::vector<Eigen::Vector2d> path_front_;
    std::vector<Eigen::Vector2d> back_path_curr_;
    Eigen::Vector2d p_goal_curr_{ 0,0 };
    Eigen::Vector2d pos_gen_{ 0,0 };
    double psi_gen_{ 0.0 };
    double s_step_{ 0.2 };

    // ===== ROS I/O =====
    rclcpp::Subscription<imac_interfaces::msg::VehicleStatusMav>::SharedPtr status_mav_sub_;
    rclcpp::Subscription<imac_interfaces::msg::VehicleStatusHils>::SharedPtr status_hils_sub_;
    rclcpp::Subscription<imac_interfaces::msg::GlobalPath>::SharedPtr                        global_path_sub_;
    rclcpp::Subscription<imac_interfaces::msg::Obstacles>::SharedPtr       obstacles_sub_;
    rclcpp::Subscription<imac_interfaces::msg::HilsObstacles>::SharedPtr       hils_obstacles_sub_;
    rclcpp::Subscription<imac_interfaces::msg::FsmMode>::SharedPtr              fsm_mode_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_curve_pub_;
    rclcpp::TimerBase::SharedPtr plan_timer_;
};
