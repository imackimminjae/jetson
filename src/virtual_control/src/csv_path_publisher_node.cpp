#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <imac_interfaces/msg/global_path.hpp>  // ★ 추가

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <limits>

class CsvPathPublisher : public rclcpp::Node
{
public:
  CsvPathPublisher()
  : rclcpp::Node("csv_path_publisher")
  {
    // 파라미터 선언
    this->declare_parameter<std::string>("csv_path", "glocal_map.csv");
    this->declare_parameter<std::string>("path_topic", "/path");
    this->declare_parameter<std::string>("global_path_topic", "/global_path");  // ★ 추가
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_rate_hz", 1.0);  // Path 재전송 주기

    csv_path_         = this->get_parameter("csv_path").as_string();
    path_topic_       = this->get_parameter("path_topic").as_string();
    global_topic_     = this->get_parameter("global_path_topic").as_string();  // ★ 추가
    frame_id_         = this->get_parameter("frame_id").as_string();
    double rate       = this->get_parameter("publish_rate_hz").as_double();
    if (rate <= 0.0) rate = 1.0;

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    global_path_pub_ = this->create_publisher<imac_interfaces::msg::GlobalPath>(
      global_topic_, 10);  // ★ 추가

    if (!loadCsv(csv_path_, points_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load CSV path from '%s'", csv_path_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Loaded %zu points from '%s' (path_topic=%s, global_topic=%s, frame_id=%s)",
        points_.size(), csv_path_.c_str(), path_topic_.c_str(), global_topic_.c_str(), frame_id_.c_str());
    }

    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      period,
      std::bind(&CsvPathPublisher::onTimer, this));
  }

private:
  bool loadCsv(const std::string & filename,
               std::vector<std::pair<double, double>> & out_points)
  {
    out_points.clear();

    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open CSV file: %s", filename.c_str());
      return false;
    }

    std::string line;
    bool first_line = true;
    while (std::getline(ifs, line)) {
      if (line.empty()) continue;

      // 헤더 X,Y 는 스킵
      if (first_line) {
        first_line = false;
        if (line.find('X') != std::string::npos || line.find('Y') != std::string::npos) {
          continue;
        }
      }

      std::stringstream ss(line);
      std::string xs, ys;
      if (!std::getline(ss, xs, ',')) continue;
      if (!std::getline(ss, ys, ',')) continue;

      try {
        double x = std::stod(xs);
        double y = std::stod(ys);
        out_points.emplace_back(x, y);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to parse line: '%s' (%s)",
                    line.c_str(), e.what());
        continue;
      }
    }

    if (out_points.empty()) {
      RCLCPP_WARN(get_logger(), "No valid points loaded from CSV: %s", filename.c_str());
      return false;
    }
    return true;
  }

  void publishPathMsg()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = frame_id_;

    path_msg.poses.reserve(points_.size());
    for (const auto & p : points_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = p.first;
      ps.pose.position.y = p.second;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }

    path_pub_->publish(path_msg);
  }

  void publishGlobalPathMsg()
  {
    imac_interfaces::msg::GlobalPath gmsg;
    gmsg.stamp = this->now();  // builtin_interfaces/Time 타입과 호환

    gmsg.g_x.reserve(points_.size());
    gmsg.g_y.reserve(points_.size());
    for (const auto & p : points_) {
      gmsg.g_x.push_back(p.first);
      gmsg.g_y.push_back(p.second);
    }

    global_path_pub_->publish(gmsg);
  }

  void onTimer()
  {
    if (points_.empty()) return;

    publishPathMsg();        // RViz나 디버깅 용
    publishGlobalPathMsg();  // TrajectoryPlanNode용
  }

private:
  std::string csv_path_;
  std::string path_topic_;
  std::string global_topic_;  // ★ 추가
  std::string frame_id_;

  std::vector<std::pair<double, double>> points_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<imac_interfaces::msg::GlobalPath>::SharedPtr global_path_pub_;  // ★ 추가
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvPathPublisher>());
  rclcpp::shutdown();
  return 0;
}

