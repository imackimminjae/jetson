#pragma once

#include "rclcpp/rclcpp.hpp"
#include "imac_interfaces/msg/vehicle_status_mav.hpp"

#include <string>
#include <array>
#include <netinet/in.h>  // sockaddr_in
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace imac_udp
{

class UdpPublisherNode : public rclcpp::Node
{
public:
  UdpPublisherNode();
  ~UdpPublisherNode();

private:
  void statusMavCallback(const imac_interfaces::msg::VehicleStatusMav::SharedPtr msg);
  void sendUdpPacket(const std::string &payload);

  // ROS
  rclcpp::Subscription<imac_interfaces::msg::VehicleStatusMav>::SharedPtr status_mav_sub_;

  // UDP socket
  int sock_;
  struct sockaddr_in remote_addr_;

  // 설정 파라미터
  std::string remote_ip_;
  int remote_port_;
};

}  // namespace imac_udp
