#include "imac_control_system/comm/udp_publisher_node.hpp"
#include <sstream>  // 문자열 스트림

using std::placeholders::_1;

namespace imac_udp
{

UdpPublisherNode::UdpPublisherNode() : Node("udp_publisher_node")
{
  // ───────────────────────────────────────────────
  // 파라미터 설정: UDP 전송 대상 IP 및 포트
  // ───────────────────────────────────────────────
  this->declare_parameter("remote_ip", "192.168.0.20");
  this->declare_parameter("remote_port", 14500);
  this->get_parameter("remote_ip", remote_ip_);
  this->get_parameter("remote_port", remote_port_);
  // ───────────────────────────────────────────────
  // UDP 송신 소켓 생성 및 주소 초기화
  // ───────────────────────────────────────────────
  sock_ = socket(AF_INET, SOCK_DGRAM, 0); // UDP 소켓 생성
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port = htons(remote_port_);
  inet_aton(remote_ip_.c_str(), &remote_addr_.sin_addr);
  // ───────────────────────────────────────────────
  // ROS 토픽 구독: GNSS 기반 위치 정보 수신
  // ───────────────────────────────────────────────
  status_mav_sub_ = this->create_subscription<imac_interfaces::msg::VehicleStatusMav>(
    "/vehicle/status_mav", 10, std::bind(&UdpPublisherNode::statusMavCallback, this, _1));

  RCLCPP_INFO(this->get_logger(), "UDP Publisher started → %s:%d",
              remote_ip_.c_str(), remote_port_);
}

UdpPublisherNode::~UdpPublisherNode()
{
  // 소켓 종료
  close(sock_);
}

// ───────────────────────────────────────────────
// PoseGns 메시지를 수신했을 때 콜백 함수
// → 문자열로 포맷팅하여 UDP로 전송
// ───────────────────────────────────────────────
void UdpPublisherNode::statusMavCallback(const imac_interfaces::msg::VehicleStatusMav::SharedPtr msg)
{
  std::ostringstream oss;
  oss << "GNS," << msg->x << "," << msg->y << "," << msg->z << "," << msg->heading;
  sendUdpPacket(oss.str());
}

// ───────────────────────────────────────────────
// 포맷된 문자열을 UDP 패킷으로 전송
// ───────────────────────────────────────────────
void UdpPublisherNode::sendUdpPacket(const std::string &payload)
{
  sendto(sock_,
         payload.c_str(),
         payload.size(),
         0,
         reinterpret_cast<struct sockaddr *>(&remote_addr_),
         sizeof(remote_addr_));
}

}  // namespace imac_udp

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imac_udp::UdpPublisherNode>());
  rclcpp::shutdown();
  return 0;
}