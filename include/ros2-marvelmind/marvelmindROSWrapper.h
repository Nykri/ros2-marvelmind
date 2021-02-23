#ifndef H_MARVELMINDROSWRAPPER
#define H_MARVELMINDROSWRAPPER


extern "C" {
	#include "marvelmind.h"
}

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::placeholders;

class marvelmindWrapper
{
public:
	marvelmindWrapper(std::shared_ptr<rclcpp::Node> nh);

	bool connect();

	bool disconnect();


private:
	struct MarvelmindHedge* hedge;

	std::shared_ptr<rclcpp::Node> nh;

	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
	rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_publisher;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

	rclcpp::TimerBase::SharedPtr timer_marvelmind_publish;

	void publishMarvelmindReadings();
};






















#endif
