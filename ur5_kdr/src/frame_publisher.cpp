// Script for Drawing and Updating Frames from the Kinematics & Dynamics of Robots Matlab Interface

// Author: Mengze Xu, 07/24/2017
// Updated By: Jakub Piwowarczyk, 08/27/2023
// Updated By: Jeremy Zhang, 11/25/2023

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/transform_stamped.h"
#include <vector>
#include <string>
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MultiFramePublisher : public rclcpp::Node
{
public:
	MultiFramePublisher()
		: Node("kdr_frame_publisher")
	{
		subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
			"kdr/tf_msg", 10, std::bind(&MultiFramePublisher::updateFrameList, this, _1));

		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		timer_ = this->create_wall_timer(
			1500ms, std::bind(&MultiFramePublisher::timer_callback, this));
	}

private:
	// timeCallback
	void timer_callback()
	{
		if (!frame_list.empty())
		{
			// if frame list is not empty
			geometry_msgs::msg::TransformStamped msg;
			for (uint i = 0; i < frame_list.size(); i++)
			{
				msg.header.stamp = this->get_clock()->now();
				msg.header.frame_id = frame_list[i].header.frame_id;
				msg.child_frame_id = frame_list[i].child_frame_id;
				msg.transform.translation.x = frame_list[i].transform.translation.x;
				msg.transform.translation.y = frame_list[i].transform.translation.y;
				msg.transform.translation.z = frame_list[i].transform.translation.z;
				msg.transform.rotation = frame_list[i].transform.rotation;
				tf_broadcaster_->sendTransform(msg);
			}
		}
		else
		{
			// ROS_INFO("Warning: frame list is empty");
		}
	}

	// update
	void updateFrameList(const geometry_msgs::msg::TransformStamped &msg)
	{
		// ROS_INFO("Receive Frame Update!");
		std::string msg_name = msg.child_frame_id;
		std::string exist_name;

		for (uint i = 0; i < frame_list.size(); i++)
		{
			exist_name = frame_list[i].child_frame_id;
			if (msg_name.compare(exist_name) == 0)
			{
				// existed frames
				if (msg.header.frame_id.compare("Delete") == 0)
				{
					// ROS_INFO("Delete Frame");
					frame_list.erase(frame_list.begin() + i);
					return;
				}
				else
				{
					// ROS_INFO("Existing frame");
					frame_list[i] = msg;
					return;
				}
			}
		}

		// ROS_INFO("New Frame");
		frame_list.push_back(msg);
		std::cout << "list size" << frame_list.size() << std::endl;
		this->timer_callback();
	}

	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
	std::vector<geometry_msgs::msg::TransformStamped> frame_list; // vector to store frames
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MultiFramePublisher>());
	rclcpp::shutdown();
	return 0;
}
