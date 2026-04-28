/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <rclcpp/rclcpp.hpp>
#include <iostream>
// #include <champ/utils/urdf_loader.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/JointWrench.hh"
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <champ_msgs/msg/contacts_stamped.hpp>

class ContactSensor: public rclcpp::Node
{
	// bool foot_contacts_[4];
	// std::vector<std::string> foot_links_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr foot_contact_force_publisher_;
	gazebo::transport::SubscriberPtr gazebo_sub;
	boost::shared_ptr<const gazebo::msgs::Contacts> contacts_msg_ = nullptr;
	std::mutex mutex_;
    
	public:
		ContactSensor():
			// foot_contacts_ {false,false,false,false},
			Node("contacts_sensor",rclcpp::NodeOptions()
					.allow_undeclared_parameters(true)
					.automatically_declare_parameters_from_overrides(true))
		{
			// std::vector<std::string> joint_names;

			// joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());
			// foot_links_.push_back(joint_names[2]);
			// foot_links_.push_back(joint_names[6]);
			// foot_links_.push_back(joint_names[10]);
			// foot_links_.push_back(joint_names[14]);

			foot_contact_force_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("foot_contact_forces", 10);

			gazebo::client::setup();
			gazebo::transport::NodePtr node(new gazebo::transport::Node());
			node->Init();

			gazebo_sub = node->Subscribe("~/physics/contacts", &ContactSensor::gazeboCallback_, this);
		}

		void gazeboCallback_(ConstContactsPtr &_msg)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			// RCLCPP_INFO(this->get_logger(), "Received %d contacts", _msg->contact_size());
			contacts_msg_ = _msg;
		}

		void publishContacts()	
		{
			std::lock_guard<std::mutex> lock(mutex_);

			if (!contacts_msg_) 
			{
				RCLCPP_WARN(this->get_logger(), "No contacts message received yet.");
				return;
			}

			sensor_msgs::msg::JointState joint_state_msg;
			joint_state_msg.header.stamp = this->get_clock()->now();
			joint_state_msg.name.resize(4);
			joint_state_msg.position.resize(12);
			joint_state_msg.velocity.resize(12);
			joint_state_msg.effort.resize(12);

			for (int i = 0; i < contacts_msg_->contact_size(); ++i) 
			{
				// RCLCPP_INFO(this->get_logger(), "Contact %d", i);
				// std::vector<std::string> results;
				std::string collision1 = contacts_msg_->contact(i).collision1();
				std::string collision2 = contacts_msg_->contact(i).collision2();
				// boost::split(results, collision, [](char c){return c == ':';});

				// RCLCPP_INFO(this->get_logger(), "Collision1: %s", collision1.c_str());
				// RCLCPP_INFO(this->get_logger(), "Collision2: %s", collision2.c_str());

				// gazebo::physics::JointWrench wrench = _msg->contact(i).wrench;
				for (int j = 0; j < contacts_msg_->contact(i).wrench().size(); ++j)
				{
					gazebo::msgs::JointWrench wrench = contacts_msg_->contact(i).wrench(j);
					// Body 1: robot
					// RCLCPP_INFO(this->get_logger(), "Wrench %d: body 1 (%s) force: (%f, %f, %f)", 
					// 	j, 
					// 	wrench.body_1_name().c_str(),
					// 	wrench.body_1_wrench().force().x(), 
					// 	wrench.body_1_wrench().force().y(), 
					// 	wrench.body_1_wrench().force().z());
					// // Body 2: ground
					// RCLCPP_INFO(this->get_logger(), "Wrench %d: body 2 (%s) force: (%f, %f, %f)", 
					// 	j, 
					// 	wrench.body_2_name().c_str(),
					// 	wrench.body_2_wrench().force().x(), 
					// 	wrench.body_2_wrench().force().y(), 
					// 	wrench.body_2_wrench().force().z());

					if (wrench.body_1_name().find("FL_foot") != std::string::npos)
					{
						// go2::FL_calf::FL_calf_fixed_joint_lump__FL_foot_collision_1
						joint_state_msg.name[0] = "FL_foot_contact";
						joint_state_msg.effort[0] = wrench.body_1_wrench().force().x();
						joint_state_msg.effort[1] = wrench.body_1_wrench().force().y();
						joint_state_msg.effort[2] = wrench.body_1_wrench().force().z();
					} else if (wrench.body_1_name().find("FR_foot") != std::string::npos)
					{
						// go2::FR_calf::FR_calf_fixed_joint_lump__FR_foot_collision_1
						joint_state_msg.name[1] = "FR_foot_contact";
						joint_state_msg.effort[3] = wrench.body_1_wrench().force().x();
						joint_state_msg.effort[4] = wrench.body_1_wrench().force().y();
						joint_state_msg.effort[5] = wrench.body_1_wrench().force().z();
					} else if (wrench.body_1_name().find("RL_foot") != std::string::npos)
					{
						// go2::RL_calf::RL_calf_fixed_joint_lump__RL_foot_collision_1
						joint_state_msg.name[2] = "RL_foot_contact";
						joint_state_msg.effort[6] = wrench.body_1_wrench().force().x();
						joint_state_msg.effort[7] = wrench.body_1_wrench().force().y();
						joint_state_msg.effort[8] = wrench.body_1_wrench().force().z();
					} else if (wrench.body_1_name().find("RR_foot") != std::string::npos)
					{
						// go2::RR_calf::RR_calf_fixed_joint_lump__RR_foot_collision_1
						joint_state_msg.name[3] = "RR_foot_contact";
						joint_state_msg.effort[9] = wrench.body_1_wrench().force().x();
						joint_state_msg.effort[10] = wrench.body_1_wrench().force().y();
						joint_state_msg.effort[11] = wrench.body_1_wrench().force().z();
					} else 
					{
						// RCLCPP_INFO(this->get_logger(), "Unknown foot contact detected");
					}
				}
			}

			// for (int i = 0; i < joint_state_msg.name.size(); ++i)
			// {
			// 	if (joint_state_msg.name[i].empty())
			// 	{
			// 		RCLCPP_WARN(this->get_logger(), "Joint name %d is empty, skipping", i);
			// 		continue;
			// 	}
			// 	RCLCPP_INFO(this->get_logger(), "Joint %s: force (%f, %f, %f)", 
			// 		joint_state_msg.name[i].c_str(),
			// 		joint_state_msg.effort[i*3], 
			// 		joint_state_msg.effort[i*3 + 1], 
			// 		joint_state_msg.effort[i*3 + 2]);
			// }

			foot_contact_force_publisher_->publish(joint_state_msg);
		}
};

void exitHandler(int sig)
{
	gazebo::client::shutdown();
	rclcpp::shutdown();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ContactSensor>();
	rclcpp::WallRate loop_rate(50);

	while (rclcpp::ok())
	{
		node->publishContacts();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}