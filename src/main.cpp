// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <signal.h>
#include "spline.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/RobotState.h>

#include "examples_common.h"
#include "Controller.h"

#include <iostream>
#include <fstream> 
#include <jsoncpp/json/json.h>
#pragma comment(lib, "jsoncpp.lib")
using namespace std;
typedef std::array<double,7> J;
J joint_states={0,0,0,0,0,0,0};
J q_d = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
J dq_d = {0,};
J coriolis={0,};
void ctrlchandler(int){exit(EXIT_SUCCESS);}
void killhandler(int){exit(EXIT_SUCCESS);}
franka::RobotState robot_state;
bool desired_recv=false;
bool is_sim = true;

void rosDesiredJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i = 0;i<7;i++){
		q_d.at(i) = msg->position[i];
		dq_d.at(i) = msg->position[i];
	}
}

void printJ(J input,char* str){
	std::cout<<"\t"<<str<<" : ";
	for(int i = 0;i<6;i++)
		std::cout<<input.at(i)<<",";
	std::cout<<input.at(6)<<std::endl;
}


bool ReadFromFile(const char* filename, char* buffer, int len){
	FILE* r = fopen(filename,"rb");
	if (NULL == r)
	   return false;
	size_t fileSize = fread(buffer, 1, len, r);
	fclose(r);
	return true;

}
bool loadJson(Json::Value& input,char* JSON_FILE){
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile(JSON_FILE, readBuffer, BufferLength)) 
	  return 0;
	std::string config_doc = readBuffer;

	Json::Reader reader;
	bool parsingSuccessful = reader.parse(config_doc,input);
	if ( !parsingSuccessful ) { 
	std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
	return 0; 
	}
	return 1;

}


int main(int argc, char** argv) {
	if(argc == 2){

	}else{
		std::cout<<"###PLEASE JSON FILE NAME###"<<std::endl;
		std::cout<<"./controller <json file name> "<<std::endl;
		return -1;
	}

    //***************************JSON LOAD START****************************************//
    char* JSON_FILE= argv[1];
    std::string robot_name = "panda_robot";

	Json::Value rootr;
	bool ret = loadJson(rootr, JSON_FILE);

    J q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
	if(ret == 1)
		for(int i = 0;i<7;i++)
			q_goal.at(i) = rootr[robot_name]["init_q"][i].asFloat();
	else
		return 0;
	for(int i = 0;i<7;i++)
		robot_state.q_d.at(i) =q_goal.at(i);

	std::string robot_ip = rootr[robot_name]["robot_ip"].asString();





    //***************************JSON LOAD END****************************************//


	try{ros::init(argc,argv,"trajectory_subscriber");}
	catch(int e){ctrlchandler(1);}


	ros::NodeHandle nh;
	ros::Subscriber desired_sub = nh.subscribe("/desired_joint_states", 1,rosDesiredJointStateCallback); 

	ros::Rate r(1000);
	std::cout<<"ROS JOINT CONTROLLER IS ON"<<std::endl;

	unsigned int count = 0;
		//*******************************************REAL ROBOT****************************************************************//
		struct {
			
		} print_data{};
		try{

			franka::Robot robot(robot_ip);
			setDefaultBehavior(robot);
			std::cout << "Conection Established"<<std::endl;
	    	MotionGenerator motion_generator(0.5, q_goal);
			std::cout << "경고: 이 예제는 로봇이 움직입니다. "
			          << "EMERGENCY버튼을 손에 들고 있어주세요" << std::endl
			          << "Enter키를 누르면 시직합니다." << std::endl;
			std::cin.ignore();
			robot.control(motion_generator);
		    robot.setCollisionBehavior(
	        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
	        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
	        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
	        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
			const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
			const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
		    franka::Model model = robot.loadModel();
			while (ros::ok()){

					std::function<franka::Torques(const franka::RobotState&, franka::Duration)> impedance_control_callback =
			        [&print_data, &model, k_gains, d_gains](const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
						// Read current coriolis terms from model.
						std::array<double, 7> coriolis = model.coriolis(state);
						std::array<double, 7> tau_d_calculated;
						for (size_t i = 0; i < 7; i++) {
						tau_d_calculated[i] =
						    k_gains[i] * (q_d[i] - state.q[i]) - d_gains[i] * dq_d[i] + coriolis[i];
						}
						std::array<double, 7> tau_d_rate_limited =
						        franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
						// Send torque command.
						return tau_d_rate_limited;
				};
	   			
	   			robot.control(impedance_control_callback);
				ros::spinOnce();
				r.sleep();
			}
		} catch (const franka::Exception& ex) {
		    std::cerr << ex.what() << std::endl;
		  }

}


