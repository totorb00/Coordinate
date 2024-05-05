/*
 * mainpp.cpp
 *
 *  Created on: Mar 1, 2024
 *      Author: sanji
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle nh;
//std_msgs::String str_msg;
std_msgs::Int16MultiArray Encoder_data;
extern int16_t buffer[5];
//char hello[] = "Hello world from STM32!";
//extern int16_t sensor_buff[5];
//extern int16_t sensor_buff[5];
//ros::Publisher chatter("chatter", &str_msg);
ros::Publisher encoder("encoder", &Encoder_data);

void setup(void){
	nh.initNode();
//	nh.advertise(chatter);
	nh.advertise(encoder);
}

void loop(void){

//	str_msg.data = hello;
//	chatter.publish(&str_msg);
//	nh.spinOnce();
//	HAL_Delay(1);
	    Encoder_data.data_length =5;
		Encoder_data.data= buffer;
		encoder.publish(&Encoder_data);
		nh.spinOnce();
}
