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

ros::NodeHandle nh;
std_msgs::Int16MultiArray Encoder_data;
std_msgs::String color_data;

//extern char color_buffer[]='';
extern int16_t buffer[5];

ros::Publisher encoder("encoder", &Encoder_data);
ros::Publisher color_sensor("color", &color_data);
void setup(void){
	nh.initNode();
	nh.advertise(encoder);
	nh.advertise(color_sensor);
}

void loop(void){
//		color_data.data= color_buffer;
	    Encoder_data.data_length =5;
		Encoder_data.data= buffer;
		encoder.publish(&Encoder_data);
		color_sensor.publish(&color_data);
		nh.spinOnce();
}
