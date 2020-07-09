#include <iostream>
#include <string>
#include <ros/ros.h>
#include "cf_hmi_include/typedefs.h"
#include "cf_hmi_include/serial.h"
#include "cf_hmi_include/cf_packet.h"
#include "cf_hmi_include/show_packet.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ros/callback_queue.h>

extern "C" void ShowReceivedPacket(void);
extern "C" int Serial_Init(char *devname, int baud_rate);
extern "C" dword BytesAvail(void);
extern "C" ubyte GetByte(void);
extern "C" void send_packet(void);
extern "C" ubyte check_for_packet(void);

ros::Publisher command_pub;
ros::Publisher enable_pub;
ros::Publisher disable_pub;

void recvString(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO_STREAM("string: " << msg->data.length());
    outgoing_response.command = 31;
    outgoing_response.data[0]=0; //col
    outgoing_response.data[1]=0; //row
	memcpy(&outgoing_response.data[2],"                ", 16);  // 16 spaces
    memcpy(&outgoing_response.data[2],msg->data.c_str(), msg->data.length());
    outgoing_response.data_length = 18;
    send_packet();
}

void recvString2(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO_STREAM("string: " << msg->data.length());
    outgoing_response.command = 31;
    outgoing_response.data[0]=0; //col
    outgoing_response.data[1]=1; //row
	memcpy(&outgoing_response.data[2],"                ", 16);  // 16 spaces
    memcpy(&outgoing_response.data[2],msg->data.c_str(), msg->data.length());
    outgoing_response.data_length = 18;
    send_packet();
}

void showButtonPress() {

	std_msgs::String command;
    std_msgs::Empty empty;

	switch(incoming_command.CRC.as_word) {
		case 42823:  std::cout<<"UP"<< std::endl; command.data = "UP"; break;
		case 31906:  std::cout<<"Right"<< std::endl; command.data = "RIGHT"; break;
		case 20025:  std::cout<<"LEFT"<< std::endl; command.data = "LEFT"; break;
		case 24496:  std::cout<<"DOWN"<< std::endl; command.data = "DOWN"; break;
		case 27947:  
			std::cout<<"ENTER"<< std::endl; 
			command.data = "ENTER"; 
			enable_pub.publish(empty);
			break;
		case 6548:   
			std::cout<<"EXIT"<< std::endl; 
			command.data = "EXIT";
			disable_pub.publish(empty);
			break;
		default: return;
	}

	command_pub.publish(command);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hmi_controller");
	ros::NodeHandle n;
	ros::Rate r(30);

	int baud = 19200;
	char* port = "/dev/ttyUSB0";

	if(Serial_Init(port,baud))
	{
		printf("Could not open port \"%s\" at \"%d\" baud.\n",port,baud);
		return(1);
	}
	else
		printf("\"%s\" opened at \"%d\" baud.\n\n",port,baud);

	while(BytesAvail())
    	GetByte();

    outgoing_response.command = 31;
    outgoing_response.data[0]=0; //col
    outgoing_response.data[1]=0; //row
    memcpy(&outgoing_response.data[2],">Hello VSI Labs<",16);
    outgoing_response.data_length = 18;
    send_packet();

    outgoing_response.command = 31;
    outgoing_response.data[0]=0; //col
    outgoing_response.data[1]=1; //row
    memcpy(&outgoing_response.data[2],">This is line 2<",16);
    outgoing_response.data_length = 18;
    send_packet();

	ros::Subscriber sub = n.subscribe("hmi_line1", 1, recvString);
	ros::Subscriber sub2 = n.subscribe("hmi_line2", 1, recvString2);

	command_pub = n.advertise<std_msgs::String>("hmi_command", 1);
	enable_pub = n.advertise<std_msgs::Empty>("/vehicle/enable", 1);
	disable_pub = n.advertise<std_msgs::Empty>("/vehicle/disable", 1);

	while (ros::ok())
	{
    	if(check_for_packet())
			showButtonPress(); 

		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}
	
	return 0;
}
