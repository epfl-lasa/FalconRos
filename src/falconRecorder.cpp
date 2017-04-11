#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64MultiArray.h"


#include <iostream>
#include <fstream>
#include <stdio.h>

#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

#define RECODING_RATE 100


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------


ofstream file_positionReal;
ofstream file_positionDesired;
ofstream file_velocityReal;
ofstream file_velocityDesired;
ofstream file_forceReal;
ofstream file_forceDesired;


//for the task adaptation module
ofstream file_velocityTask1;
ofstream file_velocityTask2;
ofstream file_beliefs;




//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// ROS callbacks to update position, velocity, and force
void RecordPositionRealCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void RecordPositionDesiredCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void RecordVelocityRealCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void RecordVelocityDesiredCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

void RecordForceRealCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void RecordForceDesiredCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);



//==============================================================================
/*
	The main function
 */
//==============================================================================

int main(int argc, char **argv)
{

	//--------------------------------------------------------------------------
	// Preparing recording folder and files
	//--------------------------------------------------------------------------

	// Creating a recording directory
	std::string recPath = "./Recordings/";
	mkdir(recPath.c_str(),0777);

	// Creating a subdirectory for a specific subject
	recPath += "TestSubject/";
	mkdir(recPath.c_str(),0777);


	// Creating a subdirectory for a specific interaction based on time
	time_t rawtime;
	tm* timeinfo;
	char buffer [80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
	recPath += string(buffer);
	mkdir(recPath.c_str(),0777);

	cout << "Recording to :" << recPath.c_str() << endl;

	std::string recPathFile_positionReal     = recPath + "/falcon_positionReal.txt";
	std::string recPathFile_positionDesired  = recPath + "/falcon_positionDesired.txt";

	std::string recPathFile_velocityReal     = recPath + "/falcon_velocityReal.txt";
	std::string recPathFile_velocityDesired  = recPath + "/falcon_velocityDesired.txt";

	std::string recPathFile_forceReal        = recPath + "/falcon_forceReal.txt";
	std::string recPathFile_forceDesired     = recPath + "/falcon_forceDesired.txt";


	file_positionReal.open(recPathFile_positionReal);
	file_positionDesired.open(recPathFile_positionDesired);

	file_velocityReal.open(recPathFile_velocityReal);
	file_velocityDesired.open(recPathFile_velocityDesired);

	file_forceReal.open(recPathFile_forceReal);
	file_forceDesired.open(recPathFile_forceDesired);


	//writing a header line
	file_positionReal     << "Time" << "\t" << "P_X" << "\t" << "P_Y" << "\t" << "P_Z" << "\n";
	file_positionDesired  << "Time" << "\t" << "P_X" << "\t" << "P_Y" << "\t" << "P_Z" << "\n";
	file_velocityReal     << "Time" << "\t" << "V_X" << "\t" << "V_Y" << "\t" << "V_Z" << "\n";
	file_velocityDesired  << "Time" << "\t" << "V_X" << "\t" << "V_Y" << "\t" << "V_Z" << "\n";
	file_forceReal        << "Time" << "\t" << "F_X" << "\t" << "F_Y" << "\t" << "F_Z" << "\n";
	file_forceDesired     << "Time" << "\t" << "F_X" << "\t" << "F_Y" << "\t" << "F_Z" << "\n";




	//--------------------------------------------------------------------------
	// Initialize the ROS topic
	//--------------------------------------------------------------------------

	ros::init(argc, argv, "falconRecorder");
	ros::NodeHandle n;

	ros::Subscriber sub_positionReal    = n.subscribe("falcon/position"               , 1000, RecordPositionRealCallback);
	ros::Subscriber sub_positionDesired = n.subscribe("/PDController/position_desired", 1000, RecordPositionDesiredCallback);
	ros::Subscriber sub_velocityReal    = n.subscribe("falcon/velocity"               , 1000, RecordVelocityRealCallback);
	ros::Subscriber sub_velocityDesired = n.subscribe("/PDController/velocity_desired", 1000, RecordVelocityDesiredCallback);
	ros::Subscriber sub_force_real      = n.subscribe("falcon/force_real"             , 1000, RecordForceRealCallback);
	ros::Subscriber sub_force_desired   = n.subscribe("falcon/force_desired"          , 1000, RecordForceDesiredCallback);



	ros::spin();

	file_positionReal.close();
	file_positionDesired.close();
	file_velocityReal.close();
	file_velocityDesired.close();
	file_forceReal.close();
	file_forceDesired.close();


	return 0;
}


//------------------------------------------------------------------------------
// OTHER FUNCTIONS
//------------------------------------------------------------------------------

void RecordPositionRealCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{


	//ROS_INFO("I heard: [%f]", msg->pose.position.x);

	file_positionReal  << msg->header.stamp 	<< "\t"
			<< msg->pose.position.x 	<< "\t"
			<< msg->pose.position.y 	<< "\t"
			<< msg->pose.position.z 	<< "\n";

}

//-----------------------------------------------------------------------------

void RecordPositionDesiredCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{


	//ROS_INFO("I heard: [%f]", msg->pose.position.x);

	file_positionDesired  << msg->header.stamp 	<< "\t"
			<< msg->pose.position.x 	<< "\t"
			<< msg->pose.position.y 	<< "\t"
			<< msg->pose.position.z 	<< "\n";

}

//-----------------------------------------------------------------------------


void RecordVelocityRealCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

	file_velocityReal  << msg->header.stamp 	<< "\t"
			<< msg->twist.linear.x 	<< "\t"
			<< msg->twist.linear.y 	<< "\t"
			<< msg->twist.linear.z 	<< "\n";

}

void RecordVelocityDesiredCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

	file_velocityDesired  << msg->header.stamp 	<< "\t"
			<< msg->twist.linear.x 	<< "\t"
			<< msg->twist.linear.y 	<< "\t"
			<< msg->twist.linear.z 	<< "\n";

}


//-----------------------------------------------------------------------------

void RecordForceRealCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{

	file_forceReal << msg->header.stamp 	<< "\t"
			<< msg->wrench.force.x 	<< "\t"
			<< msg->wrench.force.y 	<< "\t"
			<< msg->wrench.force.z 	<< "\n";

}

//-----------------------------------------------------------------------------

void RecordForceDesiredCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{

	file_forceDesired << msg->header.stamp 	<< "\t"
			<< msg->wrench.force.x 	<< "\t"
			<< msg->wrench.force.y 	<< "\t"
			<< msg->wrench.force.z 	<< "\n";
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------





