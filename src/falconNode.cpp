#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include <iostream>
#include <stdio.h>

#include "chai3d.h"

// You need to #include <memory> in order for the compiler to know about std::shared_ptr.
//#include <memory>

//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------


// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a global variable to store the position [m], velocity [m/s], and force [N?] of the haptic device
cVector3d hapticDevicePosition;
cVector3d hapticDeviceVelocity;
cVector3d hapticDeviceForce_real;
cVector3d hapticDeviceForce_desired;

// flags to indicate if the haptic simulation currently running or has terminated 
bool simulationRunning = false;
bool simulationFinished = true;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;



//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// callbeack when a new ROS message is recieved
void chatterCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);




//==============================================================================
/*
	The main function
 */
//==============================================================================


int main(int argc, char **argv)
{

	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();
	handler->update();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// open a connection with the haptic device
	hapticDevice->open();
	hapticDevice->calibrate();

	// retrieve information about the current haptic device
	cHapticDeviceInfo info = hapticDevice->getSpecifications();

	//--------------------------------------------------------------------------
	// Initialize the ROS topic
	//--------------------------------------------------------------------------

	ros::init(argc, argv, "falconNode");
	ros::NodeHandle n;

	ros::Publisher pub_position = n.advertise<geometry_msgs::PoseStamped>("falcon/position", 1000);
	ros::Publisher pub_velocity = n.advertise<geometry_msgs::TwistStamped>("falcon/velocity", 1000);
	ros::Publisher pub_force    = n.advertise<geometry_msgs::WrenchStamped>("falcon/force_real", 1000);

	ros::Subscriber sub_force = n.subscribe("falcon/force_desired", 1000, chatterCallback);

	ros::Rate loop_rate(1000);



	//--------------------------------------------------------------------------
	// START HAPTIC SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	//hapticsThread->start(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);


	// setup callback when application exits
	atexit(close);




	//--------------------------------------------------------------------------
	// MAIN ROS LOOP
	//--------------------------------------------------------------------------

	ros::Time::now();



	int count = 0;
	while (ros::ok())
	{


		//cout << freqCounterHaptics << endl;

		ros::Time time_now = ros::Time::now();

		std_msgs::String msg;

		geometry_msgs::PoseStamped msg_position;
		msg_position.header.stamp = time_now;
		msg_position.header.frame_id = "/end_effector";
		msg_position.pose.position.x = hapticDevicePosition(0);
		msg_position.pose.position.y = hapticDevicePosition(1);
		msg_position.pose.position.z = hapticDevicePosition(2);
		pub_position.publish(msg_position);


		geometry_msgs::TwistStamped msg_velocity;
		msg_velocity.header.stamp = time_now;
		msg_velocity.header.frame_id = "/end_effector";
		msg_velocity.twist.linear.x = hapticDeviceVelocity(0);
		msg_velocity.twist.linear.y = hapticDeviceVelocity(1);
		msg_velocity.twist.linear.z = hapticDeviceVelocity(2);
		pub_velocity.publish(msg_velocity);


		geometry_msgs::WrenchStamped msg_force_real;
		msg_force_real.header.stamp = time_now;
		msg_force_real.header.frame_id = "/end_effector";
		msg_force_real.wrench.force.x = hapticDeviceForce_real(0);
		msg_force_real.wrench.force.y = hapticDeviceForce_real(1);
		msg_force_real.wrench.force.z = hapticDeviceForce_real(2);
		pub_force.publish(msg_force_real);





		//std::cout << cStr(freqCounterHaptics.getFrequency(), 0) << std::endl;




		ros::spinOnce();

		loop_rate.sleep();
		++count;


	}




	return 0;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------

void chatterCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{


	//ROS_INFO("I heard: [%f]", msg->wrench.force.x);

	hapticDeviceForce_desired(0) = msg->wrench.force.x;
	hapticDeviceForce_desired(1) = msg->wrench.force.y;
	hapticDeviceForce_desired(2) = msg->wrench.force.z;
}


//-----------------------------------------------------------------------------

void updateHaptics(void)
{
	// simulation in now running
	simulationRunning  = true;
	simulationFinished = false;

	// main haptic simulation loop
	while(simulationRunning)
	{
		cVector3d position;
		cVector3d linearVelocity;
		cVector3d force_real;
		cVector3d force_desired (0,0,0);

		// read position, velocity, and force
		hapticDevice->getPosition(position);
		hapticDevice->getLinearVelocity(linearVelocity);
		hapticDevice->getForce(force_real);


		// update global variable
		hapticDevicePosition   = position;
		hapticDeviceVelocity   = linearVelocity;
		hapticDeviceForce_real = force_real;


		// READ AND APPLY FORCES
		force_desired.add(hapticDeviceForce_desired);
		hapticDevice->setForce(force_desired);


		// update frequency counter
		freqCounterHaptics.signal(1);

	}

	// exit haptics thread
	simulationFinished = true;
}

//-----------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	hapticDevice->close();

	// delete resources
	delete hapticsThread;
	delete handler;

	std::cout<< "\n" << "Terminating the program safely!" << "\n" <<std::endl;
}

//------------------------------------------------------------------------------




