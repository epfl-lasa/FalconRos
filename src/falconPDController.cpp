#include <ros/ros.h>

#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include <mutex>

#include <vector>


class falconPDController
{
	private:
	ros::NodeHandle n;

	// -------------------------------------------------------------
	// Declaring subscribers and publisher

	// reading from falcon
	ros::Subscriber sub_positionReal;
	ros::Subscriber sub_velocityReal;
	ros::Subscriber pub_forceReal;

	// reading from the top-level controller or motion generator
	ros::Subscriber sub_positionDesired;
	ros::Subscriber sub_velocityDesired;

	ros::Subscriber sub_Pgain;
	ros::Subscriber sub_Dgain;

	// commanding the falcon
	ros::Publisher pub_forceDesired;

	// notifying interested nodes
	ros::Publisher pub_forceExternal;

	// -------------------------------------------------------------
	// Declaring variables

	// Keep the falcon states
	std::vector<float> positionReal;
	std::vector<float> velocityReal;
	std::vector<float> forceReal;


	// Keep the desired motion
	std::vector<float> positionDesired;
	std::vector<float> velocityDesired;
	std::vector<float> forceDesired;
	std::vector<float> forceDesiredSmooth;


	// keep the gains
	float Pgain;
	float Dgain;

	std::vector<float> positionError;
	std::vector<float> velocityError;

	// keep the desired force
	//std::vector<float> forceDesired;
	geometry_msgs::WrenchStamped msg_forceDesired;


	// keep the external forces
	std::vector<float> forceExternal;

	// a lock for when more than one function wants to update the desired force
	std::mutex m;



public:
	falconPDController()
{

		//Reading falcon position, velocity, and force form the corresponding ROS topic and updating the corresponding  variables
		sub_positionReal = n.subscribe("/falcon/position"  , 10, &falconPDController::UpdatePositionReal, this);
		sub_velocityReal = n.subscribe("/falcon/velocity"  , 10, &falconPDController::UpdateVelocityReal, this);
		pub_forceReal    = n.subscribe("/falcon/force_real", 10, &falconPDController::UpdateForceReal   , this);

		//Reading the desired position and the desired velocity form the corresponding ROS topic and updating the corresponding variables
		sub_positionDesired = n.subscribe("/PDController/position_desired", 10, &falconPDController::UpdatePositionDesired, this);
		sub_velocityDesired = n.subscribe("/PDController/velocity_desired", 10, &falconPDController::UpdateVelocityDesired, this);

		//Reading Controller parameters P-gain and D-gain form the corresponding ROS topic and updating the corresponding variables
		sub_Pgain = n.subscribe("/PDController/Pgain", 10, &falconPDController::UpdatePgain, this);
		sub_Dgain = n.subscribe("/PDController/Dgain", 10, &falconPDController::UpdateDgain, this);

		//Topic to publish: the name of the topic is used by robot_state_publisher
		pub_forceDesired = n.advertise<geometry_msgs::WrenchStamped>("/falcon/force_desired", 1);

		//Topic to publish the estimated external force
		pub_forceExternal = n.advertise<geometry_msgs::WrenchStamped>("/PDController/force_external", 1);

		// setting up the vectors
		positionReal.resize(3);
		velocityReal.resize(3);
		forceReal.resize(3);
		positionDesired.resize(3);
		velocityDesired.resize(3);
		//forceDesired.resize(3);
		forceExternal.resize(3);
		positionError.resize(3);
		velocityError.resize(3);
		forceDesired.resize(3);
		forceDesiredSmooth.resize(3);

		// setting up the gains
		Pgain = 0;
		Dgain = 30;
}


	void UpdatePositionReal(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		positionReal[0] = msg->pose.position.x;
		positionReal[1] = msg->pose.position.y;
		positionReal[2] = msg->pose.position.z;
		UpdateForceDesired();
	}

	void UpdateVelocityReal(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		velocityReal[0] = msg->twist.linear.x;
		velocityReal[1] = msg->twist.linear.y;
		velocityReal[2] = msg->twist.linear.z;
		UpdateForceDesired();
	}

	void UpdateForceReal(const geometry_msgs::WrenchStamped::ConstPtr& msg)
	{
		forceReal[0] = msg->wrench.force.x;
		forceReal[1] = msg->wrench.force.y;
		forceReal[2] = msg->wrench.force.z;
		UpdateForceDesired();
	}

	void UpdatePositionDesired(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		positionDesired[0] = msg->pose.position.x;
		positionDesired[1] = msg->pose.position.y;
		positionDesired[2] = msg->pose.position.z;
		UpdateForceDesired();
	}

	void UpdateVelocityDesired(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		velocityDesired[0] = msg->twist.linear.x;
		velocityDesired[1] = msg->twist.linear.y;
		velocityDesired[2] = msg->twist.linear.z;
		UpdateForceDesired();
	}

	void UpdatePgain(const std_msgs::Float64::ConstPtr& msg)
	{
		Pgain = msg->data;
		std::cout << "received a new Pgain: " << Pgain << std::endl;
		UpdateForceDesired();
	}

	void UpdateDgain(const std_msgs::Float64::ConstPtr& msg)
	{
		Dgain = msg->data;
		std::cout << "received a new Dgain: " << Dgain << std::endl;
		UpdateForceDesired();
	}

	void filterForce()
	{
		// 0 ≤ alpha ≤ 1 ; a smaller value basically means more smoothing
	    for ( int i = 0; i < 3 ; i++ )
	    {
	    	forceDesiredSmooth[i] = forceDesiredSmooth[i] + 0.9 * (forceDesired[i] - forceDesiredSmooth[i]);

	    	if (forceDesiredSmooth[i] > 15)
	    		forceDesiredSmooth[i] = 15;

	    	if (forceDesiredSmooth[i] < -15)
	    		forceDesiredSmooth[i] = -15;
	    }

	}

	void UpdateForceDesired()
	{
		m.lock();

		positionError[0] = positionReal[0] - positionDesired[0];
		positionError[1] = positionReal[1] - positionDesired[1];
		positionError[2] = positionReal[2] - positionDesired[2];

		velocityError[0] = velocityReal[0] - velocityDesired[0];
		velocityError[1] = velocityReal[1] - velocityDesired[1];
		velocityError[2] = velocityReal[2] - velocityDesired[2];

		for(int i = 0 ; i < 3 ; i++ )
		{
//			if(positionError[i] < 0.005 && positionError[i] > -0.005)
//				positionError[i] = 0;
//
//			if(velocityError[i] < 0.01  && velocityError[i] > -0.01 )
//				velocityError[i] = 0;

			if(velocityError[i] > 0.4)
				velocityError[i] = 0.4;

			if(velocityError[i] < -0.4)
				velocityError[i] = -0.4;

		}

		//std::cout << velocityReal[0] << "\t" << velocityDesired[0] << "\t" << velocityError[0] << std::endl;

		forceDesired[0] = -1 * Pgain * positionError[0] - 1 * Dgain * velocityError[0];
		forceDesired[1] = -1 * Pgain * positionError[1] - 1 * Dgain * velocityError[1];
		forceDesired[2] = -1 * Pgain * positionError[2] - 1 * Dgain * velocityError[2];

		filterForce();

		msg_forceDesired.header.stamp = ros::Time::now();
		msg_forceDesired.wrench.force.x = forceDesiredSmooth[0];
		msg_forceDesired.wrench.force.y = forceDesiredSmooth[1];
		msg_forceDesired.wrench.force.z = forceDesiredSmooth[2];


		pub_forceDesired.publish(msg_forceDesired);

		//std::cout << "V_rx: " << velocityReal[0] << "  V_dx: " << velocityDesired[0] << " Dgain : " << Dgain  << " F: "  << forceDesired.wrench.force.x  << std::endl;


		m.unlock();
	}


};//End of class falconPDController

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "PDController");

	//Create an object of class JointMapper to read the robot joint angles and send it to robot_state_publisher and rviz
	falconPDController pdController_object;

	ros::spin();

	return 0;
}
