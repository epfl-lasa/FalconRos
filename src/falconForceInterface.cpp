#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include <mutex>




class falconSpringDamper
{
public:
	falconSpringDamper()
	{

	}

	~falconSpringDamper()
	{

	}



	void initialize()
	{

		ros::NodeHandle n;

		//Reading falcon position and velocity form the corresponding ROS topic and update the applied force
		sub_velocity = n.subscribe("/falcon/position", 10, &falconSpringDamper::updateSpring, this);
		sub_position = n.subscribe("/falcon/velocity", 10, &falconSpringDamper::updateDamper, this);

		//Topic to publish: the name of the topic is used by robot_state_publisher
		pub_force = n.advertise<geometry_msgs::WrenchStamped>("/falcon/force_desired", 1);

	}



	void updateSpring(const geometry_msgs::PoseStamped::ConstPtr& msg_pose)
	{
		force_spring.wrench.force.x = -1000 * msg_pose->pose.position.x;
		force_spring.wrench.force.y = -1000 * msg_pose->pose.position.y;
		force_spring.wrench.force.z = -1000 * msg_pose->pose.position.z;


		m.lock();

		force_total.wrench.force.x = force_spring.wrench.force.x + force_damping.wrench.force.x;
		force_total.wrench.force.y = force_spring.wrench.force.y + force_damping.wrench.force.y;
		force_total.wrench.force.z = force_spring.wrench.force.z + force_damping.wrench.force.z;





		force_total.header.stamp = ros::Time::now();

		pub_force.publish(force_total);

		m.unlock();
	}

	void updateDamper(const geometry_msgs::TwistStamped::ConstPtr& msg_vel)
	{
		force_damping.wrench.force.x = -20 * msg_vel->twist.linear.x;
		force_damping.wrench.force.y = -20 * msg_vel->twist.linear.y;
		force_damping.wrench.force.z = -20 * msg_vel->twist.linear.z;


		m.lock();

		force_total.wrench.force.x = force_spring.wrench.force.x + force_damping.wrench.force.x;
		force_total.wrench.force.y = force_spring.wrench.force.y + force_damping.wrench.force.y;
		force_total.wrench.force.z = force_spring.wrench.force.z + force_damping.wrench.force.z;

		force_total.header.stamp = ros::Time::now();

		pub_force.publish(force_total);

		m.unlock();
	}




private:

	ros::Subscriber sub_position;
	ros::Subscriber sub_velocity;
	ros::Publisher pub_force;


	geometry_msgs::WrenchStamped force_spring;
	geometry_msgs::WrenchStamped force_damping;
	geometry_msgs::WrenchStamped force_total;

	std::mutex m;



};//End of class SubscribeAndPublish


// this function closes the application
//void close(void);







// Since flag is

int main(int argc, char **argv)
{

	//setup callback when application exits
	//	atexit(close);


	//Initiate ROS
	ros::init(argc, argv, "falconSpringDamper");


	falconSpringDamper SpringDamper_object;

	SpringDamper_object.initialize();


	ros::spin();

	return 0;
}

