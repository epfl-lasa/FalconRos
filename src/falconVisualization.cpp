#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include <sstream>
#include <iostream>
#include <stdio.h>



#include "chai3d.h"
#include "GLFW/glfw3.h"

using namespace chai3d;
using namespace std;



//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a handle to window display context with initial width and height
GLFWwindow* window = NULL;
int width  = 0;
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small sphere (cursor) representing the haptic device
cShapeSphere* cursor;

// a line representing the velocity of the haptic device
cShapeLine* velocity;

// a font for rendering text
cFontPtr font;

// a label to display the position [m] and force [N] of the haptic device and the rate [Hz] at which the simulation is running
cLabel* labelHapticDevicePosition;
cLabel* labelHapticDeviceForce;
cLabel* labelRates;

// a scope to monitor position and force values of haptic device
cScope* scope_position;
cScope* scope_force;

// a level widget to display velocity and force
cLevel* levelVelocity;
cLevel* levelForce;

// three dials to display position information
cDial* dialPowX;
cDial* dialPowY;
cDial* dialPowZ;

// a global variable to store the position [m], velocity [m/s], and force [N?] of the haptic device
cVector3d hapticDevicePosition;
cVector3d hapticDeviceVelocity;
cVector3d hapticDeviceForce;
cVector3d hapticDevicePower;




//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// this function renders the scene
void updateGraphics(void);

// ROS callbacks to update position, velocity, and force
void UpdatePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void UpdateVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void UpdateForceCallback(const    geometry_msgs::WrenchStamped::ConstPtr& msg);



//------------------------------------------------------------------------------
// MAIN FUNCTION
//------------------------------------------------------------------------------

int main(int argc, char **argv)
{

	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.9 * mode->height;
	int h = 0.6 * mode->height;
	int x = 0.6 * (mode->width  - w);
	int y = 0.6 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height); 

	// set position of window
	glfwSetWindowPos(window, x, y);

    	// set resize callback
    	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);



	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
			cVector3d(0.0, 0.0, 0.0),    // look at position (target)
			cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

	// set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// create a directional light source
	light = new cDirectionalLight(world);

	// insert light source inside world
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// define direction of light beam
	light->setDir(-1.0, 0.0, 0.0);

	// create a sphere (cursor) to represent the haptic device
	cursor = new cShapeSphere(0.01);

	// insert cursor inside world
	world->addChild(cursor);

	// create small line to illustrate the velocity of the haptic device
	velocity = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

	// insert line inside world
	world->addChild(velocity);



    	//--------------------------------------------------------------------------
    	// WIDGETS
    	//--------------------------------------------------------------------------

    	// create a font
    	font = NEW_CFONTCALIBRI20();


    	// create a label to display the position of haptic device
    	labelHapticDevicePosition = new cLabel(font);
    	camera->m_frontLayer->addChild(labelHapticDevicePosition);

    	// create a label to display the force of haptic device
    	labelHapticDeviceForce = new cLabel(font);
    	camera->m_frontLayer->addChild(labelHapticDeviceForce);

    	// create a label to display graphic rate of the simulation
    	labelRates = new cLabel(font);
    	camera->m_frontLayer->addChild(labelRates);

    	// create a scope to plot haptic device position data
    	scope_position = new cScope();
    	camera->m_frontLayer->addChild(scope_position);
    	scope_position->setLocalPos(100,60);
    	scope_position->setRange(-0.1, 0.1);
    	scope_position->setSignalEnabled(true, true, true, false);
    	scope_position->setTransparencyLevel(0.7);

    	// create a scope to plot haptic device force data
    	scope_force = new cScope();
    	camera->m_frontLayer->addChild(scope_force);
    	scope_force->setLocalPos(100,190);
    	scope_force->setRange(-10, 10);
    	scope_force->setSignalEnabled(true, true, true, false);
    	scope_force->setTransparencyLevel(0.7);

    	// create a level to display velocity data
    	levelVelocity = new cLevel();
    	camera->m_frontLayer->addChild(levelVelocity);
    	levelVelocity->setLocalPos(20, 60);
    	levelVelocity->setRange(0.0, 1.0);
    	levelVelocity->setWidth(60);
    	levelVelocity->setNumIncrements(30);
    	levelVelocity->setSingleIncrementDisplay(false);
    	levelVelocity->setTransparencyLevel(0.5);

    	// create a level to display force data
    	levelForce = new cLevel();
    	camera->m_frontLayer->addChild(levelForce);
    	levelForce->setLocalPos(20, 190);
   	levelForce->setRange(0.0, 100);
    	levelForce->setWidth(60);
    	levelForce->setNumIncrements(30);
    	levelForce->setSingleIncrementDisplay(false);
    	levelForce->setTransparencyLevel(0.5);

    	// three dials to display position data
    	dialPowX = new cDial();
    	camera->m_frontLayer->addChild(dialPowX);
    	dialPowX->setLocalPos(890, 270);
    	dialPowX->setRange(-10, 10);
    	dialPowX->setSize(60);
    	dialPowX->setSingleIncrementDisplay(true);

    	dialPowY = new cDial();
    	camera->m_frontLayer->addChild(dialPowY);
    	dialPowY->setLocalPos(890, 185);
    	dialPowY->setRange(-10, 10);
    	dialPowY->setSize(60);
    	dialPowY->setSingleIncrementDisplay(true);

    	dialPowZ = new cDial();
    	camera->m_frontLayer->addChild(dialPowZ);
    	dialPowZ->setLocalPos(890, 100);
    	dialPowZ->setRange(-10, 10);
    	dialPowZ->setSize(60);
    	dialPowZ->setSingleIncrementDisplay(true);


	//--------------------------------------------------------------------------
	// Initialize the ROS topic
	//--------------------------------------------------------------------------

  ros::init(argc, argv, "falconVis");
  ros::NodeHandle n;

  ros::Subscriber sub_position = n.subscribe("falcon/position"   , 1000, UpdatePositionCallback);
  ros::Subscriber sub_velocity = n.subscribe("falcon/velocity"   , 1000, UpdateVelocityCallback);
  ros::Subscriber sub_force    = n.subscribe("falcon/force_real" , 1000, UpdateForceCallback);

  ros::Rate loop_rate(1000);

        //--------------------------------------------------------------------------
    	// MAIN  LOOP
    	//--------------------------------------------------------------------------
	
	windowSizeCallback(window, width, height);

	while (!glfwWindowShouldClose(window))
  	{

        	// get width and height of window
        	glfwGetWindowSize(window, &width, &height);

        	// render graphics
        	updateGraphics();

        	// swap buffers
        	glfwSwapBuffers(window);

        	// process events
        	glfwPollEvents();

        	// signal frequency counter
        	freqCounterGraphics.signal(1);

		    ros::spinOnce();

    		loop_rate.sleep();

		//cout << "All good!" << endl;
	}


	return 0;

}


//-----------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//-----------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

    // update position of label
    //labelHapticDevicePosition->setLocalPos(20, width - 60, 0);
    //labelHapticDeviceForce->setLocalPos(20, width - 60, 0);


    // update position of label
   // labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of scope
    scope_position->setSize(width - 200, 120);
    scope_force->setSize(width - 200, 120);


    // update position of dials
    dialPowX->setLocalPos(width - 50, 270);
    dialPowY->setLocalPos(width - 50, 185);
    dialPowZ->setLocalPos(width - 50, 100);
}

//-----------------------------------------------------------------------------


void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position and force data
    //labelHapticDevicePosition->setText( hapticDevicePosition.str(3));
    //labelHapticDeviceForce->setText( hapticDeviceForce.str(3));

    // update haptic and graphic rate data
    //labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
    //                    cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    //labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    // update arrow
    velocity->m_pointA = hapticDevicePosition;
    velocity->m_pointB = cAdd(hapticDevicePosition, hapticDeviceVelocity);

    // update position of cursor
    cursor->setLocalPos(hapticDevicePosition);


    // update information to scope
    scope_position->setSignalValues(hapticDevicePosition.x(),
                                    hapticDevicePosition.y(),
                                    hapticDevicePosition.z());

    // update information to scope
    scope_force->setSignalValues(hapticDeviceForce.x(),
                                 hapticDeviceForce.y(),
                                 hapticDeviceForce.z());

        hapticDevicePower(0) = hapticDeviceVelocity(0) * hapticDeviceForce(0);
        hapticDevicePower(1) = hapticDeviceVelocity(1) * hapticDeviceForce(1);
        hapticDevicePower(2) = hapticDeviceVelocity(2) * hapticDeviceForce(2);


    // update information to dials
    dialPowX->setValue(hapticDevicePower.x());
    dialPowY->setValue(hapticDevicePower.y());
    dialPowZ->setValue(hapticDevicePower.z());

    // update velocity information to level
    levelVelocity->setValue(hapticDeviceVelocity.length());
    levelForce->setValue(hapticDeviceForce.length());


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////


    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//-----------------------------------------------------------------------------

void UpdatePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{


  //ROS_INFO("I heard: [%f]", msg->wrench.force.x);

	hapticDevicePosition(0) = msg->pose.position.x;
	hapticDevicePosition(1) = msg->pose.position.y;
	hapticDevicePosition(2) = msg->pose.position.z;
}

//-----------------------------------------------------------------------------

void UpdateVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{


  //ROS_INFO("I heard: [%f]", msg->wrench.force.x);

	hapticDeviceVelocity(0) = msg->twist.linear.x;
	hapticDeviceVelocity(1) = msg->twist.linear.y;
	hapticDeviceVelocity(2) = msg->twist.linear.z;
}

//-----------------------------------------------------------------------------

void UpdateForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{


  //ROS_INFO("I heard: [%f]", msg->wrench.force.x);

	hapticDeviceForce(0) = msg->wrench.force.x;
	hapticDeviceForce(1) = msg->wrench.force.y;
	hapticDeviceForce(2) = msg->wrench.force.z;
}


