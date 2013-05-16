/*
 * main.cpp
 *
 *  Created on: Jan 12, 2013
 *      Author: tomaskolo
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>

#include "kinect_motor/LED.h"
#include "kinect_motor/angle.h"
#include "user.h"
#include "nullable.h"
#include "motor.h"
#include "math.h"
#include "kalmanFilter.h"
#include "audio.h"
#include "control.h"
#include "environment.h"

using namespace std;

void spinThread()
{
	ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trolley");

    boost::thread spin_thread(&spinThread);

	ros::NodeHandle nh;
	ros::Publisher ledPublisher = nh.advertise<kinect_motor::LED>("led", 10);
	ros::Publisher motor = nh.advertise<kinect_motor::angle>("angle", 10);

	cout << "Start" << endl;

	// Prostredi.
	Environment * environment = new Environment();
	// Nacteni prostredi ze souboru.
	environment->Load("configuration.xml");

	// Rizeni pohybu robota.
	Control control(environment);

    while (ros::ok()) {

    	// Pozice osoby. NITE.
    	Nullable<XnPoint3D> userPosition = User::getInstance().UserPosition();

    	// Rizeni pohybu robota.
    	control.Position(userPosition);

		if (userPosition.HasValue()) {
			// Osoba byla detekovana.

			// Rozsvit celni diodu.
			kinect_motor::LED msg;
			msg.ledColor = 2;
			ledPublisher.publish(msg);
		}
		else {

			// Zhasni celni diodu.
			kinect_motor::LED msg;
			msg.ledColor = 0;
			ledPublisher.publish(msg);
		}
    }

    cout << "Finished" << endl;

    spin_thread.join();

    delete environment;
    return 0;
}

