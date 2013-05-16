#include <ros/ros.h>
#include <iostream>

#include "libfreenect-audio.h"
#include "libfreenect.h"
#include "kinect_motor/angle.h"
#include "kinect_motor/LED.h"

freenect_device * device = 0;

void Angle(const kinect_motor::angle::ConstPtr & kinect_motor)
{
	ROS_INFO("New angle is: %f", kinect_motor->angle);

	freenect_set_tilt_degs(device, kinect_motor->angle);
}

void Led(const kinect_motor::LED::ConstPtr & kinect_led)
{
	ROS_INFO("New led color: %d", kinect_led->ledColor);

	if (kinect_led->ledColor == 1) {
		freenect_set_led(device, LED_GREEN);
	}
	else if (kinect_led->ledColor == 2) {
		freenect_set_led(device, LED_RED);
	}
	else if (kinect_led->ledColor == 3) {
		freenect_set_led(device, LED_YELLOW);
	}
	else if (kinect_led->ledColor == 4) {
		freenect_set_led(device, LED_BLINK_GREEN);
	}
	else if (kinect_led->ledColor == 5) {
		// 5 is the same as 4
		freenect_set_led(device, LED_BLINK_GREEN);
	}
	else if (kinect_led->ledColor == 6) {
		freenect_set_led(device, LED_BLINK_RED_YELLOW);
	}
	else if (kinect_led->ledColor == 0) {
		freenect_set_led(device, LED_OFF);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect_motor");

	freenect_context * context = 0;
	if (freenect_init(&context, NULL) < 0) {
		ROS_ERROR("freenect_init() failed");
		return 1;
	}

	freenect_set_log_level(context, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(context, (freenect_device_flags)(FREENECT_DEVICE_MOTOR));

	int devicesCout = freenect_num_devices (context);
	ROS_INFO("Number of devices found: %d", devicesCout);

	if (devicesCout < 1) {
		freenect_shutdown(context);
		return 1;
	}

	ROS_INFO("Connecting to device: 1");

	if (freenect_open_device(context, &device, 0) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(context);
		return 1;
	}

	freenect_set_tilt_degs(device, 0);

	ros::NodeHandle nodeHandleForAngle;
	ros::Subscriber angleSubscriber = nodeHandleForAngle.subscribe("angle", 10, Angle);

	ros::NodeHandle nodeHandleForLed;
	ros::Subscriber ledSubscriber = nodeHandleForLed.subscribe("led", 10, Led);

	ros::spin();

	freenect_close_device(device);
	freenect_shutdown(context);

	return 0;
}
