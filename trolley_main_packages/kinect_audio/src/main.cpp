#include <ros/ros.h>
#include <libfreenect.h>
#include <libfreenect-audio.h>
#include <signal.h>
#include "kinect_audio/audio.h"

ros::Publisher audioPublisher;
bool die = false;

void callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown)
{
	if (audioPublisher.getNumSubscribers() > 0) {

		kinect_audio::audio audio;

		for (unsigned int i = 0; i < num_samples; ++i) {

			audio.mic1.push_back(mic1[i]);
			audio.mic2.push_back(mic2[i]);
			audio.mic3.push_back(mic3[i]);
			audio.mic4.push_back(mic4[i]);
		}

		audioPublisher.publish(audio);
	}
}

void cleanup(int sig) {
	die = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_audio");

	freenect_context * contex;
	if (freenect_init(&contex, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}
	freenect_set_log_level(contex, FREENECT_LOG_SPEW);
	freenect_select_subdevices(contex, FREENECT_DEVICE_AUDIO);

	int nr_devices = freenect_num_devices(contex);
	printf ("Number of devices found: %d\n", nr_devices);
	if (nr_devices < 1) {
		freenect_shutdown(contex);
		return 1;
	}

	freenect_device * device;
	int deviceNumber = 0;
	if (freenect_open_device(contex, &device, deviceNumber) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(contex);
		return 1;
	}

	freenect_set_audio_in_callback(device, callback);
	freenect_start_audio(device);
	signal(SIGINT, cleanup);

	audioPublisher = ros::NodeHandle().advertise<kinect_audio::audio>("audio", 10);

	while(!die && freenect_process_events(contex) >= 0) {
		// If we did anything else, it might go here.
		// Alternately, we might split off another thread
		// to do this loop while the main thread did something
		// interesting.
	}

	freenect_shutdown(contex);
	return 0;
}
