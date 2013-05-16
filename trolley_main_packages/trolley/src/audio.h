/*
 * audio.h
 *
 *  Created on: Apr 7, 2013
 *      Author: tomaskolo
 */

#ifndef AUDIO_H_
#define AUDIO_H_

#include <string>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <kinect_audio/audio.h>
#include "nullable.h"
#include "kalmanFilter.h"
#include "statistics.h"

class Audio {
public:
	/**
	 *  Konstruktor.
	 */
	Audio(const std::string & msgType = "audio");

	/**
	 *  Vrati uhel prichodu zdroje zvuku.
	 *
	 *  @return Varti null pokud zdroj nebyl detekovan.
	 */
	Nullable<double> Angle();

private:

	/**
	 * CallBack pro audio data.
	 */
	void AudioDataCallBack(const kinect_audio::audio::ConstPtr & audioData);

	/// Subcriber.
	ros::Subscriber _audioSubscriber;

	// Pro vypocet rozptylu.
	Statistics<double, 10> _shiftSizeStatistic;

	// Kalamanuv filtr.
	Math::KalmanFilter _kalman;

	// Uhel zdroje.
	Nullable<double> _angle;

	// Maximalni mozny posuv mezi signaly.
	// Zavisle na vzdalenosti mezi mikrofony.
	static const unsigned int _samplesDistance = 8;
};

#endif /* AUDIO_H_ */
