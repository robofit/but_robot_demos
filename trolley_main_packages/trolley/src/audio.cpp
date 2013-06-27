/*
 * audio.cpp
 *
 *  Created on: Apr 7, 2013
 *      Author: tomaskolo
 */

#include "audio.h"
#include <iostream>
#include <algorithm>
#include <exception>

#include "utilities.h"

using namespace std;

void Audio::AudioDataCallBack(const kinect_audio::audio::ConstPtr & audioData)
{
    ////////////////////////////// KALMAN FILTR //////////////////////////////

    // Filtrovana data pro kanal jedna.
    vector<double> filtr1 ;
    // Filtrovana data pro kanal dva.
    vector<double> filtr2 ;

    // Pres vsechny navzorkovane data (audioData = microphone1, audioData2 = microphone3).
    for (kinect_audio::audio::_mic1_type::const_iterator itData = audioData->mic1.begin(), itData2 = audioData->mic3.begin();
    		itData != audioData->mic1.end() && itData2 != audioData->mic2.end(); ++itData, ++itData2) {

    	// Predikce.
    	_kalman.Predict();

    	// Vstupni matice namerenych dat.
    	double input[2][1] = {
    				{*itData},
    				{*itData2}};

    	// Korekce.
    	_kalman.Correct(Math::Matrix<2,1>(input));

    	// Vystup.
    	Math::Matrix<4,1> output = _kalman.GetX();

    	// Ulozeni filtrovanych dat do vektoru.
    	filtr1.push_back(output.at(0,0));
    	filtr2.push_back(output.at(1,0));
    }

    //////////////////////////// UPRAVA AMPLITUDY ////////////////////////////

    // Kazdy signal z jednotlivych kanalu je nejdrive potreba upravit na rozsah o stejnem aplitude.
    // Tim se zajisti, ze nebude dochazet k nepresnostem pri vypoctu posuvu mezi jednotlivymi fazemi.


    // Vrati maximalni hodnotu z vektoru.
	double maxFiltr1 = *max_element(filtr1.begin(), filtr1.end());
	// Vrati minimalni hodnotu z vektoru.
	double minFiltr1 = *min_element(filtr1.begin(), filtr1.end());
	// Vrati maximalni odchylku.
	double absMaxFilter1 = max(fabs(maxFiltr1), fabs(minFiltr1));
	// Upravy signal na jednotkovou velikost amplitudy.
	transform(filtr1.begin(), filtr1.end(), filtr1.begin(), bind1st(multiplies<double>(), 1.0/absMaxFilter1));

	// Vrati maximalni hodnotu z vektoru.
	double maxFiltr2 = *max_element(filtr2.begin(), filtr2.end());
	// Vrati minimalni hodnotu z vektoru.
	double minFiltr2 = *min_element(filtr2.begin(), filtr2.end());
	// Vrati maximalni odchylku.
	double absMaxFilter2 = max(fabs(maxFiltr2), fabs(minFiltr2));
	// Upravy signal na jednotkovou velikost amplitudy.
	transform(filtr2.begin(), filtr2.end(), filtr2.begin(), bind1st(multiplies<double>(), 1.0/absMaxFilter2));

	////////////////////// VYPOCET POSUVU MEZI SIGNALY //////////////////////


	// Priznak prvniho pruchodu cyklem.
    bool firstTime = true;
    // Minimalni suma rozdilu signalu.
	double minSum = 0;
	// Vypocitany posuv mezi signaly.
	unsigned int shiftSize = 0;

	// Pres vsechny posuvy.
	for (unsigned int shift = 0; shift < (2 * _samplesDistance); ++shift) {

		// Suma rozdilu signalu.
		double sum = 0.0;
		vector<double>::const_iterator itShift = filtr2.begin() + shift;
		for (vector<double>::const_iterator itBase = filtr1.begin() + _samplesDistance, endIt = filtr1.end() - _samplesDistance; itBase != endIt; ++itBase, ++itShift) {

			// Referencni signal.
			const double & valueMic1 = *itBase;
			// Posouvany signal.
			const double & valueMic2 = *itShift;
			// Suma rozdilu signalu.
			sum = sum + fabs(valueMic1 - valueMic2);
		}

		if (firstTime) {
			// Prvni pruchod.
			minSum = sum;
			firstTime = false;
		}
		else if (sum < minSum) {
			// Nalezena presnejsi hodnota posuvu.
			minSum = sum;
			shiftSize = shift;
		}
	}

	// Pridej hodnotu do statistik.
	_shiftSizeStatistic.addValue(shiftSize);

	// Rozptyl.
	if (_shiftSizeStatistic.Variance() < 0.5) {
		// Rozptyl je pod nastavenou hranici.

		// Detekovany uhel zdroje zvuku.
		_angle = _shiftSizeStatistic.ArithmeticMean();
	}
}

Audio::Audio(const std::string & msgType) : _kalman(1.0, 3, 5000), _angle(8.0)
{
	// Stavovy vektor pro kalmanuv filtr.
	double Xvalue[4][1] = {
			{0},
			{0},
			{0},
			{0}};

	_kalman.SetX(Math::Matrix<4,1>(Xvalue));

	// Subscriber pro prijem audio dat.
	_audioSubscriber = ros::NodeHandle().subscribe(msgType, 10, &Audio::AudioDataCallBack, this);
}

Nullable<double> Audio::Angle()
{
	if (_angle.HasValue()) {

		// Lock - dodelat
		Nullable<double> angle(_angle);
		_angle.SetNull();

		if(angle > ((_samplesDistance * 2) - 1) || angle < 0) {
			// Interni chyba. Nemuze nastat.
			std::runtime_error("Bad samples distance.");
		}

		if (Utilities::compare_two_doubles(angle, 8.0) || Utilities::compare_two_doubles((angle - 1), 7)) {
			// Cil je na ose snimace.
			return 0.0;
		}
		else {
			// Prevede posuv ve vzorcich na stupne.
			return (-180.0 / ((_samplesDistance * 2.0) - 1.0) * angle) + 90.0;
		}
	}
	else {
		// Uhel zdroje nebyl detekovan.
		return Nullable<double>();
	}
}

