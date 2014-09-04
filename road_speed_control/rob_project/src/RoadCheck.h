/**
 * Trida pro analizu kvality cesty
 */

#ifndef RCH_H
#define	RCH_H
#include<iostream>
#include<string>
#include<cstdlib>
#include <cstdio>
#include <complex>
#include <valarray>
#include "Filter.h"


using namespace std;

 
namespace roadcheck {
		
	class RoadCheck {
	public:
		/**
		 * Konstruktor
		 */
		RoadCheck();
		/**
		 * Konstruktor
		 * @param frame_length_angle delka okna pro vypocet uhlu
		 * @param g odecitana hodnota 
		 * @param gth prah urcujici ktere pretizeni uz vadi 
		 * @param frame_length_road delka okna pro analyzu cesty
		 */
		RoadCheck(int frame_length_angle, float g, float gth, int frame_length_road); // nastaveni delkz okna pro uhel, nasobek v G , prah pretizeni, delka okna pro analizu cesty
		~RoadCheck();
		
		/**
		 * Pridani dat akcelerometu do filtru
		 * @param x osa x
		 * @param y osa y
		 * @param z osa z
		 * @return koeficient kvality cesty
		 */
		float addSample(float x, float y, float z); // pridani dat z akcelerometru do filtru, x,y,z jednotlive osy
		void clearAll();
		bool countAngles(); //spocitat uhli
		float getAngleX(); //ziskani jednotlivych naklonu
		float getAngleY();
		float getAngleZ();
		float getV();

		/**
		 * Vypocet medianu tri hodnot
		 */
		float median(float a, float b, float c); //medianovy filtr o delce 3

	private:
		float filter(float input);

		float g;
		float gth;

		float *x_frame;
		float *y_frame;
		float *z_frame;

		float *v_frame;
		float *filter_d;

		float angle_x;
		float angle_y;
		float angle_z;

		float v;

		int end;
		int length;
		int count;
		bool frame_full;

		Filter *timing;
		Filter *mean;
		float *b_timing;
		float *b_mean;
		float *med;
		int med_i;
	};

}

#endif

