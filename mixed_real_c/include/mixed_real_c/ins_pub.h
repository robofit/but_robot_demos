/*********************************************************************************
 *      Nazov suboru:   Hlavickovy subor aplikacie pre vkladanie virtualnych
 *                      objektov do dat robota
 *      Balicek:        mixed_real_c
 *      Subor:          ins_pub.h
 *      Datum:          12.1.2013
 *      Posledna zmena: 11.5.2013
 *      Autor:          Jakub Fisla xfisla00@stud.fit.vutbr.cz
 *
 *      Popis:          Hlavickovy subor, vkladanie virtualnych objektov do dat robota
                        pod Robot Operating System
 ********************************************************************************/
/**
 *      @file ins_pub.h
 *      
 *      @brief Hlavickovy subor aplikacie na vkladanie virtualnych objektov do dat robota
 *      @author Jakub Fisla (xfisla00)
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"

#define doNotCross 1

#define LEN_FRONT_VECT 2
#define PI 3.14159265359
#define DIS_STRENGTH 0.008

double UNIT_BTW_LAS = (PI/360.0);	//uhol medzi laserovymi lucmi
double MAX_ANGLE = 2*PI/0.75;		//rozpetie snimania laseru
double las_ang_max =  PI*0.75;		//ruzpetie vlavo od laseru
double las_ang_min = -PI*0.75;		//rozpetie vpravo od laseru

/**
 * Struktura obsahujuca data virtualneho objektu.
 * Obsahuje meno (name), x suradnice, y suradnice, a intenzitu nasnimaneho signalu
 */
typedef struct preset{
	std::string name;
	std::vector<double> x;
	std::vector<double> y;
	double intensity;
}Preset;


/**
 * Trieda popisujuca virtualny objekt, obsahue zoznam bodov objektu, intenzitu jeho nasnimaneho signalu, popripade tazisko
 */
class virtualObject{
	private:
		tf::Vector3 cent;
	public:
		std::vector<tf::Vector3> points;
		double intensity;

		virtualObject(float x, float y, float z, float side);	//recangle
		virtualObject();	//polygon
		void polygonObject(Preset pr);
		tf::Vector3 getCent();
};

/**
 * Treda zastresuje hlavnu funkcnost aplikacie, nacita virtualne objekty, vypocita ich polohua  upravi data
 */
class insertObj{
	private:
		std::vector<virtualObject> vObjects;
		tf::Vector3 front;
		tf::Vector3 laserPos;
		tf::StampedTransform transform;

		tf::Vector3 getPosition(tf::TransformListener *listener);
		void setFrontVector();
		double setObjAngle(tf::Vector3 base, tf::Vector3 point);
		tf::Vector3 getLaserVector(int laserNo, float len);
		tf::Vector3 cross(tf::Vector3 o1, tf::Vector3 o2, tf::Vector3 l1, tf::Vector3 l2);
	public:
		void addObj(float x, float y, float z, float side);
		void addPolygon(Preset pr);
		void loadObjects();
		void startIns(sensor_msgs::LaserScan * sc_msg);
};

//Global variables
insertObj * pIObj;		//ukozovatel na objekt hlavnej triedy
sensor_msgs::LaserScan mixed_scan;	//Novo vytvorena sprava
double noise = DIS_STRENGTH;	//uroven sumu
int test_info = 0;		//uroven vystupnych vypisov

/*** Koniec suboru ins_pub.h ***/
