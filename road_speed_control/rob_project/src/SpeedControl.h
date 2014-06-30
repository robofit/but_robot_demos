
/**
* Trida pro kontrolu maximalni rychlosti robora.
*/


#ifndef SC_H
#define	SC_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <math.h> 
#include "RoadCheck.h"
#include <string> 
#include <sstream>


namespace roadcheck {

	//nastaveni maximalni rychlosti
	#define SPEED_SET_TEXT "rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS max_vel_x "
	#define ACCELERATE_SET_TEXT "rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS acc_lim_x "

	//parametrz v parametrovem serveru
	#define PARAM_LENGTH_PATH "/rob_project/frame_length_angle"
	#define PARAM_LENGTH_ROAD_PATH "/rob_project/frame_length_road"
	#define PARAM_G_PATH "/rob_project/g"
	#define PARAM_G_TH_PATH "/rob_project/gth"

	#define PARAM_MAX_SPEED_ROAD_PATH "/rob_project/max_speed_road"
	#define PARAM_MAX_ACC "/rob_project/max_acc"

	#define PARAM_A_SPEED_ROAD_PATH "/rob_project/a_speed_road"
	#define PARAM_A_SPEED_AX_PATH "/rob_project/a_speed_ax"
	#define PARAM_A_SPEED_AY_PATH "/rob_project/a_speed_ay"
	#define PARAM_A_ACC "/rob_project/a_acc"

	#define PARAM_AUTOMAT "/rob_project/auto"

	//defaultni parametry
	#define FRAME_LENGTH_MAX 600
	#define FRAME_LENGTH_DEF 60
	#define FRAME_LENGTH_ROAD_DEF 40
	#define G 1
	#define MAX_SPEED_DEF 0.5

	#define CMD_PUB "/cmd_vel"

	#define PI 3.1415


	using namespace std;

	class SpeedControl {
	public:
		//Konstruktor
		SpeedControl(ros::NodeHandle n); 

		~SpeedControl() {
			if (rc != NULL)delete rc;
		};
		/**
		 * Vypocet a nastaveni maximalni rychlosti
		 * @param road_coef koeficient cesty
		 * @param anglex naklon v ose x
		 * @param angley naklon v ose y
		 */
		void setMaxSpeed(float road_coef, float anglex, float angley); //nastaveni maximalni rychlosti robota, koeficient cesty, uhel naklonu x a y
		/**
		 * Volani pri nacitani dat z akcelerometru
		 * @param msg
		 */
		void callback(const sensor_msgs::ImuConstPtr &msg); //volani na odchytavani zprav ros
		/**
		 * Volani pri nacitani dat o zmene rychlosti
		 * @param msg
		 */
		void callbackManual(const geometry_msgs::TwistConstPtr &msg);
		/**
		 * Funkce pro ziskani minimalni hodnoty z parametru
		 * @param s1
		 * @param s2
		 * @param s3
		 * @return 
		 */
		double min(double s1, double s2, double s3); // vyber minima
	private:
		RoadCheck *rc;
		ros::Publisher vel_pub;
	   
		double roadc;  //koeficient kvality cesty
		double anglex; //uhel nakronu 
		double angley;
		
		double automat; // priznak autonomni jizdy
		
		double max_speed_road; //uprava zrychleni
		double max_acc;
		
		double a_speed_road; //koeficienty funkci
		double a_speed_ax;
		double a_speed_ay;
		double a_acc;
	};

}
#endif
