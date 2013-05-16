/*
 * environment.cpp
 *
 *  Created on: Apr 16, 2013
 *      Author: tomaskolo
 */

#include "environment.h"

#include <string.h>
#include <ros/ros.h>
#include <tinyxml/tinyxml.h>

using namespace std;

Environment::Environment()
	: _linearPID(2.0, -2.0, 0.0005, 0.00013, 0.0, 50), _angularPID(2.0, -2.0, 0.0021, 0.0011, 0.0, 35), _strategy(Strategy::FIRST)
{
}

bool Environment::Load(const char * fileName)
{
	/// Nacteni xml do stromu.
    TiXmlDocument doc(fileName);

    if(doc.LoadFile()) {

    	/// Root koren.
    	TiXmlElement * root=doc.FirstChildElement();

    	/// Uzel Control.
    	TiXmlElement * element = root->FirstChild( "Control" )->FirstChildElement();

    	while (element) {

			const string & key = element->ValueStr();
			string text;
			if (element && element->GetText()) {
				text = element->GetText();
			}

			if (key.empty() || text.empty()) {

				element = element->NextSiblingElement();
				ROS_WARN_STREAM("Error in the configuration.");

				continue;
			}

    		if (key == "PID") {

    			double p = 0.0;
    			if (element->QueryDoubleAttribute("p",&p) != TIXML_SUCCESS) {
    				ROS_WARN_STREAM("Wrong PID in the configuration.");
    			}
    			double i = 0.0;
    			if (element->QueryDoubleAttribute("i",&i) != TIXML_SUCCESS) {
    				ROS_WARN_STREAM("Wrong PID in the configuration.");
    			}
    			double d = 0.0;
    			if (element->QueryDoubleAttribute("d",&d) != TIXML_SUCCESS) {
    				ROS_WARN_STREAM("Wrong PID in the configuration.");
    			}

    			//std::cout << p << " " << i << " "<< d <<   std::endl;
    		}
    		else if (key == "Distance") {

    			double distanceValue = 0.0;

    			try {

    				distanceValue = boost::lexical_cast<int>(text);
    			} catch (boost::bad_lexical_cast & e) {
    				ROS_WARN_STREAM("Distance value in the configuration is bad.");
				}

    			//cout << distanceValue << endl;
    		}
    		else {

    			ROS_WARN_STREAM("Unknown element in the configuration.");
    		}



    		element = element->NextSiblingElement();
    	}

    	// Uzel motor.
    	element = root->FirstChild("Motor")->FirstChildElement();

    	while (element) {

			const string & key = element->ValueStr();
			string text;
			if (element && element->GetText()) {
				text = element->GetText();
			}

			if (key.empty() || text.empty()) {

				element = element->NextSiblingElement();
				ROS_WARN_STREAM("Error in the configuration.");

				continue;
			}

    		if (key == "MaxSpeed") {

    			double value = 0.0;
    			if (element->QueryDoubleAttribute("value",&value) != TIXML_SUCCESS) {
    				ROS_WARN_STREAM("Wrong PID in the configuration.");
    			}

    			//std::cout << value << std::endl;
    		}
    		else if (key == "MaxAcceleration") {

    			double value = 0.0;
    			if (element->QueryDoubleAttribute("value",&value) != TIXML_SUCCESS) {
    				ROS_WARN_STREAM("Wrong PID in the configuration.");
    			}

    			//std::cout << value << std::endl;
    		}
    		else if (key == "MotorTopic") {

    			//std::cout << text << std::endl;
    		}
    		else if (key == "PositionTopic") {

    			//std::cout << text << std::endl;
    		}
    		else {

    			ROS_WARN_STREAM("Unknown element in the configuration.");
    		}

    		element = element->NextSiblingElement();
    	}

    	// Uzel audio.
    	element = root->FirstChild("Audio")->FirstChildElement();

    	while (element) {

			const string & key = element->ValueStr();
			string text;
			if (element && element->GetText()) {
				text = element->GetText();
			}

			if (key.empty() || text.empty()) {

				element = element->NextSiblingElement();
				ROS_WARN_STREAM("Error in the configuration.");

				continue;
			}

    		if (key == "Topic") {

    			//std::cout << text << std::endl;
    		}
    		else {

    			ROS_WARN_STREAM("Unknown element in the configuration.");
    		}

    		element = element->NextSiblingElement();
    	}

    	// Uzel strategy.
    	element = root->FirstChild("Strategy")->FirstChildElement();

    	while (element) {

			const string & key = element->ValueStr();
			string text;
			if (element && element->GetText()) {
				text = element->GetText();
			}

			if (key.empty() || text.empty()) {

				element = element->NextSiblingElement();
				ROS_WARN_STREAM("Error in the configuration.");

				continue;
			}

    		if (key == "Type") {


    			if (text == "1") {

    				_strategy = Strategy::FIRST;
    			}
    			else if (text == "2") {

    				_strategy = Strategy::SECOND;
    			}
    			else if (text == "3") {

    				_strategy = Strategy::THIRD;
    			}
    			else {
    				ROS_WARN_STREAM("Bad strategy in the configuration.");
    			}


    		}
    		else {

    			ROS_WARN_STREAM("Unknown element in the configuration.");
    		}

    		element = element->NextSiblingElement();
    	}

    }
    else {
    	ROS_ERROR_STREAM("Can not open the configuration.");

    	return false;
    }

    return true;
}


