/*
 * user.h
 *
 *  Created on: Mar 11, 2013
 *      Author: tomaskolo
 */

#ifndef USER_H_
#define USER_H_

#include "nullable.h"

#include <XnCppWrapper.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <string>

class User {
public:
	/**
	 * Navrhovy vzor jedinacek.
	 */
	static User & getInstance();

	/**
	 * Destruktor.
	 */
	~User();

	Nullable<XnPoint3D> UserPosition();

	void TargetConfiguration(unsigned int calibration);

private:
	/**
	 * Privatni konstruktor.
	 */
	User();

	/**
	 * Copy konstruktor neni implementovan.
	 */
	User(const User &);

	/**
	 * Operator prirazeni neni implementovan.
	 */
	void operator=(const User &);

	/**
	 * Metoda volana pri nalezeni noveho uzivatele.
	 */
	static void  XN_CALLBACK_TYPE NewUser(xn::UserGenerator & generator, XnUserID nId, void* pCookie);

	/**
	 * Metoda volana pri ztrate uzivatele.
	 */
	static void XN_CALLBACK_TYPE LostUser(xn::UserGenerator & generator, XnUserID nId, void* pCookie);

	///< Kalibrace.
	unsigned int _calibration;

	///< Konfiguracni soubor pro OpenNI
	const std::string _configurationFile;

	///< OpenNI kontext.
	xn::Context	_context;

	///< Hloubkovy generator OpenNI.
	xn::DepthGenerator _depthGenerator;

	///< User OpenNi.
	xn::UserGenerator _userGenerator;

	///< Handle
	XnCallbackHandle _userCallbacksHandle;

	///< User track ID.
	Nullable<XnUserID> _trackingID;

};

#endif /* USER_H_ */
