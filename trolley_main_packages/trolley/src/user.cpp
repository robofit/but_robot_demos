/*
 * user.cpp
 *
 *  Created on: Mar 11, 2013
 *      Author: tomaskolo
 */

#include "user.h"
#include <stdexcept>
#include <iostream>

User & User::getInstance()
{
	static User instance;
	// Vrati instanci.
	return instance;
}

Nullable<XnPoint3D> User::UserPosition()
{
	// Proved update.
	_context.WaitAndUpdateAll();

	if(_trackingID.HasValue()) {
		// Sleduje se kontretni ID cile.

		// Promena pro ulozeni pozice cile.
		XnPoint3D userPosition;
		// Vrat pozice cile podle ID.
		XnStatus status = _userGenerator.GetCoM(_trackingID, userPosition);
		if (status != XN_STATUS_OK) {
			// Sledovany cil se ztratil.

			_calibration = 20;
			// Nastav ID cile na null.
			_trackingID.SetNull();
			// Vyber dalsi cil, ktrery se bude sledovat.
			return UserPosition();
		}
		else if (userPosition.X == 0.0 && userPosition.Y == 0.0 && userPosition.Z == 0.0) {
			// Sledovany cil se ztratil.

			_calibration = 20;
			// Nastav ID cile na null.
			_trackingID.SetNull();
			// Vyber dalsi cil, ktrery se bude sledovat.
			return UserPosition();
		}
		else if (_calibration != 0){
			--_calibration;
			return Nullable<XnPoint3D>();
		}
		else {
			// Vrati aktualizovanou pocizi cile.
			return userPosition;
		}
	}
	else {
		// Nesledujeme zadny cil. Vyber prvni platny.

		XnUserID users[15];
		// Velikost pole users.
		XnUInt16 usersCount = 15;
		// Po provedeni funkce se argument "usersCount" zmeni na pocet nalezenych uzivatelu.
		_userGenerator.GetUsers(users, usersCount);

		for (XnUInt16 i = 0; i < usersCount; ++i) {

			XnPoint3D userPosition;
			XnStatus status = _userGenerator.GetCoM(users[i], userPosition);
			if (status != XN_STATUS_OK) {
				// Cil neni platny. Vyber dalsi.
				continue;
			}
			else if (userPosition.X == 0.0 && userPosition.Y == 0.0 && userPosition.Z == 0.0) {
				// Cil neni platny. Vyber dalsi.
				continue;
			}
			else {
				// Cil je platny.
				// Uloz ID cile.
				_trackingID = users[i];
				// Vrat polohu.
				return Nullable<XnPoint3D>();
			}
		}
	}
	return Nullable<XnPoint3D>();
}

void User::TargetConfiguration(unsigned int calibration)
{
	_calibration = calibration;
}

User::User()
	: _calibration(20), _configurationFile("OpenNI.xml")
{
	XnStatus status = _context.InitFromXmlFile(_configurationFile.c_str());
	if (status != XN_STATUS_OK) {
		throw std::runtime_error("Bad configuration or Kinect is not connected");
	}

	status = _context.FindExistingNode(XN_NODE_TYPE_DEPTH, _depthGenerator);
	if (status != XN_STATUS_OK) {
		throw std::runtime_error("Find depth generator failed");
	}

	status = _context.FindExistingNode(XN_NODE_TYPE_USER, _userGenerator);
	if (status != XN_STATUS_OK) {

		status = _userGenerator.Create(_context);
		if (status != XN_STATUS_OK) {
			throw std::runtime_error("Find user generator failed");
		}
	}

	if (!_userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		throw std::runtime_error("Supplied user generator doesn't support skeleton");
	}

	_userGenerator.RegisterUserCallbacks(User::NewUser,User::LostUser, NULL, _userCallbacksHandle);

	//userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	status = _context.StartGeneratingAll();
	if (status != XN_STATUS_OK) {
		throw std::runtime_error("Failed: startGenerating");
	}
}

User::~User()
{
	_context.Shutdown();
}

void XN_CALLBACK_TYPE User::NewUser(xn::UserGenerator & generator, XnUserID nId, void* pCookie)
{
	std::cout << "New user" << nId << std::endl;
}


void XN_CALLBACK_TYPE User::LostUser(xn::UserGenerator & generator, XnUserID nId, void* pCookie)
{
	std::cout << "Lost user" << nId << std::endl;
}



