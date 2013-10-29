/* vieworks-vp plugin synchronisation information class
 * Copyright (C) 2013 IPANEMA USR3461, CNRS/MCC.
 * Written by Serge Cohen <serge.cohen@synchrotron-soleil.fr>
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3 of
 * the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this file. If not, see <http://www.gnu.org/licenses/>.
 */

// System headers :
//#include <sstream>
//#include <iostream>
//#include <string>
//#include <math.h>

// Camera SDK headers :

// LImA headers :

// VieworksVP plugin headers :
#include "VieworksVPSyncCtrlObj.h"


//---------------------------
//- utility variables
//---------------------------


//---------------------------
//- @brief constructor
//---------------------------
lima::VieworksVP::SyncCtrlObj::SyncCtrlObj(lima::VieworksVP::Camera& cam) :
m_cam(cam)
{
  DEB_CONSTRUCTOR();
}

//---------------------------
//- @brief destructor
//---------------------------
lima::VieworksVP::SyncCtrlObj::~SyncCtrlObj()
{
  DEB_DESTRUCTOR();
}


#pragma mark -
#pragma mark Trigger Mode :
bool
lima::VieworksVP::SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
  DEB_MEMBER_FUNCT();
  return m_cam.checkTrigMode(trig_mode);
}
void
lima::VieworksVP::SyncCtrlObj::setTrigMode(TrigMode  trig_mode)
{
  DEB_MEMBER_FUNCT();
  m_cam.setTrigMode(trig_mode);
}
void
lima::VieworksVP::SyncCtrlObj::getTrigMode(TrigMode& trig_mode)
{
  DEB_MEMBER_FUNCT();
  m_cam.getTrigMode(trig_mode);
}

#pragma mark -
#pragma mark Exposition time :
void
lima::VieworksVP::SyncCtrlObj::setExpTime(double  exp_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.setExpTime(exp_time);
}
void
lima::VieworksVP::SyncCtrlObj::getExpTime(double& exp_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.getExpTime(exp_time);
}

#pragma mark -
#pragma mark Latency time :
void
lima::VieworksVP::SyncCtrlObj::setLatTime(double  lat_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.setLatTime(lat_time);
}
void
lima::VieworksVP::SyncCtrlObj::getLatTime(double& lat_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.getLatTime(lat_time);
}

#pragma mark -
#pragma mark Frame numbers :
void
lima::VieworksVP::SyncCtrlObj::setNbHwFrames(int  nb_frames)
{
  DEB_MEMBER_FUNCT();
  m_cam.setNbFrames(nb_frames);
}
void
lima::VieworksVP::SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
  DEB_MEMBER_FUNCT();
  m_cam.getNbHwAcquiredFrames(nb_frames);
}

void
lima::VieworksVP::SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
  DEB_MEMBER_FUNCT();
  double min_time;
  double max_time;
  m_cam.getExposureTimeRange(min_time, max_time);
  valid_ranges.min_exp_time = min_time;
  valid_ranges.max_exp_time = max_time;
  
  m_cam.getLatTimeRange(min_time, max_time);
  valid_ranges.min_lat_time = min_time;
  valid_ranges.max_lat_time = max_time; 
}


/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
