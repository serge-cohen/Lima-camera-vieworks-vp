/* vieworks-vp plugin detector information class
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
#include "VieworksVPDetInfoCtrlObj.h"


//---------------------------
//- utility variables
//---------------------------


//---------------------------
//- @brief constructor
//---------------------------
lima::VieworksVP::DetInfoCtrlObj::DetInfoCtrlObj(lima::VieworksVP::Camera& cam) :
m_cam(cam)
{
  DEB_CONSTRUCTOR();
}

//---------------------------
//- @brief destructor
//---------------------------
lima::VieworksVP::DetInfoCtrlObj::~DetInfoCtrlObj()
{
  DEB_DESTRUCTOR();
}

void
lima::VieworksVP::DetInfoCtrlObj::getMaxImageSize(Size& max_image_size)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorImageSize(max_image_size);
}

void
lima::VieworksVP::DetInfoCtrlObj::getDetectorImageSize(Size& det_image_size)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorImageSize(det_image_size);
}

void
lima::VieworksVP::DetInfoCtrlObj::getDefImageType(ImageType& def_image_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.getImageType(def_image_type);
}

void
lima::VieworksVP::DetInfoCtrlObj::getCurrImageType(ImageType& curr_image_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.getImageType(curr_image_type);
}

void
lima::VieworksVP::DetInfoCtrlObj::setCurrImageType(ImageType  curr_image_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.setImageType(curr_image_type);
}

void
lima::VieworksVP::DetInfoCtrlObj::getPixelSize(double& xsize, double& ysize)
{
  DEB_MEMBER_FUNCT();
  m_cam.getPixelSize(xsize, ysize);
}

void
lima::VieworksVP::DetInfoCtrlObj::getDetectorType(std::string& det_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorType(det_type);
}

void
lima::VieworksVP::DetInfoCtrlObj::getDetectorModel(std::string& det_model)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorModel(det_model);
}


void
lima::VieworksVP::DetInfoCtrlObj::registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
  DEB_MEMBER_FUNCT();
  // Do nothing ... So far I on't know exactly what it is suppose to do
}

void
lima::VieworksVP::DetInfoCtrlObj::unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
  DEB_MEMBER_FUNCT();
  // Do nothing ... So far I on't know exactly what it is suppose to do  
}



/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
