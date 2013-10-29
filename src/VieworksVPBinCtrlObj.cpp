/* vieworks-vp plugin binning class
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

#include "Andor3BinCtrlObj.h"

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
lima::VieworksVP::BinCtrlObj::BinCtrlObj(Camera &cam) : m_cam(cam)
{
  DEB_CONSTRUCTOR();
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
lima::VieworksVP::BinCtrlObj::~BinCtrlObj()
{
  DEB_DESTRUCTOR();
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void
lima::VieworksVP::BinCtrlObj::getBin(Bin &aBin)
{
  DEB_MEMBER_FUNCT();
  m_cam.getBin(aBin);
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void
lima::VieworksVP::BinCtrlObj::checkBin(Bin &aBin)
{
  DEB_MEMBER_FUNCT();
  m_cam.checkBin(aBin);
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void
lima::VieworksVP::BinCtrlObj::setBin(const Bin& aBin)
{
  DEB_MEMBER_FUNCT();    
  m_cam.setBin(aBin);
}

