#ifndef VIEWORKSVPBINCTRLOBJ_H
#define VIEWORKSVPBINCTRLOBJ_H

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

#include "lima/HwInterface.h"
#include "VieworksVPCamera.h"

namespace lima
{
  namespace VieworksVP
  {
    
    /*******************************************************************
     * \class BinCtrlObj
     * \brief Control object providing VieworksVP Bin interface
     *******************************************************************/
    class BinCtrlObj : public HwBinCtrlObj
    {
	    DEB_CLASS_NAMESPC(DebModCamera, "BinCtrlObj", "VieworksVP");
      
	  public:
	    BinCtrlObj(Camera& cam);
	    virtual ~BinCtrlObj();
	    
	    virtual void getBin(Bin& bin);
	    virtual void checkBin(Bin& bin);
	    virtual void setBin(const Bin& bin);
	  private:
	    Camera& m_cam;
      
    };
    
  } // namespace VieworksVP
} // namespace lima

#endif // VIEWORKSVPBINCTRLOBJ_H
