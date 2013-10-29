#ifndef VIEWORKSVPINTERFACE_H
#define VIEWORKSVPINTERFACE_H

/* vieworks-vp plugin hardware interface class
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

// Camera SDK headers :

// LImA headers :
#include "HwInterface.h"

// VieworksVP plugin headers :
#include "VieworksVPCamera.h"
#include "VieworksVPDetInfoCtrlObj.h"
#include "VieworksVPSyncCtrlObj.h"
#include "VieworksVPRoiCtrlObj.h"
#include "VieworksVPBinCtrlObj.h"

namespace lima
{
  namespace VieworksVP
  {
    class Interface;
    
    
    /*******************************************************************
     * \class Interface
     * \brief VieworksVP hardware interface
     *******************************************************************/
    
    class Interface : public HwInterface
    {
	    DEB_CLASS_NAMESPC(DebModCamera, "VieworksVPInterface", "VieworksVP");
      
    public:
	    Interface(Camera& cam);
	    virtual ~Interface();
      
	    //- From HwInterface
	    virtual void    getCapList(CapList& o_cap_list) const;
	    virtual void    reset(ResetLevel reset_level);
	    virtual void    prepareAcq();
	    virtual void    startAcq();
	    virtual void    stopAcq();
	    virtual void    getStatus(StatusType& status);
	    virtual int     getNbHwAcquiredFrames();
      
      // Giving the possibility to get directly the camera object :
      Camera& getCamera()
      { return m_cam; }
        
      
    private:
	    Camera&         m_cam;
      CapList         m_cap_list;
	    DetInfoCtrlObj  *m_det_info;
	    SyncCtrlObj     *m_sync;
      RoiCtrlObj			*m_roi;
      BinCtrlObj			*m_bin;
    };
    
    
    
  } // namespace VieworksVP
} // namespace lima


#endif  /* VIEWORKSVPINTERFACE_H */

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
