#ifndef VIEWORKSVPDETINFOCTRLOBJ_H
#define VIEWORKSVPDETINFOCTRLOBJ_H

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

// Camera SDK headers :

// LImA headers :
#include "lima/HwInterface.h"

// VieworksVP plugin headers :
#include "VieworksVPCamera.h"

namespace lima
{
  namespace VieworksVP
  {
    
    /*******************************************************************
     * \class DetInfoCtrlObj
     * \brief Control object providing VieworksVP detector info interface
     *******************************************************************/
    
    class DetInfoCtrlObj : public HwDetInfoCtrlObj
    {
	    DEB_CLASS_NAMESPC(DebModCamera, "DetInfoCtrlObj", "VieworksVP");
      
    public:
	    DetInfoCtrlObj(Camera& cam);
	    virtual ~DetInfoCtrlObj();
      
	    virtual void getMaxImageSize(Size& max_image_size);
	    virtual void getDetectorImageSize(Size& det_image_size);
      
	    virtual void getDefImageType(ImageType& def_image_type);
	    virtual void getCurrImageType(ImageType& curr_image_type);
	    virtual void setCurrImageType(ImageType  curr_image_type);
      
	    virtual void getPixelSize(double& xsize, double& ysize);
	    virtual void getDetectorType(std::string& det_type);
	    virtual void getDetectorModel(std::string& det_model);
      
	    virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
	    virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
      
    private:
	    Camera& m_cam;
    };
    
  } // namespace VieworksVP
} // namespace lima


#endif  /* VIEWORKSVPDETINFOCTRLOBJ_H */

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
