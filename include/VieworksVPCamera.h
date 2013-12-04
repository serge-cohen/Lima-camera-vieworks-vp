#ifndef VIEWORKSVPCAMERA_H
#define VIEWORKSVPCAMERA_H

/* vieworks-vp plugin camera class
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

#if defined (__GNUC__) && (__GNUC__ == 3) && defined (__ELF__)
#   define GENAPI_DECL __attribute__((visibility("default")))
#   define GENAPI_DECL_ABSTRACT __attribute__((visibility("default")))
#endif

// System headers :
#include <stdlib.h>
#include <limits>
#include <ostream>

// Camera SDK headers :
#include <atcore.h>

// LImA headers :
#include "HwMaxImageSizeCallback.h"
#include "HwBufferMgr.h"

// VieworksVP plugin headers :

namespace lima
{
  namespace VieworksVP
  {
    /*******************************************************************
     * \class Camera
     * \brief object controlling the vieworks-vp camera via vieworks-vp SDK driver
     *******************************************************************/
    class Camera
    {
      DEB_CLASS_NAMESPC(DebModCamera, "Camera", "VieworksVP");
    public:
      
      enum Status { Ready, Exposure, Readout, Latency, Fault };

      // Enum representing the entry possible for gti/sti (test image)
      enum VP_test_image { Off=0, Fixed1=1, Fixed2=2, Moving=3 };
      
      Camera(const std::string& bitflow_path, int camera_number=0);
      ~Camera();

      // Preparing the camera's SDK to acquire frames
      void prepareAcq();
      // Launches the SDK's acquisition and the m_acq_thread to retrieve frame buffers as they are ready
      void startAcq();
      // Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
      void stopAcq();

      // -- detector info object
      void getImageType(ImageType& type);
      void setImageType(ImageType type);
      
      void getDetectorType(std::string& type);
      void getDetectorModel(std::string& model);
      void getDetectorImageSize(Size& size);

      // -- Buffer control object
      HwBufferCtrlObj* getBufferCtrlObj();
      
      //-- Synch control object
      bool checkTrigMode(TrigMode mode);
      void setTrigMode(TrigMode  mode);
      void getTrigMode(TrigMode& mode);
      
      void setExpTime(double  exp_time);
      void getExpTime(double& exp_time);
      
      void setLatTime(double  lat_time);
      void getLatTime(double& lat_time);
      
      void getExposureTimeRange(double& min_expo, double& max_expo) const;
      void getLatTimeRange(double& min_lat, double& max_lat) const;
      
      void setNbFrames(int  nb_frames);
      void getNbFrames(int& nb_frames);
      void getNbHwAcquiredFrames(int &nb_acq_frames);
      
      void checkRoi(const Roi& set_roi, Roi& hw_roi);
      void setRoi(const Roi& set_roi);
      void getRoi(Roi& hw_roi);
      
      bool isBinningAvailable();
      void checkBin(Bin& ioBin);
      void setBin(const Bin& iBin);
      void getBin(Bin& oBin);
      
      void setShutterMode(ShutterMode mode);
      void getShutterMode(ShutterMode& mode);
      
      void getPixelSize(double& sizex, double& sizey);

      void getStatus(Camera::Status& status);
      
      // --- Acquisition interface
      void reset();
      int getNbHwAcquiredFrames();

      // -- vieworks-vp specific, LIMA don't worry about it !
      void initialiseController();

      void setTemperatureSP(double temp);  // à exporter (avec le get)
      void getTemperatureSP(double& temp);
      void getTemperature(double& temp);   // à exporter (read-only)
      void setCooler(bool flag);					 // à exporter (avec le get)
      void getCooler(bool& flag);
      void getCoolingStatus(std::string& status);  // à exporter (read-only)

    private:
      // -- some internals :
      // Stopping an acquisition, iForce : without waiting the end of frame buffer retrieval by m_acq_thread
      void _stopAcq(bool iImmediate);
      // Setting the status in a thread safe manner :
      void _setStatus(Camera::Status iStatus, bool iForce);
      
    private:
      // -- vieworks-vp Lower level functions
      
    private:
      class _AcqThread;
      friend class _AcqThread;

      // -- Members
      // LIMA / Acquisition (thread) related :
      SoftBufferCtrlObj						m_buffer_ctrl_obj;
      // Pure thread and signals :
      _AcqThread*                 m_acq_thread;						// The thread retieving frame buffers from the SDK
      Cond                        m_cond;									// Waiting condition for inter thread signaling
      volatile bool								m_acq_thread_waiting;   // The m_acq_thread is waiting (main uses it to tell it to stop waiting)
      volatile bool								m_acq_thread_running;		// The m_acq_thread is running (main uses it to accept stopAcq)
      volatile bool								m_acq_thread_should_quit; // The main thread signals to m_acq_thread that it should quit.

      // A bit more general :
      size_t											m_nb_frames_to_collect; // The number of frames to collect in current sequence
      size_t											m_image_index;					// The index in the current sequence of the next image to retrieve
      bool												m_buffer_ringing;				// Should the buffer be considered as a ring buffer rather than a single use buffer.
      Status											m_status;								// The current status of the camera
      
      // LIMA / Not directly acquisition related :
      std::string                 m_detector_model;
      std::string                 m_detector_type;
      std::string									m_detector_serial;
      Size												m_detector_size;
      double											m_exp_time;

      // -- vieworks-vp SDK stuff
      bool                        m_cooler;
      double                      m_temperature_sp;
      std::map<int, std::string>  m_andor3_error_maps;

      static bool						s_SDK_initted;
    };
    

  } // namespace VieworksVP
} // namespace lima


#endif  /* VIEWORKSVPCAMERA_H */

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
