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
      enum VP_data_bits {VP_8bits=8, VP_10bits=10, VP_12bits=12};
      enum VP_LUT_control {VP_no_lut=0, VP_lut_1=1, VP_lut_2=2};
      enum VP_trigger_mode {VP_free_run=0, VP_std_mode=1, VP_fast_mode=2, VP_double_mode=3, VP_overlap_mode=4};
      enum VP_exp_source {VP_camera=0, VP_pulse_width=1};
      enum VP_trigger_source {VP_CC1=1, VP_external=2};
      enum VP_pixel_clock {VP_30MHz_pclk=0, VP_40MHz_pclk=1};
      
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

      void setTestImage(VP_test_image i_test_image);
      void getTestImage(VP_test_image &o_test_image) const;
      void setDataBits(VP_data_bits i_data_bits);
      void getDataBits(VP_data_bits &o_data_bits) const;
      void setLUTcontrol(VP_LUT_control i_lut);
      void getLUTcontrol(VP_LUT_control &o_lut) const;
      void setAsynchronousReset(bool i_AR);
      void getAsynchronousReset(bool &o_AR) const;
      void setFlatFieldCorrection(bool i_FFC);
      void getFlatFieldCorrection(bool &o_FFC) const;
      void setDefectCorrection(bool i_DC);
      void getDefectCorrection(bool &o_DC) const;
      void setImageInvert(bool i_II);
      void getImageInvert(bool &o_II) const;
      void setHorizontalFlip(bool i_HF);
      void getHorizontalFlip(bool &o_HF) const;
      void setTrigger(VP_trigger_mode i_trig);
      void getTrigger(VP_trigger_mode &o_trig) const;
      void setExpSource(VP_exp_source i_exp_src);
      void getExpSource(VP_exp_source &o_exp_src) const;
      void setTriggerSource(VP_trigger_source i_trig_src);
      void getTriggerSource(VP_trigger_source &o_trig_src) const;
      void setTriggerPolarity(bool i_pol);
      void getTriggerPolaroty(bool &o_pol) const;
      void setExpMusTime(unsigned int i_time);
      void getExpMusTime(unsigned int &o_time) const;
      void setStrobeOffsetMus(unsigned int i_time);
      void getStrobeOffsetMus(unsigned int &o_time) const;
      void setStrobePolarity(bool i_pol);
      void getStrobePolaroty(bool &o_pol) const;
      void setAnalogGain(unsigned short i_gain);
      void getAnalogGain(unsigned short &o_gain) const;
      void setAnalogOffset(unsigned char i_off);
      void getAnalogOffset(unsigned char &o_off) const;
      void setFlatFieldIteration(unsigned char i_iter);
      void getFlatFieldIteration(unsigned char &o_iter) const;
      void setFlatFieldOffset(unsigned short i_off);
      void getFlatFieldOffset(unsigned short &o_off) const;
      
      void setTemperatureSP(int i_temp);
      void getTemperatureSP(int &o_temp) const;
      void setPixelClock(VP_pixel_clock i_clk);
      void getPixelClock(VP_pixel_clock &o_clk) const;
      void setFanStatus(bool i_bootl);
      void getFanStatus(bool &o_bool) const;
      void setPeltierControl(bool i_bootl);
      void getPeltierControl(bool &o_bool) const;
      
      void getMCUversion(std::string &o_string) const;
      void getModelNumber(std::string &o_string) const;
      void getFPGAversion(std::string &o_string) const;
      void getSerialNumber(std::string &o_string) const;
      void getCurrentTemperature(double &o_temp) const;
      void getSensorTemperature(double &o_temp) const;
      
      void setOneParam(std::string i_name, std::string i_value1);
      void setTwoParam(std::string i_name, std::string i_value1, std::string i_value2);
      void getOneParam(std::string i_name, std::string &o_value1) const;
      void getTwoParam(std::string i_name, std::string &o_value1, std::string &o_value2) const;
      void command(std::string i_name);
      
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

      
      // LIMA / Not directly acquisition related :
      std::string                 m_detector_model;
      std::string                 m_detector_type;
      std::string									m_detector_serial;
      Size												m_detector_size;
      double											m_exp_time;

      // -- vieworks-vp SDK stuff
      bool                        m_cooler;
      double                      m_temperature_sp;

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
