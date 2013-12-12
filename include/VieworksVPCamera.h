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
#include "SiSoME4Grabber.h"
#include "SiSoME4SerialLine.h"

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
      
      Camera(const std::string& i_siso_dir_5="", int board_index=0, int cam_port=0, const std::string& applet_name="mirror_vd4_VieworksVP29MC.hap", unsigned int dma_index=0);
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
      void setTestImage(VP_test_image i_test_image);  // to export, values 0-3 EXPERT
      void getTestImage(VP_test_image &o_test_image) const;
      void setDataBits(VP_data_bits i_data_bits);     // To export, values 8,10,12
      void getDataBits(VP_data_bits &o_data_bits) const;
      void setLUTcontrol(VP_LUT_control i_lut);       // To export, values 0-2 EXPERT
      void getLUTcontrol(VP_LUT_control &o_lut) const;
      void setAsynchronousReset(bool i_AR);           // To export, value 0-1 EXPERT
      void getAsynchronousReset(bool &o_AR) const;
      void setFlatFieldCorrection(bool i_FFC);        // To export, value 0-1 EXPERT
      void getFlatFieldCorrection(bool &o_FFC) const;
      void setDefectCorrection(bool i_DC);            // To export, value 0-1 EXPERT
      void getDefectCorrection(bool &o_DC) const;
      void setImageInvert(bool i_II);                 // To export, value 0-1 EXPERT
      void getImageInvert(bool &o_II) const;
      void setHorizontalFlip(bool i_HF);              // To export, value 0-1 EXPERT
      void getHorizontalFlip(bool &o_HF) const;
      void setTrigger(VP_trigger_mode i_trig);
      void getTrigger(VP_trigger_mode &o_trig) const;
      void setExpSource(VP_exp_source i_exp_src);
      void getExpSource(VP_exp_source &o_exp_src) const;
      void setTriggerSource(VP_trigger_source i_trig_src);
      void getTriggerSource(VP_trigger_source &o_trig_src) const;
      void setTriggerPolarity(bool i_pol);            // To export, value 0-1 EXPERT
      void getTriggerPolaroty(bool &o_pol) const;
      void setExpMusTime(unsigned int i_time);
      void getExpMusTime(unsigned int &o_time) const;
      void setStrobeOffsetMus(unsigned int i_time);   // To export, value 0-10000 EXPERT
      void getStrobeOffsetMus(unsigned int &o_time) const;
      void setStrobePolarity(bool i_pol);             // To export, value 0-1 EXPERT
      void getStrobePolaroty(bool &o_pol) const;
      void setAnalogGain(unsigned short i_gain);      // To export, value 0-899
      void getAnalogGain(unsigned short &o_gain) const;
      void setAnalogOffset(unsigned char i_off);      // To export, value 0-255
      void getAnalogOffset(unsigned char &o_off) const;
      void setFlatFieldIteration(unsigned char i_iter); // To export, value 0-4 EXPERT
      void getFlatFieldIteration(unsigned char &o_iter) const;
      void setFlatFieldOffset(unsigned short i_off);    // To export, value 0-4095 EXPERT
      void getFlatFieldOffset(unsigned short &o_off) const;
      
      void setTemperatureSP(int i_temp);             // To export, value 5-
      void getTemperatureSP(int &o_temp) const;
      void setPixelClock(VP_pixel_clock i_clk);      // To export, value 0-1
      void getPixelClock(VP_pixel_clock &o_clk) const;
      void setFanStatus(bool i_bootl);               // To export, value 0-1
      void getFanStatus(bool &o_bool) const;
      void setPeltierControl(bool i_bootl);          // To export, value 0-1
      void getPeltierControl(bool &o_bool) const;
      
      void getMCUversion(std::string &o_string) const;   // To export
      void getModelNumber(std::string &o_string) const;  // To export
      void getFPGAversion(std::string &o_string) const;  // To export
      void getSerialNumber(std::string &o_string) const; // To export
      void getCurrentTemperature(double &o_temp) const;  // To export
      void getSensorTemperature(double &o_temp) const;   // To export
      
      void setOneParam(std::string i_name, int i_value1);
      void setTwoParams(std::string i_name, int i_value1, int i_value2);
      void getOneParam(std::string i_name, std::string &o_value1) const;
      void getOneParam(std::string i_name, int &o_value1) const;
      void getTwoParams(std::string i_name, int &o_value1, int &o_value2) const;
      std::string command(const std::string& i_cmd);
      std::string get_command(const std::string& i_cmd) const;
      static int checkComError(const std::string &i_answer);
      static int checkComError(const std::string &i_answer, std::string &o_message);
    private:
      // -- some internals :
      // Stopping an acquisition, iForce : without waiting the end of frame buffer retrieval by m_acq_thread
      void doStopAcq(bool iImmediate);
      // Setting the status in a thread safe manner :
      void setStatus(Camera::Status iStatus, bool iForce);
      
    private:
      // -- vieworks-vp Lower level functions
      
    private:
      class _AcqThread;
      friend class _AcqThread;

      
      // -- vieworks-vp SDK stuff
      siso_me4::Grabber						m_grabber;
      siso_me4::SerialLine				&m_serial_line;

      // LIMA / Not directly acquisition related :
      std::string                 m_detector_model;
      std::string                 m_detector_type;
      std::string									m_detector_serial;
      Size												m_detector_size;
      double											m_exp_time;
      
      
    };
    

  } // namespace VieworksVP
} // namespace lima


#endif  /* VIEWORKSVPCAMERA_H */

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
