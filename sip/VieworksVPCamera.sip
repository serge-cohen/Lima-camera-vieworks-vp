/* vieworks-vp plugin camera class interface generation
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
namespace VieworksVP
{
  class Camera /NoDefaultCtors/
  {
%TypeHeaderCode
#include <VieworksVPCamera.h>
%End

  public:

// SIP is not working with the namespace, we have to give the enums and use them directly in here, even the one coming from other parts of LIMA :

    enum Status { Ready, Running, Fault };
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

      void prepareAcq();
      // Launches the SDK's acquisition and the m_acq_thread to retrieve frame buffers as they are ready
      void startAcq();
      // Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
      void stopAcq();

      // -- detector info object
      void getImageType(ImageType& type /Out/ );
      void setImageType(ImageType type /In/ );
      
      void getDetectorType(std::string& type /Out/ );
      void getDetectorModel(std::string& model /Out/ );
      void getDetectorImageSize(Size& size /Out/ );

      // -- Buffer control object
      HwBufferCtrlObj* getBufferCtrlObj();
      
      //-- Synch control object
      bool checkTrigMode(TrigMode mode /In/ );
      void setTrigMode(TrigMode  mode /In/ );
      void getTrigMode(TrigMode& mode /Out/ );
      
      void setExpTime(double  exp_time /In/ );
      void getExpTime(double& exp_time /Out/ );
      
      void setLatTime(double  lat_time /In/ );
      void getLatTime(double& lat_time /Out/ );
      
      void getExposureTimeRange(double& min_expo /Out/ , double& max_expo /Out/ ) const;
      void getLatTimeRange(double& min_lat /Out/ , double& max_lat /Out/ ) const;
      
      void setNbFrames(int  nb_frames /In/ );
      void getNbFrames(int& nb_frames /Out/ );
      void getNbHwAcquiredFrames(int &nb_acq_frames /Out/ );
      
      void checkRoi(const Roi& set_roi /In/ , Roi& hw_roi /Out/ );
      void setRoi(const Roi& set_roi /In/ );
      void getRoi(Roi& hw_roi /Out/ );
      
      bool isBinningAvailable();
      void checkBin(Bin& ioBin /In,Out/ );
      void setBin(const Bin& iBin /In/ );
      void getBin(Bin& oBin /Out/ );
      
      void setShutterMode(ShutterMode mode /In/ );
      void getShutterMode(ShutterMode& mode /Out/ );
      
      void getPixelSize(double& sizex /Out/ , double& sizey /Out/ );

      void getStatus(Status& o_status /Out/ );
      
      // --- Acquisition interface
      void reset();
      // int getNbHwAcquiredFrames();

      // -- vieworks-vp specific, LIMA don't worry about it !
      void setTestImage(VP_test_image i_test_image /In/ );
      void getTestImage(VP_test_image &o_test_image /Out/ ) const;
      void setDataBits(VP_data_bits i_data_bits /In/ );
      void getDataBits(VP_data_bits &o_data_bits /Out/ ) const;
      void setLUTcontrol(VP_LUT_control i_lut /In/ );
      void getLUTcontrol(VP_LUT_control &o_lut /Out/ ) const;
      void setAsynchronousReset(bool i_AR /In/ );
      void getAsynchronousReset(bool &o_AR /Out/ ) const;
      void setFlatFieldCorrection(bool i_FFC /In/ );
      void getFlatFieldCorrection(bool &o_FFC /Out/ ) const;
      void setDefectCorrection(bool i_DC /In/ );
      void getDefectCorrection(bool &o_DC /Out/ ) const;
      void setImageInvert(bool i_II /In/ );
      void getImageInvert(bool &o_II /Out/ ) const;
      void setHorizontalFlip(bool i_HF);              // To export, value 0-1 EXPERT
      void getHorizontalFlip(bool &o_HF /Out/ ) const;
      void setTrigger(VP_trigger_mode i_trig);
      void getTrigger(VP_trigger_mode &o_trig /Out/ ) const;
      void setExpSource(VP_exp_source i_exp_src);
      void getExpSource(VP_exp_source &o_exp_src /Out/ ) const;
      void setTriggerSource(VP_trigger_source i_trig_src);
      void getTriggerSource(VP_trigger_source &o_trig_src /Out/ ) const;
      void setTriggerPolarity(bool i_pol);            // To export, value 0-1 EXPERT
      void getTriggerPolarity(bool &o_pol /Out/ ) const;
      void setExpMusTime(unsigned int i_time);
      void getExpMusTime(unsigned int &o_time /Out/ ) const;
      void setStrobeOffsetMus(unsigned int i_time);   // To export, value 0-10000 EXPERT
      void getStrobeOffsetMus(unsigned int &o_time /Out/ ) const;
      void setStrobePolarity(bool i_pol);             // To export, value 0-1 EXPERT
      void getStrobePolarity(bool &o_pol /Out/ ) const;
      void setAnalogGain(unsigned short i_gain);      // To export, value 0-899
      void getAnalogGain(unsigned short &o_gain /Out/ ) const;
      void setAnalogOffset(unsigned char i_off);      // To export, value 0-255
      void getAnalogOffset(unsigned char &o_off /Out/ ) const;
      void setFlatFieldIteration(unsigned char i_iter); // To export, value 0-4 EXPERT
      void getFlatFieldIteration(unsigned char &o_iter /Out/ ) const;
      void setFlatFieldOffset(unsigned short i_off);    // To export, value 0-4095 EXPERT
      void getFlatFieldOffset(unsigned short &o_off /Out/ ) const;
      
      void setTemperatureSP(int i_temp);             // To export, value 5-
      void getTemperatureSP(int &o_temp /Out/ ) const;
      void setPixelClock(VP_pixel_clock i_clk);      // To export, value 0-1
      void getPixelClock(VP_pixel_clock &o_clk /Out/ ) const;
      void setFanStatus(bool i_bootl);               // To export, value 0-1
      void getFanStatus(bool &o_bool /Out/ ) const;
      void setPeltierControl(bool i_bootl);          // To export, value 0-1
      void getPeltierControl(bool &o_bool /Out/ ) const;
      
      void getMCUversion(std::string &o_string /Out/ ) const;   // To export
      void getModelNumber(std::string &o_string /Out/ ) const;  // To export
      void getFPGAversion(std::string &o_string /Out/ ) const;  // To export
      void getSerialNumber(std::string &o_string /Out/ ) const; // To export
      void getCurrentTemperature(double &o_temp /Out/ ) const;  // To export
      void getSensorTemperature(double &o_temp /Out/ ) const;   // To export
      
      void setOneParam(std::string i_name, int i_value1);
      void setTwoParams(std::string i_name, int i_value1, int i_value2);
      void getOneParam(std::string i_name, std::string &o_value1 /Out/ ) const;
      // void getOneParam(std::string i_name, int &o_value1 /Out/ ) const;
      void getTwoParams(std::string i_name, int &o_value1 /Out/ , int &o_value2 /Out/ ) const;
      std::string command(const std::string& i_cmd);
      std::string get_command(const std::string& i_cmd) const;
      // static int checkComError(const std::string &i_answer);
      static int checkComError(const std::string &i_answer, std::string &o_message /Out/ );
      
      void getReadoutTime(double &o_time /Out/ ) const; // Computes the readout time.
      void computeModeAndFPS();
  };

};

