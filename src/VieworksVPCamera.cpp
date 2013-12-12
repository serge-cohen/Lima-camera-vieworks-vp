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

// System headers :
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

// Camera SDK headers :

// LImA headers :

// VieworksVP plugin headers :
#include "VieworksVPCamera.h"


/* Some very local solution for string manipulations */
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <vector>

// trim from start
static inline std::string &ltrim(std::string &s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
  return ltrim(rtrim(s));
}

// Splitting a string on a given delimiter
static inline std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems, bool triming=false) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if ( triming ) {
      trim(item);
    }
    elems.push_back(item);
  }
  return elems;
}

namespace lima {
  namespace VieworksVP {
    // The enums defined by the VisualApplet SDK :
    namespace CameraGrayAreaBase {
      enum DvalMode {
        DVAL_Enabled	= 1,
        DVAL_Disabled	= 0
      };
      enum CLformat {
        SingleTap8Bit	= 0,
        SingleTap10Bit	= 1,
        SingleTap12Bit	= 2,
        SingleTap14Bit	= 3,
        SingleTap16Bit	= 4,
        DualTap8Bit	= 5,
        DualTap10Bit	= 6,
        DualTap12Bit	= 7
      };
      
    }
    
    namespace RemovePixel {
      enum m_t_FlushMode {
        EoL	= 0,
        EoF	= 1
      };
      
    }
    
    namespace ModuloCount {
      enum me_CountEntityType {
        PIXEL	= 0,
        LINE	= 1,
        FRAME	= 2
      };
      enum me_AutoClearMode {
        EoL	= 0,
        EoF	= 1,
        NONE	= 2
      };
      
    }
    
    namespace FrameBufferRandomRead {
      enum me_InfiniteSource {
        ENABLED	= 0,
        DISABLED	= 1
      };
      
    }
    
    namespace ImageFifo {
      enum me_InfiniteSource {
        ENABLED	= 0,
        DISABLED	= 1
      };
      
    }
    
    namespace SYNC {
      enum m_t_SyncModeType {
        SyncToMin	= 0,
        SyncToMax	= 1
      };
      
    }
    
    namespace TrgPortArea {
      enum m_e_TriggerModeN {
        GrabberControlled	= 1,
        ExternSw_Trigger	= 2
      };
      enum m_e_EnableN {
        OFF	= 0,
        ON	= 1
      };
      enum m_e_ImageTrgInSourceN {
        InSignal0	= 0,
        InSignal1	= 1,
        InSignal2	= 2,
        InSignal3	= 3,
        SoftwareTrigger	= 4,
        InSignal4	= 5,
        InSignal5	= 6,
        InSignal6	= 7,
        InSignal7	= 8
      };
      enum m_e_PolarityN {
        HighActive	= 0,
        LowActive	= 1
      };
      enum m_e_CCsourceN {
        Exsync	= 0,
        ExsyncInvert	= 1,
        Hdsync	= 2,
        HdsyncInvert	= 3,
        Flash	= 4,
        FlashInvert	= 5,
        Gnd	= 7,
        Vcc	= 8
      };
    }

    namespace names {
      std::string reorder_half_height1 = "Device1_Process0_Horizontale_Spiegelung_module8_Value";
      std::string reorder_quat_width = "Device1_Process0_Horizontale_Spiegelung_module39_ImageWidth";
      std::string reorder_half_height2 = "Device1_Process0_Horizontale_Spiegelung_module39_ImageHeight";
      std::string roi_off_x = "Device1_Process0_module41_XOffset";
      std::string roi_width = "Device1_Process0_module41_XLength";
      std::string roi_off_y = "Device1_Process0_module41_YOffset";
      std::string roi_half_height = "Device1_Process0_module41_YLength";
      std::string trig_mode = "Device1_Process0_TrigCam_TriggerMode";
      std::string trig_ext_sync = "Device1_Process0_TrigCam_ExsyncEnable";
      std::string trig_soft_pulse = "Device1_Process0_TrigCam_SoftwareTrgPulse";
      std::string trig_ext_fps = "Device1_Process0_TrigCam_ExsyncFramesPerSec";
      std::string trig_ext_width = "Device1_Process0_TrigCam_ExsyncExposure";
      std::string trig_cc1_signal = "Device1_Process0_TrigCam_CC1output";

    }
    
  }
}


/* NOTES :
 Liekly should set the following parameters :
 Param 0: FG_WIDTH,64  |  value=6576 0x19b0
 Param 1: FG_HEIGHT,c8  |  value=4384 0x1120
 Param 2: FG_TIMEOUT,258  |  value=1000000 0xf4240

 In term of triggering possibilities :
 Device1_Process0_TrigCam_TriggerMode
 Device1_Process0_TrigCam_CC1output
 
 About the parameters used in the VisualApplet used to address the camera :
 Device1_Process0_Camera_Format -> seems to set the format of the data (cf. enum CLformat)
 
 Device1_Process0_module41_XLength -> Image width
 Device1_Process0_module41_YLength -> Image half-height ???
 
 For the re-ordering of the pixels :
 Device1_Process0_Horizontale_Spiegelung_module5_RamAddressWidth (=A) -> the «width» of a «reordered-pixel»
 Device1_Process0_Horizontale_Spiegelung_module39_ImageWidth (=B) -> the width of the image in «reordered-pixels»
 Hence A*B/bytes_per_px should be the line width of the image (ROI) in true pixels
 
 Device1_Process0_Horizontale_Spiegelung_module39_ImageHeight -> Half the height of the ROI (which should be centered vertically !!!)
 
 */

// Defining the parameter names of the vieworks-vp SDK :

lima::VieworksVP::Camera::Camera(const std::string& i_siso_dir_5, int board_index, int cam_port, const std::string& applet_name, unsigned int dma_index) :
m_grabber(i_siso_dir_5, board_index, cam_port, applet_name, dma_index),
m_serial_line(m_grabber.getSerialLine()),
m_detector_model(""),
m_detector_type("Vieworks VP camera"),
m_detector_serial("UNKOWN"),
m_detector_size(lima::Size(1, 1)),
m_exp_time(-1.0)
{
  // Setting the serial line
  m_serial_line.setBaudRate(lima::siso_me4::SerialLine::BR19200);
  m_serial_line.setParity(lima::siso_me4::SerialLine::Off);
  m_serial_line.init();
  
  // Getting model
  getOneParam("mn", m_detector_model);
  
  // Getting the pixel size of the detector:
  //   * we first make sure we get to the full area no-binning mode :
  setOneParam("rm", 0);
  //   * then we extract the values from ha and va :
  int		the_left, the_right, the_top, the_bottom;
  getTwoParams("ha", the_left, the_right);
  getTwoParams("va", the_top, the_bottom);
  m_detector_size = lima::Size(the_right+1, the_bottom+1);
  
  // Getting exposure time (lima : exposure time in s)
  int		the_exp_time_mus;
  getOneParam("et", the_exp_time_mus);
  m_exp_time = static_cast<double>(the_exp_time_mus) * 1.0e-6;

  // Setting the camera to 12bit mode :
  setOneParam("db", 12);
  
  // Setting up the VisualApplet for the acquisition !!!
  // * First lets do the one that will never be changed :
  m_grabber.setParameterNamed("Device1_Process0_Camera_UseDval", CameraGrayAreaBase::DVAL_Enabled);
  m_grabber.setParameterNamed("Device1_Process0_module20_Divisor", 2);
  m_grabber.setParameterNamed("Device1_Process0_module11_Number", 1);
  m_grabber.setParameterNamed("Device1_Process0_module12_Number", 1);
  m_grabber.setParameterNamed("Device1_Process0_module13_Divisor", 2);
  m_grabber.setParameterNamed("Device1_Process0_module34_Value", 1);
  m_grabber.setParameterNamed("Device1_Process0_module35_Value", 1);
  m_grabber.setParameterNamed("Device1_Process0_module36_AppendNumber", 2);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_CC2output", TrgPortArea::Vcc);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_CC3output", TrgPortArea::Vcc);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_CC4output", TrgPortArea::Vcc);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_ExsyncDelay", 0);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_FlashDelay", 0);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_SoftwareTrgDeadTime", 1000);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_DebouncingTime", 0.122);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_ExsyncPolarity", TrgPortArea::LowActive);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_FlashPolarity", TrgPortArea::LowActive);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_ImgTrgInSource", TrgPortArea::InSignal0);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_ImgTrgDownscale", 1);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_Accuracy", 10);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_ImgTrgInPolarity", TrgPortArea::HighActive);
  m_grabber.setParameterNamed("Device1_Process0_TrigCam_FlashEnable", TrgPortArea::OFF);

  // * The ones that it would be nice at some point to be able to change :
  m_grabber.setPixelFormat(lima::siso_me4::Grabber::siso_px_16b);
  m_grabber.setParameterNamed("Device1_Process0_Camera_Format", CameraGrayAreaBase::DualTap12Bit);

  // * Then the ones that might (and will) be changed during the setting of the camera :
  m_grabber.setHeight(m_detector_size.getHeight());
  m_grabber.setWidth(m_detector_size.getWidth());
  m_grabber.setDeviceTimeout(static_cast<uint32_t>(m_exp_time * 5.e6)); // 5 times the exposure time.
  m_grabber.setParameterNamed(names::reorder_half_height1, m_detector_size.getHeight() >> 1); // Half of the lines only
  m_grabber.setParameterNamed(names::reorder_quat_width, m_detector_size.getWidth() >> 2); // horizontal size / 4 (since 64bits for 16bits/px)
  m_grabber.setParameterNamed(names::reorder_half_height2, m_detector_size.getHeight() >> 1);	// Again only affects half of the lines
  m_grabber.setParameterNamed(names::roi_off_x, 0);			// The ROI
  m_grabber.setParameterNamed(names::roi_width, m_detector_size.getWidth());		// The ROI
  m_grabber.setParameterNamed(names::roi_off_y, 0);			// The ROI
  m_grabber.setParameterNamed(names::roi_half_height, m_detector_size.getHeight()/2);		// The ROI (half height)
  m_grabber.setParameterNamed(names::trig_mode, TrgPortArea::GrabberControlled);
  m_grabber.setParameterNamed(names::trig_ext_sync, TrgPortArea::ON);
  m_grabber.setParameterNamed(names::trig_soft_pulse, 1);
  m_grabber.setParameterNamed(names::trig_ext_fps, 8);
  m_grabber.setParameterNamed(names::trig_ext_width, static_cast<int>(m_exp_time * 1.e6));
  m_grabber.setParameterNamed(names::trig_cc1_signal, TrgPortArea::Exsync);
}

lima::VieworksVP::Camera::~Camera()
{
  
}

// Preparing the camera's SDK to acquire frames
void
lima::VieworksVP::Camera::prepareAcq()
{
  
}
// Launches the SDK's acquisition and the m_acq_thread to retrieve frame buffers as they are ready
void
lima::VieworksVP::Camera::startAcq()
{
  
}
// Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
void
lima::VieworksVP::Camera::stopAcq()
{
  
}

// -- detector info object
void
lima::VieworksVP::Camera::getImageType(ImageType& type)
{
  
}
void
lima::VieworksVP::Camera::setImageType(ImageType type)
{
  
}

void
lima::VieworksVP::Camera::getDetectorType(std::string& type)
{
  
}
void
lima::VieworksVP::Camera::getDetectorModel(std::string& model)
{
  
}
void
lima::VieworksVP::Camera::getDetectorImageSize(Size& size)
{
  
}

// -- Buffer control object
lima::HwBufferCtrlObj*
lima::VieworksVP::Camera::getBufferCtrlObj()
{
  
}

//-- Synch control object
bool
lima::VieworksVP::Camera::checkTrigMode(TrigMode mode)
{
  
}
void
lima::VieworksVP::Camera::setTrigMode(TrigMode  mode)
{
  
}
void
lima::VieworksVP::Camera::getTrigMode(TrigMode& mode)
{
  
}

void
lima::VieworksVP::Camera::setExpTime(double  exp_time)
{
  
}
void
lima::VieworksVP::Camera::getExpTime(double& exp_time)
{
  
}

void
lima::VieworksVP::Camera::setLatTime(double  lat_time)
{
  
}
void
lima::VieworksVP::Camera::getLatTime(double& lat_time)
{
  
}

void
lima::VieworksVP::Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
  
}
void
lima::VieworksVP::Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
  
}

void
lima::VieworksVP::Camera::setNbFrames(int  nb_frames)
{
  
}
void
lima::VieworksVP::Camera::getNbFrames(int& nb_frames)
{
  
}
void
lima::VieworksVP::Camera::getNbHwAcquiredFrames(int &nb_acq_frames)
{
  
}

void
lima::VieworksVP::Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
  
}
void
lima::VieworksVP::Camera::setRoi(const Roi& set_roi)
{
  
}
void
lima::VieworksVP::Camera::getRoi(Roi& hw_roi)
{
  
}

bool
lima::VieworksVP::Camera::isBinningAvailable()
{
  return false;
}

void
lima::VieworksVP::Camera::checkBin(Bin& ioBin)
{
  
}
void
lima::VieworksVP::Camera::setBin(const Bin& iBin)
{
  
}
void
lima::VieworksVP::Camera::getBin(Bin& oBin)
{
  
}

void
lima::VieworksVP::Camera::setShutterMode(ShutterMode mode)
{
  
}
void
lima::VieworksVP::Camera::getShutterMode(ShutterMode& mode)
{
  
}

void
lima::VieworksVP::Camera::getPixelSize(double& sizex, double& sizey)
{
  
}

void
lima::VieworksVP::Camera::getStatus(Camera::Status& status)
{
  
}

// --- Acquisition interface
void
lima::VieworksVP::Camera::reset()
{
  
}
int
lima::VieworksVP::Camera::getNbHwAcquiredFrames()
{
  
}

// -- vieworks-vp specific, LIMA don't worry about it !

void
lima::VieworksVP::Camera::setTestImage(VP_test_image i_test_image)
{
  
}

void
lima::VieworksVP::Camera::getTestImage(VP_test_image &o_test_image) const
{
  
}
void
lima::VieworksVP::Camera::setDataBits(VP_data_bits i_data_bits)
{
  
}

void
lima::VieworksVP::Camera::getDataBits(VP_data_bits &o_data_bits) const
{
  
}
void
lima::VieworksVP::Camera::setLUTcontrol(VP_LUT_control i_lut)
{
  
}

void
lima::VieworksVP::Camera::getLUTcontrol(VP_LUT_control &o_lut) const
{
  
}
void
lima::VieworksVP::Camera::setAsynchronousReset(bool i_AR)
{
  
}

void
lima::VieworksVP::Camera::getAsynchronousReset(bool &o_AR) const
{
  
}
void
lima::VieworksVP::Camera::setFlatFieldCorrection(bool i_FFC)
{
  
}

void
lima::VieworksVP::Camera::getFlatFieldCorrection(bool &o_FFC) const
{
  
}
void
lima::VieworksVP::Camera::setDefectCorrection(bool i_DC)
{
  
}

void
lima::VieworksVP::Camera::getDefectCorrection(bool &o_DC) const
{
  
}
void
lima::VieworksVP::Camera::setImageInvert(bool i_II)
{
  
}

void
lima::VieworksVP::Camera::getImageInvert(bool &o_II) const
{
  
}
void
lima::VieworksVP::Camera::setHorizontalFlip(bool i_HF)
{
  
}

void
lima::VieworksVP::Camera::getHorizontalFlip(bool &o_HF) const
{
  
}
void
lima::VieworksVP::Camera::setTrigger(VP_trigger_mode i_trig)
{
  
}
void
lima::VieworksVP::Camera::getTrigger(VP_trigger_mode &o_trig) const
{
  
}
void
lima::VieworksVP::Camera::setExpSource(VP_exp_source i_exp_src)
{
  
}
void
lima::VieworksVP::Camera::getExpSource(VP_exp_source &o_exp_src) const
{
  
}
void
lima::VieworksVP::Camera::setTriggerSource(VP_trigger_source i_trig_src)
{
  
}
void
lima::VieworksVP::Camera::getTriggerSource(VP_trigger_source &o_trig_src) const
{
  
}
void
lima::VieworksVP::Camera::setTriggerPolarity(bool i_pol)
{
  
}

void
lima::VieworksVP::Camera::getTriggerPolaroty(bool &o_pol) const
{
  
}
void
lima::VieworksVP::Camera::setExpMusTime(unsigned int i_time)
{
  
}
void
lima::VieworksVP::Camera::getExpMusTime(unsigned int &o_time) const
{
  
}
void
lima::VieworksVP::Camera::setStrobeOffsetMus(unsigned int i_time)
{
  
}

void
lima::VieworksVP::Camera::getStrobeOffsetMus(unsigned int &o_time) const
{
  
}
void
lima::VieworksVP::Camera::setStrobePolarity(bool i_pol)
{
  
}

void
lima::VieworksVP::Camera::getStrobePolaroty(bool &o_pol) const
{
  
}
void
lima::VieworksVP::Camera::setAnalogGain(unsigned short i_gain)
{
  
}

void
lima::VieworksVP::Camera::getAnalogGain(unsigned short &o_gain) const
{
  
}
void
lima::VieworksVP::Camera::setAnalogOffset(unsigned char i_off)
{
  
}

void
lima::VieworksVP::Camera::getAnalogOffset(unsigned char &o_off) const
{
  
}
void
lima::VieworksVP::Camera::setFlatFieldIteration(unsigned char i_iter)
{
  
}

void
lima::VieworksVP::Camera::getFlatFieldIteration(unsigned char &o_iter) const
{
  
}
void
lima::VieworksVP::Camera::setFlatFieldOffset(unsigned short i_off)
{
  
}

void
lima::VieworksVP::Camera::getFlatFieldOffset(unsigned short &o_off) const
{
  
}

void
lima::VieworksVP::Camera::setTemperatureSP(int i_temp)
{
  
}

void
lima::VieworksVP::Camera::getTemperatureSP(int &o_temp) const
{
  
}
void
lima::VieworksVP::Camera::setPixelClock(VP_pixel_clock i_clk)
{
  
}

void
lima::VieworksVP::Camera::getPixelClock(VP_pixel_clock &o_clk) const
{
  
}
void
lima::VieworksVP::Camera::setFanStatus(bool i_bootl)
{
  
}

void
lima::VieworksVP::Camera::getFanStatus(bool &o_bool) const
{
  
}
void
lima::VieworksVP::Camera::setPeltierControl(bool i_bootl)
{
  
}

void
lima::VieworksVP::Camera::getPeltierControl(bool &o_bool) const
{
  
}

void
lima::VieworksVP::Camera::getMCUversion(std::string &o_string) const
{
  
}

void
lima::VieworksVP::Camera::getModelNumber(std::string &o_string) const
{
  
}

void
lima::VieworksVP::Camera::getFPGAversion(std::string &o_string) const
{
  
}

void
lima::VieworksVP::Camera::getSerialNumber(std::string &o_string) const
{
  
}

void
lima::VieworksVP::Camera::getCurrentTemperature(double &o_temp) const
{
  
}

void
lima::VieworksVP::Camera::getSensorTemperature(double &o_temp) const
{
  
}


void
lima::VieworksVP::Camera::setOneParam(std::string i_name, int i_value1)
{
  DEB_MEMBER_FUNCT();
  std::ostringstream	the_stream;
  std::string					the_ans;
  int									the_err_code;
  std::string					the_err_msg;

  the_stream << 's' << i_name << " " << i_value1;
  the_ans = command(the_stream.str());
  if ( (the_err_code = checkComError(the_ans, the_err_msg)) ) {
    DEB_WARNING() << "While trying to set the value of " << i_name << " to "
    << i_value1 << " got the following error code : " << the_err_code
    << " : '" << the_err_msg << "'";
  }
  if ( the_ans.compare("OK") ) {
    DEB_WARNING() << "While trying to set the value of " << i_name << " to "
    << i_value1 << ", the answer was neitehr an explicit error nor the standard OK : '"
    << the_ans << "'";
  }
}

void
lima::VieworksVP::Camera::setTwoParams(std::string i_name, int i_value1, int i_value2)
{
  DEB_MEMBER_FUNCT();
  std::ostringstream	the_stream;
  std::string					the_ans;
  int									the_err_code;
  std::string					the_err_msg;
  
  the_stream << 's' << i_name << " " << i_value1 << " " << i_value2;
  the_ans = command(the_stream.str());
  if ( (the_err_code = checkComError(the_ans, the_err_msg)) ) {
    DEB_WARNING() << "While trying to set the value of " << i_name << " to "
    << i_value1 << " got the following error code : " << the_err_code
    << " : '" << the_err_msg << "'";
  }
  if ( the_ans.compare("OK") ) {
    DEB_WARNING() << "While trying to set the value of " << i_name << " to "
    << i_value1 << ", the answer was neitehr an explicit error nor the standard OK : '"
    << the_ans << "'";
  }
}

void
lima::VieworksVP::Camera::getOneParam(std::string i_name, std::string &o_value1) const
{
  DEB_MEMBER_FUNCT();
  std::ostringstream	the_stream;
  std::string					the_ans;
  int									the_err_code;
  std::string					the_err_msg;
  
  the_stream << 'g' << i_name;
  the_ans = get_command(the_stream.str());
  if ( (the_err_code = checkComError(the_ans, the_err_msg)) ) {
    DEB_WARNING() << "While trying to get the value of " << i_name
    << " got the following error code : " << the_err_code
    << " : '" << the_err_msg << "'.\n\tWill NOT modify the value upon return,"
    << " which will stay at : " << o_value1;
    return;
  }
  std::istringstream	the_parser(the_ans);
  the_parser >> o_value1;
  return;
}


void
lima::VieworksVP::Camera::getOneParam(std::string i_name, int &o_value1) const
{
  DEB_MEMBER_FUNCT();
  std::ostringstream	the_stream;
  std::string					the_ans;
  int									the_err_code;
  std::string					the_err_msg;
  
  the_stream << 'g' << i_name;
  the_ans = get_command(the_stream.str());
  if ( (the_err_code = checkComError(the_ans, the_err_msg)) ) {
    DEB_WARNING() << "While trying to get the value of " << i_name
    << " got the following error code : " << the_err_code
    << " : '" << the_err_msg << "'.\n\tWill NOT modify the value upon return,"
    << " which will stay at : " << o_value1;
    return;
  }
  std::istringstream	the_parser(the_ans);
  the_parser >> o_value1;
  return;
}

void
lima::VieworksVP::Camera::getTwoParams(std::string i_name, int &o_value1, int &o_value2) const
{
  DEB_MEMBER_FUNCT();
  std::ostringstream	the_stream;
  std::string					the_ans;
  int									the_err_code;
  std::string					the_err_msg;
  
  the_stream << 'g' << i_name;
  the_ans = get_command(the_stream.str());
  if ( (the_err_code = checkComError(the_ans, the_err_msg)) ) {
    DEB_WARNING() << "While trying to get the value of " << i_name
    << " got the following error code : " << the_err_code
    << " : '" << the_err_msg << "'.\n\tWill NOT modify the value upon return,"
    << " which will stay at : " << o_value1;
    return;
  }
  std::istringstream	the_parser(the_ans);
  the_parser >> o_value1 >> o_value2;
  return;
}

//! Performing a command (writing the i_cmd to the serial line, then \r) and
//    returning the answer of the camera, that is the second line since the
//    first is the echo of the command.
//  Indeed we are also checking that the first line is the exact echo of what we
//    believe we have sent in.
std::string
lima::VieworksVP::Camera::command(const std::string& i_cmd)
{
  DEB_MEMBER_FUNCT();
  // Prepare the string to be sent to the camera :
  std::string the_sent_text = i_cmd;
  std::string the_answer;
  trim(the_sent_text);
  the_sent_text += "\r"; // the camera expects the command to be terminated by a '\r'.
  DEB_TRACE() << "About to send '" << the_sent_text << " to the camera's serail line";
  m_serial_line.writeReadStr(the_sent_text, the_answer, 1024, ">", false, 10);
  DEB_TRACE() << "Received the following answer : '" << the_answer << "' from the camera";
  
  // We have to split the answer in two lines, check that the first line is exactly what we believe it should be !
  std::vector<std::string>		the_lines;
  split(the_answer, '\n', the_lines, true);
  for (size_t i=0; the_lines.size() != i; ++i) {
    DEB_TRACE() << "Line " << i << " of the answer is '" << the_lines.at(i) << ",";
  }
  // Now checking that the first line is the exact echo of the request :
  trim(the_sent_text);
  if ( the_lines.at(0) != the_sent_text ) {
    DEB_WARNING() << "While communicating on the serial port to the camera, the echo line is not the sent line !";
    DEB_WARNING() << "Sent : '" << the_sent_text << "'";
    DEB_WARNING() << "Echo : '" << the_lines.at(0) << "'";
  }
  return the_lines.at(1);
}

// The caller basically takes the engagement that nothing will be «modified» on
//   the camera due to the requested command.
std::string
lima::VieworksVP::Camera::get_command(const std::string& i_cmd) const
{
  DEB_MEMBER_FUNCT();
  // Prepare the string to be sent to the camera :
  std::string the_sent_text = i_cmd;
  std::string the_answer;
  trim(the_sent_text);
  the_sent_text += "\r"; // the camera expects the command to be terminated by a '\r'.
  DEB_TRACE() << "About to send '" << the_sent_text << " to the camera's serail line";
  m_serial_line.writeReadStr(the_sent_text, the_answer, 1024, ">", false, 10);
  DEB_TRACE() << "Received the following answer : '" << the_answer << "' from the camera";
  
  // We have to split the answer in two lines, check that the first line is exactly what we believe it should be !
  std::vector<std::string>		the_lines;
  split(the_answer, '\n', the_lines, true);
  for (size_t i=0; the_lines.size() != i; ++i) {
    DEB_TRACE() << "Line " << i << " of the answer is '" << the_lines.at(i) << ",";
  }
  // Now checking that the first line is the exact echo of the request :
  trim(the_sent_text);
  if ( the_lines.at(0) != the_sent_text ) {
    DEB_WARNING() << "While communicating on the serial port to the camera, the echo line is not the sent line !";
    DEB_WARNING() << "Sent : '" << the_sent_text << "'";
    DEB_WARNING() << "Echo : '" << the_lines.at(0) << "'";
  }
  return the_lines.at(1);
}


int
lima::VieworksVP::Camera::checkComError(const std::string &i_answer)
{
  DEB_STATIC_FUNCT();
  if ( i_answer.compare(0, 8, "Error : ") ) {
    return 0;
  }
  std::string		the_code_hex=i_answer.substr(8, i_answer.npos);
  std::istringstream	the_code_st(the_code_hex);
  int						the_err_code;
  the_code_st >> std::hex >> the_err_code;
  return the_err_code;
}

int
lima::VieworksVP::Camera::checkComError(const std::string &i_answer, std::string &o_message)
{
  DEB_STATIC_FUNCT();
  int						the_err_code = checkComError(i_answer);

  switch (the_err_code) {
    case 0:
      o_message = "no error";
      break;
      
    case 0x80000481:
      o_message = "values of parameter not valid";
      break;
    case 0x80000482:
      o_message = "number of parameter is not matched";
      break;
    case 0x80000484:
      o_message = "command that does not exist";
      break;
    case 0x80000486:
      o_message = "no execution right";
      break;
      
    default:
      o_message = "undocumented error code";
      break;
  }
  return the_err_code;
}



// Stopping an acquisition, iForce : without waiting the end of frame buffer retrieval by m_acq_thread
void
lima::VieworksVP::Camera::doStopAcq(bool iImmediate)
{
  
}
// Setting the status in a thread safe manner :
void
lima::VieworksVP::Camera::setStatus(Camera::Status iStatus, bool iForce)
{
  
}


/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
