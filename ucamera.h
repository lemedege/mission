/**********************************************************
 * Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es) 
 * Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 * This software is released under BSD license as expressed below
 * -------------------------------------------------------------------
 * Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es) 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *   must display the following acknowledgement:
 *   
 *   This product includes software developed by the Ava group of the University of Cordoba.
 *   
 * 4. Neither the name of the University nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************/


/***************************************************************************
*   Copyright (C) 2016 by DTU (Christian Andersen)                        *
*   jca@elektro.dtu.dk                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License as        *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Lesser General Public      *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#ifndef UCAMERA_H
#define UCAMERA_H


#include <iostream>
#include <sys/time.h>
#include <thread>
// #include <mutex>
#include <opencv2/core/core.hpp>
#include "urun.h"
#include "ubridge.h"
#include "utime.h"
#include "u2dline.h"
// this should be defined in the CMakeList.txt ? or ?
#ifdef raspicam_CV_LIBS
#define PI_CAM
#ifdef PI_CAM
#include <raspicam/raspicam.h>
#endif
#endif

//#define PI_CAM

using namespace std;

/**
 * The camera class has the functions
 * to open, close, configure and
 * capture images from the raspberry pi
 * camera */
class UCamera : public URun
{ // raw camera functions 
public:
  int imgAverage;
  // flag to save an image to disk
  bool saveImage;  

public:
  /** Constructor */
  UCamera(UBridge * reg);
  /** destructor */
  ~UCamera();
  void stop();

private:
  // pointer to regbot interface
  UBridge * regbot;
  // read thread handle
//   thread * th1;
//   // set true to stop thread
//   bool th1stop;
  // image saved number
  int imageNumber = 0;
  // time image was taken
  UTime imTime, im2Time;
  // logfile for images
  FILE * logImg = NULL;
  
#ifdef PI_CAM
protected:
  /**
   * The raw raspberry pi camera device, as defined by the 
   * Ava Group of the University of Cordoba */
#ifdef raspicam_CV_LIBS
  raspicam::RaspiCam Camera;
#else
  cv::VideoCapture camera(0);
#endif
  /**
   * Save image to flashdisk */
  void saveImageToFlash(cv::Mat& im);
  
  /** do some image processing
   * \param im is first image with no line
   * \param im2 is second image with laser line
   * \param imTime is tiomestamp of first image,
   * \param im2Time is tiomestamp of second image,
   * \returns true if line found. */
//  bool imageLineProcess(cv::Mat * im, cv::Mat * im2, UTime imTime, UTime im2Time);
 /**
  * Ransac lines */
//  U2Dline findMainLine(std::vector<cv::Point> &linePoints);
 
#ifdef raspicam_CV_LIBS
 
  /** text support for exposure */
  raspicam::RASPICAM_EXPOSURE getExposureFromString ( string str )
  {
    if ( str=="OFF" ) return raspicam::RASPICAM_EXPOSURE_OFF;
    if ( str=="AUTO" ) return raspicam::RASPICAM_EXPOSURE_AUTO;
    if ( str=="NIGHT" ) return raspicam::RASPICAM_EXPOSURE_NIGHT;
    if ( str=="NIGHTPREVIEW" ) return raspicam::RASPICAM_EXPOSURE_NIGHTPREVIEW;
    if ( str=="BACKLIGHT" ) return raspicam::RASPICAM_EXPOSURE_BACKLIGHT;
    if ( str=="SPOTLIGHT" ) return raspicam::RASPICAM_EXPOSURE_SPOTLIGHT;
    if ( str=="SPORTS" ) return raspicam::RASPICAM_EXPOSURE_SPORTS;
    if ( str=="SNOW" ) return raspicam::RASPICAM_EXPOSURE_SNOW;
    if ( str=="BEACH" ) return raspicam::RASPICAM_EXPOSURE_BEACH;
    if ( str=="VERYLONG" ) return raspicam::RASPICAM_EXPOSURE_VERYLONG;
    if ( str=="FIXEDFPS" ) return raspicam::RASPICAM_EXPOSURE_FIXEDFPS;
    if ( str=="ANTISHAKE" ) return raspicam::RASPICAM_EXPOSURE_ANTISHAKE;
    if ( str=="FIREWORKS" ) return raspicam::RASPICAM_EXPOSURE_FIREWORKS;
    return raspicam::RASPICAM_EXPOSURE_AUTO;
  }
  /** text support for white balance */
  raspicam::RASPICAM_AWB getAwbFromString ( string str ) 
  {
    if ( str=="OFF" ) return raspicam::RASPICAM_AWB_OFF;
    if ( str=="AUTO" ) return raspicam::RASPICAM_AWB_AUTO;
    if ( str=="SUNLIGHT" ) return raspicam::RASPICAM_AWB_SUNLIGHT;
    if ( str=="CLOUDY" ) return raspicam::RASPICAM_AWB_CLOUDY;
    if ( str=="SHADE" ) return raspicam::RASPICAM_AWB_SHADE;
    if ( str=="TUNGSTEN" ) return raspicam::RASPICAM_AWB_TUNGSTEN;
    if ( str=="FLUORESCENT" ) return raspicam::RASPICAM_AWB_FLUORESCENT;
    if ( str=="INCANDESCENT" ) return raspicam::RASPICAM_AWB_INCANDESCENT;
    if ( str=="FLASH" ) return raspicam::RASPICAM_AWB_FLASH;
    if ( str=="HORIZON" ) return raspicam::RASPICAM_AWB_HORIZON;
    return raspicam::RASPICAM_AWB_AUTO;
  }
#endif
public:
  /** Capture an image and load image to cv::Mat structure
   * \param image is the destination for the image
   * \returns timestamp when the image was grabbed. */
  timeval capture(cv::Mat &image);
  /**
   * Configure camera */
  bool setupCamera()
  { // image should be a multible of 320x240
    // or the image will be cropped 
    int w = (2592/640)*320; // (2592/320)*320; // 320*5; // 
    int h = (1992/480)*240; // (1992/240)*240; // 240*3; //
    printf("image size %dx%d\n", h, w);
    
#ifdef raspicam_CV_LIBS    
    Camera.setWidth(w);
    Camera.setHeight(h);
    Camera.setExposure(getExposureFromString ("AUTO"));
    Camera.setVideoStabilization(false);
    Camera.setAWB(getAwbFromString("AUTO"));
    Camera.setMetering(raspicam::RASPICAM_METERING_SPOT);
    cout<<"Connecting to camera"<<endl;
    if ( !Camera.open() ) {
      cerr<<"Error opening camera"<<endl;
      return false;
    }
    for (int i = 0; i < 30; i++)
      // just to make sure camera settins har reached steady state
      Camera.grab();
    cout<<"Connected to pi-camera ='"<<Camera.getId() <<
          "' bufs="<<Camera.getImageBufferSize( )<<" bytes\r\n";
#endif
    return true;
  }
  /**
   * OpenCV image capture */
//   static cv::Mat GetImageFromCamera(cv::VideoCapture& camera);
  /**
   * method to do the image analysis - see ucamera.cpp */
  void run();
#endif
};

#endif
