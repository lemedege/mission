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

// #include <iostream>
// #include <sys/time.h>
// #include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"
#include "utime.h"

using namespace std;

//#define PI_CAM
#ifndef PI_CAM
// use when testing code without a camera, or not on a raspberry
UCamera::UCamera(UBridge * reg)
{
  printf("No camera support - set '#define PI_CAM' in ucamera.h to enable\n");
  th1 = NULL;
  th1stop = false;
}

/** destructor */
UCamera::~UCamera()
{
  stop();
}

#endif

void UCamera::stop()
{
  th1stop = true;
  if (th1 != NULL)
    th1->join();
#ifdef raspicam_CV_LIBS
  Camera.release();
#endif
  printf("Finished, camera closed\n");  
}


#if defined PI_CAM

/** Constructor */
UCamera::UCamera(UBridge * reg)
{
  th1 = NULL;
  th1stop = false;
  imgAverage = 0;
  saveImage = false;
//   lineImage = false;
  regbot = reg;
  bool isOK = setupCamera();
  if (isOK)
  {
    const int MNL = 100;
    char date[MNL];
    char name[MNL];
    th1 = new thread(runObj, this);
    printf("Camera started OK\n");
    imTime.now();
    imTime.getForFilename(date);
    // construct filename
    snprintf(name, MNL, "log_image_%s.txt", date);
    logImg = fopen(name, "w");
    if (logImg != NULL)
    {
      UTime t;
      t.setTime(regbot->info->bootTime);
      const int MSL = 50;
      char s[MSL];
      fprintf(logImg, "%% data log started at %s\n", t.getDateTimeAsString(s));
      fprintf(logImg, "%% 1 Time [sec]\n");
      fprintf(logImg, "%% 2 Regbot time [sec]\n");
      fprintf(logImg, "%% 3 image number\n");
      fprintf(logImg, "%% 4 image file name\n");
      fflush(logImg);
    }
    else
      printf("Failed to open image logfile\n");
  }
  else
    printf("Camera setup failed\n");
}

/** destructor */
UCamera::~UCamera()
{
  stop();
  if (logImg != NULL)
    fclose(logImg);
}



/**
  * Implementation of capture and timestamp image */
timeval UCamera::capture(cv::Mat &image)
{
  timeval imageTime;
#ifdef raspicam_CV_LIBS
  Camera.grab();
  gettimeofday(&imageTime, NULL);
  image.create(Camera.getHeight(), Camera.getWidth(), CV_8UC3);
  Camera.retrieve ( image.data);
#else
  gettimeofday(&imageTime, NULL);  
#endif
  return imageTime;
}

void UCamera::run()
{
  cv::Mat im;
  cv::Mat im2;
  cv::Mat imd; // differense image
//   int lineState = 0;
//   UTime imTime, im2Time;
//   bool isOK = false;
  printf("# camera thread started\n");
  while (not th1stop)
  {
    // capture RGB image to a Mat structure
    imTime = capture(im);
    int sum = 0; // of red
    int n = 0;
    for (int row = 2; row < im.rows; row+=55)
    {
      for (int col= 2; col < im.cols; col+=15)
      {
        n++;
        cv::Vec3b pix = im.at<cv::Vec3b>(row, col);
        sum += pix.val[2]; // format is BGR, so use red
      }
//       printf("# row=%d, n=%d sum=%d, avg=%d\n", row, n, sum, sum/n);
    }
    imgAverage = sum/n;
    if (saveImage)
    {
      printf("# saving - avg=%d\n", imgAverage);
      saveImageToFlash(im);
//       cv::imwrite("savedImage.png", im);
      saveImage = false;
      printf("Image saved\n");
    }
  }  
}


void UCamera::saveImageToFlash(cv::Mat& im)
{
  const int MNL = 100;
  char date[MNL];
  char name[MNL] = "savedImage.png";
  saveImage = false;
  // use date in filename 
  // get date as string
  printf("# Trying to save\n");
//   float imgTime = regbot->info.getTime();
  imTime.getForFilename(date);
  // construct filename
  snprintf(name, MNL, "image_%s_%03d.png", date, imageNumber);
  // convert to RGB
  cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
  // make PNG option - compression 9
  vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  // save image
  cv::imwrite(name, im, compression_params);
  // debug message
  printf("Saved image to: %s\n", name);
  if (logImg != NULL)
  { // save to image logfile
    fprintf(logImg, "%ld.%03ld %.3f %d '%s'\n", imTime.getSec(), imTime.getMilisec(), regbot->info->regbotTime, imageNumber, name);
    fflush(logImg);
  }
  imageNumber++;
}



// Read an image from the camera
// static cv::Mat UCamera::GetImageFromCamera(cv::VideoCapture& camera)
// {
//   cv::Mat frame;
//   camera >> frame;
//   return frame;
// }

// bool UCamera::imageLineProcess(cv::Mat * imOrg, cv::Mat * im2Org, UTime imTime, UTime im2Time)
// {
//   int w = imOrg->cols;
//   int h = imOrg->rows;
//   int stride = imOrg->step;
//   cv::Mat im;
//   cv::Mat im2;
//   cv::Mat dImg(h, w, CV_8UC1);
//   uint8_t * dst = dImg.data;
//   const int limit = 0;
//   printf("# image is %d x %d (stride=%d)\n", h, w, stride);
//   cv::cvtColor(*imOrg, im, cv::COLOR_RGB2BGR);
//   cv::cvtColor(*im2Org, im2, cv::COLOR_RGB2BGR);
//   int cmax = 0;
//   for (int row = h - h; row < h; row++)
//   {
//     dst = dImg.data + row * dImg.step;
//     uchar * px1 = im.data + row * stride;
//     uchar * px2 = im2.data + row * stride;
//     int v;
//     for (int col= 0; col < w; col++)
//     {
// //       cv::Vec3b pix1 = im->at<cv::Vec3b>(row, col);
// //       cv::Vec3b pix2 = im2->at<cv::Vec3b>(row, col);
//       // look in the red channel only (as line is red)
//       v = (px1[2] - px2[2] + px2[0] - px1[0]);
//       if (v < limit)
//         *dst = 0;
//       else if (v > 255+limit)
//         *dst = 255-limit;
//       else
//         *dst = v - limit;
//       if (*dst > cmax)
//         cmax = *dst;
//       dst++;
//       px2 += 3;
//       px1 += 3;
//     }
//     //       printf("# row=%d, n=%d sum=%d, avg=%d\n", row, n, sum, sum/n);
//   }
//   printf("# max value is %d\n", cmax);
//   cv::imwrite("savedDiffImgRaw1.png", im);
//   cv::imwrite("savedDiffImg.png", dImg);
//   cv::imwrite("savedDiffImgRaw2.png", im2);
//   // save array of points
//   cv::Mat d2, dla, th, th2;
//   cv::pyrDown(dImg, d2);
//   cv::Laplacian(d2, dla,CV_8U);
//   cv::imwrite("savedLaplacian.png", dla);
//   // skip the laplacian
//   cv::threshold(d2, th, cmax/20+5, 255, 0);
//   cv::imwrite("savedThreshold.png", th);
//   //
//   if (false)
//   {
//     int erosion_size = 1;
//     cv::Mat element = cv::getStructuringElement(0, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
//                                                   cv::Point( erosion_size, erosion_size) );  
//     cv::erode(th, th2, element);
//     cv::imwrite("savedThreshold2.png", th2);
//   }
//   else
//     th2 = th;
//   //
//   std::vector<cv::Point> linePoints;
//   linePoints.reserve(1000);
//   int rowStart = (th2.rows * 700) / dImg.rows;
//   for (int row = rowStart; row < th2.rows; row ++)
//   { // skip the first rows
//     dst = th2.data + row * th2.step;
//     for (int col= 0; col < th2.cols; col++)
//     {
//       if (*dst++)
//       {
//         linePoints.push_back(cv::Point(col, row));
//       }
//     }
// //     printf("# row %d pixels %d\n", row, linePoints.size()); 
//   }
//   printf("# found pixels %d\n", linePoints.size());
//   // first guess
//   {
//     U2Dline line1;
//     line1.set(linePoints);
//     cv::Point p1(0, cvRound(2.0 * line1.getY(0)));
//     cv::Point p2(th2.cols*2 - 1, cvRound(2.0 * line1.getY(th2.cols - 1)));
//     cv::line(im , p1, p2, cv::Scalar(255,0,0), 1, 8 );
//   }
//   // improve line
//   U2Dline mainLine;
//   mainLine = findMainLine(linePoints);
//   {
//     // get x,y points across image (with twice the resolution)
//     cv::Point p1(0, cvRound(2.0 * mainLine.getY(0)));
//     cv::Point p2(th2.cols*2 - 1, cvRound(2.0 * mainLine.getY(th2.cols - 1)));
//     printf("# main line from %d,%d to %d,%d\n", p1.x, p1.y, p2.x, p2.y);
//     cv::line(im , p1, p2, cv::Scalar(0,255,0), 3, 8 );
//   }
//   cv::imwrite("savedDiffImgRaw2ann.png", im);
//   //
//   if (false)
//   {
//     printf("# image max value is %d\n", cmax);
//     cv::threshold(dImg, th, cmax/10, 255, 0);
//     cv::imwrite("savedThreshold.png", th);
//     std::vector<cv::Vec2f> lines;
//     cv::HoughLines(th, lines, 2, 2*M_PI/180, 600);
//     printf("# hough found %d lines\n", lines.size());
//     int c = 255;
//     int lw = 3;
//     for( size_t i = 0; i < lines.size(); i++ )
//     {
//       float rho = lines[i][0];
//       float theta = lines[i][1];
//       printf("# line %d is rho=%g, theta=%g\n", i, rho, theta);
//       double a = cos(theta), b = sin(theta);
//       double x0 = a*rho, y0 = b*rho;
//       cv::Point pt1(cvRound(x0 + 2000*(-b)),
//                 cvRound(y0 + 2000*(a)));
//       cv::Point pt2(cvRound(x0 - 2000*(-b)),
//                 cvRound(y0 - 2000*(a)));
//       cv::line(im , pt1, pt2, cv::Scalar(c,255- c,0), lw, 8 );
//       if (i > 0)
//         break;
//       if (c > 30)
//         c-=10;
//       if (lw > 1)
//         lw--;
//     }
//     cv::imwrite("savedDiffImgRaw2ann.png", im);
//     printf("# image line done\n");
//     // detect edges
//     cv::Mat edg, blr;
//   //   cv::blur(dImg, blr, cv::Size(3,3));
//   //   cv::Canny(blr, edg, 50, 100);
//   //   cv::imwrite("savedBlur.png", blr);
//   //   cv::imwrite("savedCanny.png", edg);
//   }
//   return true;
// }
// 
// 
// U2Dline UCamera::findMainLine(std::vector<cv::Point> &linePoints)
// {
//   U2Dline line;
//   line.set(linePoints);
//   std::vector<cv::Point> pNear;
//   pNear.reserve(500);
//   float d;
//   line.print((char*)"main line");
//   for (int i = 0; i < (int)linePoints.size(); i++)
//   { // test if they are near, then again
//     // fit line and iterate until fine line
//     // maybe prefer higher rather than læower distance
//     cv::Point v = linePoints.at(i);
//     d = line.distanceSigned(v.x, v.y);
//     if (d < 1 and d > -50)
//     {
//       pNear.push_back(v);
// //       printf("# ok at %d,%d (%g pixels)\n", v.x, v.y, d);
//     }
//   }
//   printf("# reduced line points from %d to %d\n", linePoints.size(), pNear.size());
//   line.set(pNear);
//   return line;
// }


#endif
 
  
