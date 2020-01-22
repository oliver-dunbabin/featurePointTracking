// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
// California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
//
// Jevois Sample module modified by Oliver Dunbabin. Property of Purl (Pennstate University Research Lab).
//
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */

#include <jevois/Core/Module.H>
#include <jevois/Image/RawImageOps.H>
#include <jevois/Debug/Timer.H>
#include <string>
#include <chrono>
#include <stdint.h>

#include <linux/videodev2.h>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/base.hpp>
//#include "klargestheap.h"
#include "sortimgbin.h"

#define SERIAL_SYNC1 0xaa
#define SERIAL_SYNC2 0x44
#define JEVOISMSGID 0x96
#define NUMCORNERS 100
#define NUMPERBIN 1

// Message format
struct msg_header {
    char sync1;                     // First message sync byte
    char sync2;                     // Second message sync byte
    unsigned char messageID;        // ID of Harris Corner message (set to any int)
    unsigned char csum;             // Message checksum
    uint32_t messageSize;           // Size of message being received
}__attribute__((packed));

struct harrisMessageFP {
    harrisMessageFP()   {harrisHeader.sync1 = SERIAL_SYNC1;
                        harrisHeader.sync2 = SERIAL_SYNC2;
                        harrisHeader.messageID = JEVOISMSGID;}
    msg_header harrisHeader;
    uint64_t time;                                      // Time frame was grabbed (ms since epoch)
    uint16_t imageWidth;                                // Width of input frame
    uint16_t imageHeight;                               // Height of input frame
    uint16_t fpCoord[NUMCORNERS*NUMPERBIN][2];          // Harris feature point coordinate (col * row)
    unsigned char fpVal[NUMCORNERS*NUMPERBIN];          // Harris feature point value
    /*vector<vector<pixel_val>> fpData;*/
}__attribute__((packed));


// Checksum calculation
unsigned char calculateCheckSum(unsigned char *buf, int byteCount, int index){
    unsigned char csum = 0;
    for (int i = index; i < byteCount; i++){
        csum += buf[i];
    }
    return csum;
}

string encodeSerialMsg(char *buf, int byteCount){
    string msg;
    for (int i = 0; i < byteCount; i++){
        msg += buf[i];
    }
    return msg;
}

// Parameters for our module:
static jevois::ParameterCategory const ParamCateg("Harris Corner Options");


//! Parameter \relates HarrisCorner
JEVOIS_DECLARE_PARAMETER(blockSize, int, "Neighbourhood size (NxN window around each pixel)", 2, ParamCateg);
//! Parameter \relates HarrisCorner
JEVOIS_DECLARE_PARAMETER(aperture, int, "Aperture size for the Sobel operator", 3, ParamCateg);
//! Parameter \relates HarrisCorner
JEVOIS_DECLARE_PARAMETER(harrisParam, double, "Harris parameter", 0.04, ParamCateg);
//! Parameter \relates HarrisCorner
JEVOIS_DECLARE_PARAMETER(boarder_width, unsigned int, "Pixels excluded around boarder of image", 10, ParamCateg);
//! Parameter \relates HarrisCorner
JEVOIS_DECLARE_PARAMETER(separation, unsigned int, "Pixel separation between identified corners", 8, ParamCateg);
//! Parameter \relates HarrisCorner
JEVOIS_DECLARE_PARAMETER(num_fp, unsigned int, "Number of corners identified in an image", NUMCORNERS, ParamCateg);




//! JeVois HarrisCorner
/*! This module executes the Harris corner detection algorithm.

    @author Oliver Dunbabin

    @videomapping GREY 640 480 28.5 YUYV 640 480 28.5 OliverDunbabin HarrisCorner
    @email otd1@psu.edu
    @copyright Copyright (C) 2019 by Oliver Dunbabin
    @distribution Unrestricted
    @restrictions None
    \ingroup modules */
class HarrisCorner : public jevois::Module,
                     public jevois::Parameter<blockSize, aperture, harrisParam,
                            boarder_width, separation, num_fp>
{
  public:
    //! Default base class constructor ok
    using jevois::Module::Module;

    //! Virtual destructor for safe inheritance
    virtual ~HarrisCorner() { }

    //############################################################################################//
    //! Processing function with Host - NO VIDEO OUTPUT                                           //
    //############################################################################################//
    virtual void process(jevois::InputFrame && inframe) override
    {
      // Create message to send over serial
      harrisMessageFP harrisfp;

      //static jevois::Timer timer("processing",60,LOG_DEBUG);
      // Wait for next available camera image:
      jevois::RawImage inimg = inframe.get();

      // Time frame was grabbed
      chrono::milliseconds time_ms = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
      harrisfp.time = time_ms.count();

      harrisfp.imageHeight = inimg.height;
      harrisfp.imageWidth  = inimg.width;

      // Convert to OpenCV grayscale
      cv::Mat grayimg = jevois::rawimage::convertToCvGray(inimg);

      // Let camera know we are done processing input image
      inframe.done();

      // Compute Harris corners directly into output image:
      cv::Mat corners32F, corners8U_norm;
      corners32F = cv::Mat::zeros(inimg.width,inimg.height,CV_32FC1);
      cv::cornerHarris(grayimg, corners32F, blockSize::get(), aperture::get(), harrisParam::get()); // Run Harris Corner algorithm to get intensity map
      cv::normalize(corners32F,corners8U_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat());


#ifdef KLARGESTHEAP_H
      int num_corners = num_fp::get();
      int boarder = boarder_width::get();
      kLargestHeap corners(corners8U_norm,num_corners,boarder);
      for (int i = 0; i<(corners8U_norm.cols-2*boarder)*(corners8U_norm.rows-2*boarder); i++)
          corners.swapRoot(corners8U_norm,i);

      std::string ser_string;
      for (int i = 0; i < num_corners; i++){
            ser_string += to_string((corners.getHeap())[i].harrisVal) + " " + to_string((corners.getHeap())[i].pixLoc.row) + " " + to_string((corners.getHeap())[i].pixLoc.col) + " ";
      }

      finish = chrono::high_resolution_clock::now();
      time_ms = chrono::duration_cast<chrono::milliseconds>(finish-start);
      dt = time_ms.count();
      harrisfp.timeF = dt;

      string harrismsg = to_string(harrisfp.messageID) + " " + to_string(harrisfp.timeI) + " " + to_string(harrisfp.timeF) + " " + to_string(harrisfp.imageWidth) + " " + to_string(harrisfp.imageHeight) + " " + ser_string;

      jevois::Module::sendSerial(harrismsg);

#else
      unsigned int num_corners = num_fp::get();
      unsigned int pix_separation = separation::get();
      unsigned int boarder = boarder_width::get();
      sortImgBin corners(corners8U_norm,num_corners,boarder,pix_separation, NUMPERBIN);

      /*harrisfp.fpData = corners.getFp();
      std::string ser_string;
      vector<vector<pixel_val>>::const_iterator row;
      vector<pixel_val>::const_iterator col;
      for (row = harrisfp.fpData.begin(); row != harrisfp.fpData.end(); ++row)
      {
          for (col = row->begin(); col != row->end(); ++col)
              ser_string += to_string((*col).harrisVal) + " " + to_string((*col).pixLoc.row) + " " + to_string((*col).pixLoc.col) + " ";
      }
      // Time the corners were extracted from the image
      finish = chrono::high_resolution_clock::now();
      time_ms = chrono::duration_cast<chrono::milliseconds>(finish-start);
      dt = time_ms.count();
      harrisfp.timeF = dt;

      string harrismsg = to_string(harrisfp.harrisHeader.messageID) + " " + to_string(harrisfp.timeI) + " " + to_string(harrisfp.timeF) + " " + to_string(harrisfp.imageWidth) + " " + to_string(harrisfp.imageHeight) + " " + ser_string + " ";
      string header = to_string(SERIAL_SYNC1) + to_string(SERIAL_SYNC2) + to_string(JEVOISMSGID) + to_string(harrismsg.length());
      */

      corners.getFp(harrisfp.fpCoord, harrisfp.fpVal);

      harrisfp.harrisHeader.messageSize = sizeof(struct harrisMessageFP);
      int byteCount         = harrisfp.harrisHeader.messageSize;
      int headerSize        = sizeof(struct msg_header);
      int index             = headerSize;
      unsigned char csum    = calculateCheckSum((unsigned char *)&harrisfp, byteCount, index);
      harrisfp.harrisHeader.csum = csum;
      string harrismsg = encodeSerialMsg((char *)&harrisfp, byteCount);
      jevois::Module::sendSerial(harrismsg);

#endif
    }

    //############################################################################################//
    //! Processing function with Host - VIDEO OUTPUT over USB                                     //
    //############################################################################################//
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
      // Create message to send over serial
      harrisMessageFP harrisfp;

      // Wait for next available camera image:
      jevois::RawImage inimg = inframe.get();

      // Time frame was grabbed
      chrono::milliseconds time_ms = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
      harrisfp.time = time_ms.count();

      harrisfp.imageHeight = inimg.height;
      harrisfp.imageWidth  = inimg.width;

      // Convert to OpenCV grayscale
      cv::Mat grayimg = jevois::rawimage::convertToCvGray(inimg);

      // Let camera know we are done processing input image
      inframe.done();

      // Wait for an image from our gadget driver into which we will put our results. Require that it must have same
      // image size as the input image, and greyscale pixels:
      jevois::RawImage outimg = outframe.get();
      outimg.require("output", inimg.width, inimg.height, V4L2_PIX_FMT_YUYV);

      // Compute Harris corners directly into output image:
      cv::Mat corners32F, corners8U_norm;
      //tmp = grayimg(cv::Rect(boarder-1,boarder-1,inimg.width-2*boarder,inimg.height-2*boarder));
      corners32F = cv::Mat::zeros(inimg.width,inimg.height,CV_32FC1);
      cv::cornerHarris(grayimg, corners32F, blockSize::get(), aperture::get(), harrisParam::get()); // Run Harris Corner algorithm to get intensity map
      cv::normalize(corners32F,corners8U_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat());

      jevois::rawimage::convertCvGRAYtoRawImage(grayimg,outimg,1);


#ifdef KLARGESTHEAP_H
      int num_corners = num_fp::get();
      //int pix_separation = separation::get();
      int boarder = boarder_width::get();
      // Draw high intensity points (corners) on image
      kLargestHeap corners(corners8U_norm,num_corners,boarder);
      for (int i = 0; i<(corners8U_norm.cols-2*boarder)*(corners8U_norm.rows-2*boarder); i++)
          corners.swapRoot(corners8U_norm,i);

      std::string ser_string;
      for (int i = 0; i < num_corners; i++){
            ser_string += to_string((corners.getHeap())[i].harrisVal) + " " + to_string((corners.getHeap())[i].pixLoc.row) + " " + to_string((corners.getHeap())[i].pixLoc.col) + " ";
            jevois::rawimage::drawCircle(outimg, (corners.getHeap()[i]).pixLoc.col, (corners.getHeap()[i]).pixLoc.row, 3, 1, jevois::yuyv::White);
      }

      finish = chrono::high_resolution_clock::now();
      time_ms = chrono::duration_cast<chrono::milliseconds>(finish-start);
      dt = time_ms.count();
      harrisfp.timeF = dt;

      string harrismsg = to_string(harrisfp.messageID) + " " + to_string(harrisfp.timeI) + " " + to_string(harrisfp.timeF) + " " + to_string(harrisfp.imageWidth) + " " + to_string(harrisfp.imageHeight) + " " + ser_string;

      jevois::Module::sendSerial(harrismsg);

#else
      unsigned short num_corners = num_fp::get();
      //int pix_separation = separation::get();
      unsigned short boarder = boarder_width::get();
      unsigned short pix_separation = separation::get();
      sortImgBin corners(corners8U_norm,num_corners,boarder,pix_separation, NUMPERBIN);

      /*harrisfp.fpData = corners.getFp();
      vector<vector<pixel_val>>::const_iterator row;
      vector<pixel_val>::const_iterator col;
      std::string ser_string;
      for (row = harrisfp.fpData.begin(); row != harrisfp.fpData.end(); ++row)
      {
          for (col = row->begin(); col != row->end(); ++col){
              jevois::rawimage::drawCircle(outimg, (*col).pixLoc.col, (*col).pixLoc.row, 3, 1, jevois::yuyv::White);
              ser_string += to_string((*col).harrisVal) + " " + to_string((*col).pixLoc.row) + " " + to_string((*col).pixLoc.col) + " ";
          }
      }

      finish = chrono::high_resolution_clock::now();
      time_ms = chrono::duration_cast<chrono::milliseconds>(finish-start);
      dt = time_ms.count();
      harrisfp.timeF = dt;

      string harrismsg = to_string(harrisfp.messageID) + " " + to_string(harrisfp.timeI) + " " + to_string(harrisfp.timeF) + " " + to_string(harrisfp.imageWidth) + " " + to_string(harrisfp.imageHeight) + " " + ser_string;
      */
      corners.getFp(harrisfp.fpCoord, harrisfp.fpVal);

      for (int i = 0; i < NUMCORNERS*NUMPERBIN; i++){
          jevois::rawimage::drawCircle(outimg, harrisfp.fpCoord[i][0], harrisfp.fpCoord[i][1], 3, 1, jevois::yuyv::White);
      }

      harrisfp.harrisHeader.messageSize = sizeof(struct harrisMessageFP);
      int byteCount         = harrisfp.harrisHeader.messageSize;
      int headerSize        = sizeof(struct msg_header);
      int index             = headerSize;
      unsigned char csum    = calculateCheckSum((unsigned char *)&harrisfp, byteCount, index);
      harrisfp.harrisHeader.csum = csum;
      string harrismsg = encodeSerialMsg((char *)&harrisfp, byteCount);
      jevois::Module::sendSerial(harrismsg);

#endif

      // Send the output image with our processing results to the host over USB:
      outframe.send(); // NOTE: optional here, outframe destructor would call it anyway
    }
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(HarrisCorner);
