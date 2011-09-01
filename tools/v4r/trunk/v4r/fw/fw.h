/***************************************************************************
 *   Copyright (C) 2009 by Markus Bader                                    *
 *   bader@acin.tuwien.ac.at                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/**
 * @file fw
 * @author Markus Bader
 * @version 0.1
 * @date 23. August 2010
 * @brief libdc1394 abstraction to capture multible camera images syncronized
 * @see libdc1394
 **/

#ifndef FW_H
#define FW_H

#include <sys/time.h>
#include <vector>
#include <opencv/cv.h>
#include <dc1394/dc1394.h>


namespace V4R {


class FW {
public:

    static const int FW_OK = 0;
    static const int FW_ERROR = 1;
    static const int FEATURE_BRIGHTNESS = (int) DC1394_FEATURE_BRIGHTNESS;
    static const int FEATURE_EXPOSURE = (int) DC1394_FEATURE_EXPOSURE;
    static const int FEATURE_SHARPNESS = (int) DC1394_FEATURE_SHARPNESS;
    static const int FEATURE_WHITE_BALANCE = (int) DC1394_FEATURE_WHITE_BALANCE;
    static const int FEATURE_HUE = (int) DC1394_FEATURE_HUE;
    static const int FEATURE_SATURATION = (int) DC1394_FEATURE_SATURATION;
    static const int FEATURE_GAMMA = (int) DC1394_FEATURE_GAMMA;
    static const int FEATURE_SHUTTER = (int) DC1394_FEATURE_SHUTTER;
    static const int FEATURE_GAIN = (int) DC1394_FEATURE_GAIN;
    static const int FEATURE_IRIS = (int) DC1394_FEATURE_IRIS;
    static const int FEATURE_FOCUS = (int) DC1394_FEATURE_FOCUS;
    static const int FEATURE_TEMPERATURE = (int) DC1394_FEATURE_TEMPERATURE;
    static const int FEATURE_TRIGGER = (int) DC1394_FEATURE_TRIGGER;
    static const int FEATURE_TRIGGER_DELAY = (int) DC1394_FEATURE_TRIGGER_DELAY;
    static const int FEATURE_WHITE_SHADING = (int) DC1394_FEATURE_WHITE_SHADING;
    static const int FEATURE_FRAME_RATE = (int) DC1394_FEATURE_FRAME_RATE;
    static const int FEATURE_ZOOM = (int) DC1394_FEATURE_ZOOM;
    static const int FEATURE_PAN = (int) DC1394_FEATURE_PAN;
    static const int FEATURE_TILT = (int) DC1394_FEATURE_TILT;
    static const int FEATURE_OPTICAL_FILTER = (int) DC1394_FEATURE_OPTICAL_FILTER;
    static const int FEATURE_CAPTURE_SIZE = (int) DC1394_FEATURE_CAPTURE_SIZE;
    static const int FEATURE_CAPTURE_QUALITY = (int) DC1394_FEATURE_CAPTURE_QUALITY;

    static const int BAYER_METHOD_NEAREST = (int) DC1394_BAYER_METHOD_NEAREST;
    static const int BAYER_METHOD_SIMPLE = (int) DC1394_BAYER_METHOD_SIMPLE;
    static const int BAYER_METHOD_BILINEAR = (int) DC1394_BAYER_METHOD_BILINEAR;
    static const int BAYER_METHOD_HQLINEAR = (int) DC1394_BAYER_METHOD_HQLINEAR;
    static const int BAYER_METHOD_DOWNSAMPLE = (int) DC1394_BAYER_METHOD_DOWNSAMPLE;
    static const int BAYER_METHOD_EDGESENSE = (int) DC1394_BAYER_METHOD_EDGESENSE;
    static const int BAYER_METHOD_VNG = (int) DC1394_BAYER_METHOD_VNG;
    static const int BAYER_METHOD_AHD = (int) DC1394_BAYER_METHOD_AHD;


    static const int BAYER_FILTER_RGGB = (int) DC1394_COLOR_FILTER_RGGB;
    static const int BAYER_FILTER_GBRG = (int) DC1394_COLOR_FILTER_GBRG;
    static const int BAYER_FILTER_GRBG = (int) DC1394_COLOR_FILTER_GRBG;
    static const int BAYER_FILTER_BGGR = (int) DC1394_COLOR_FILTER_BGGR;

    /**
    * Constructor \n
    * @param modeNr
    * @param fpsNr
    * @param printModeSelection on true it will print all available modes
    * @post init
    */
    FW ();
    /**
    * Constructor \n
    * Using this constructor no cameras are involved
    * @param rFileList
    * @post init
    */
    FW (const std::vector< std::vector<std::string> > &rFileList);
    /**
    * Constructor which calles init \n
    * @param modeNr
    * @param fpsNr
    * @param cameraGuids guids which defines the order of the cameras \n
    * if you like to capture only one camera you have to fill the vector with your wanted guid
    * @param printModeSelection on true it will print all available modes
    * @see readCameraGuids
    */
    FW ( int modeNr, int fpsNr, std::vector<uint64_t> cameraGuids = std::vector<uint64_t>(0), bool printModeSelection = true);
    /**
    * Destructor
    **/
    ~FW();
    
    /**
    * Returns the guids of the cameras on the bus
    * @return vector with camera guids
    */
    std::vector<uint64_t> readCameraGuids();
    /**
    * Captures mutliple images simultaneous in defined format\n
    * the images are copied
    * @param rImages vector with preallocated images in the correct size and depth
    * @param pSaveToFolders if pFolder != NULL ist will store the captured images there
    * @see getTimeLastFrame
    * @return zero on success
    */
    int capture ( const std::vector<IplImage*> &rImages, const std::vector<std::string> *pSaveToFolders = NULL);
    /**
    * Captures mutliple images simultaneous in defined format\n
    * the images are NOT copied
    * @param rImages vector with preallocated image headers in the correct size and depth
    * @post enqueue
    * @see getTimeLastFrame
    * @return zero on success
    */
    int dequeue ( const std::vector<IplImage*> &rImageHeaders, const std::vector<std::string> *pSaveToFolders = NULL );
    /**
    * Captures a single images of a defined camera\n
    * the images are NOT copied
    * @param pImageHeaders preallocated image header 
    * @post enqueue
    * @see getTimeLastFrame
    * @return zero on success
    */
    int dequeue ( IplImage *pImageHeaders, int cameraIdx, const std::string *pSaveToFolders = NULL );
    /**
    * dequeues all images from all cameras\n
    * @pre dequeue
    */
    int enqueue ( );
    /**
    * dequeues a images from camerasIdx\n
    * @param cameraIdx
    * @pre dequeue
    */
    int enqueue (int cameraIdx );
    /**
    * conterts rgb format into yuv422
    * @param pSrc
    * @param pDes
    */
    void rgbToYUV422(IplImage *pSrc, IplImage *pDes);
    /**
    * conterts bayer format into RGB/BGR -> see filter
    * @param pSrc
    * @param pDes
    * @param method
    * BAYER_METHOD_NEAREST \n
    * BAYER_METHOD_SIMPLE \n
    * BAYER_METHOD_BILINEAR \n
    * BAYER_METHOD_HQLINEAR \n
    * BAYER_METHOD_DOWNSAMPLE \n
    * BAYER_METHOD_EDGESENSE \n
    * BAYER_METHOD_VNG \n
    * BAYER_METHOD_AHD \n
    * @param filter see the libdc1394 dc1394color_filter_t description for more information
    * BAYER_FILTER_RGGB --> RGB\n
    * BAYER_FILTER_GBRG\n
    * BAYER_FILTER_GRBG --> BGR \n
    * BAYER_FILTER_BGGR\n
    */
    void bayerTo(IplImage *pSrc, IplImage *pDes, int method = BAYER_METHOD_SIMPLE, int filter = BAYER_FILTER_GRBG);
    /**
    * Image Width
    */
    unsigned int getWidth() {
        return mWidth;
    }
    /**
    * Image Height
    */
    unsigned int getHeight() {
        return mHeight;
    }
    /**
    * Number of cameras
    */
    unsigned int getNrOfCameras() {
        return mNrOFCameras;
    }
    /**
    * Returns the color coding from the video mode. Works with scalable image formats too.
    */
    unsigned int getColorCoding() {
        return mColorCoding;
    }
    /**
    * Returns the bit-space used by a pixel.  </br>
    * uses: dc1394_get_color_coding_bit_size </br>
    * This is different from the data depth!</br>
    * For instance, RGB16 has a bit space of 48 bits, YUV422 is 16bits and YU411 is 12bits.
    */
    unsigned int getColorCodingBitSize() {
        return mColorCodingBitSize;
    }
    /**
    * Returns the number of bits per pixel for a certain color coding. </br>
    * uses: dc1394_get_color_coding_data_depth </br>
    * This is the size of the data sent on the bus, the effective data depth may vary. </br>
    * Example: RGB16 is 16, YUV411 is 8, YUV422 is 8.
    */
    unsigned int getColorDataDepth() {
        return mColorDataDepth;
    }
    /**
    * Creates an empty image matching the video mode and allocated memory
    * @see cvCreateImage
    * @post cvReleaseImage()
    */
    IplImage* createImage() {
        return cvCreateImage(cvSize(mWidth, mHeight), mColorDataDepth, mChannels );
    }
    /**
    * Creates an empty image header matching the video mode
    * @see cvCreateImage
    * @post cvReleaseImageHeader()
    */
    IplImage* createImageHeader() {
        return cvCreateImageHeader(cvSize(mWidth, mHeight), mColorDataDepth, mChannels );
    }

    /**
    * Sets camera features all cameras
    * @param defFeature \n
    * FEATURE_BRIGHTNESS\n
    * FEATURE_EXPOSURE\n
    * FEATURE_SHARPNESS\n
    * FEATURE_WHITE_BALANCE\n
    * FEATURE_HUE\n
    * FEATURE_SATURATION\n
    * FEATURE_GAMMA\n
    * FEATURE_SHUTTER\n
    * FEATURE_GAIN\n
    * FEATURE_IRIS\n
    * FEATURE_FOCUS\n
    * FEATURE_TEMPERATURE\n
    * FEATURE_TRIGGER\n
    * FEATURE_TRIGGER_DELAY\n
    * FEATURE_WHITE_SHADING\n
    * FEATURE_FRAME_RATE\n
    * FEATURE_ZOOM\n
    * FEATURE_PAN\n
    * FEATURE_TILT\n
    * FEATURE_OPTICAL_FILTER\n
    * FEATURE_CAPTURE_SIZE\n
    * FEATURE_CAPTURE_QUALITY\n
    * @param value
    * @return zero on success
    */
    int setFeature(int defFeature, int value);
    /**
    * Sets camera features on one cameras based on the cameraIdx parameter
    * @param cameraIdx
    * @return zero on success
    */
    int setFeature(int defFeature, int value, int cameraIdx);
    /**
    * gets camera features of one camera
    * @param defFeature \n
    * @param cameraIdx \n
    * @return zero on success
    */
    int getFeature(int defFeature, uint32_t *pValue, int cameraIdx = 0);
    /**
     * Timestamp of the last caputred/dequed frame of a camera
     * @param cameraIdx
     * @return timestmap
    */
    timeval getTimeLastFrame(int cameraIdx = 0);
    /**
     * Inits the libdc1394 driver
     * @param modeNr
     * @param fpsNr
     * @param cameraGuids guids which defines the order of the cameras \n
     * if you like to capture only one camera you have to fill the vector with your wanted guid
     * @param printModeSelection
    * @return zero on success
    */
    int init ( int modeNr, int fpsNr, std::vector<uint64_t> cameraGuids = std::vector<uint64_t>(0), bool printModeSelection = false );
    
    /**
     * Inits the file driver
     * @param rFileList file list to read
     * @return zero on success
    */
    int init ( const std::vector< std::vector<std::string> > &rFileList );
    /**
     * Removes and delocates image buffers
     * the destructor calls this function to
    */
    void cleanup ( void );
    /**
     * frame index
     * returns the numbers of the images caputred since init
     * @param cameraIdx
     * @return frame count of camera cameraIdx
    */
    unsigned int frameCount (int cameraIdx = 0 );
    
private:

    unsigned int mWidth;
    unsigned int mHeight;
    unsigned int mChannels;
    unsigned int mNrOFCameras;
    unsigned int mColorCoding;
    unsigned int mColorCodingBitSize;
    unsigned int mColorDataDepth;
    IplImage *mpImgTmp;
    std::vector<int> mImagesDequeuIdx; /// Number of dequeued images per camera;
    std::vector<timeval> mTimestamps;
    std::vector< std::vector<std::string> > mFileList;
    std::vector<unsigned int> mFrameCount;
    std::vector<IplImage *> mFileImages;
    unsigned int mFileCountMin;
};
}

#endif
// kate: indent-mode cstyle; space-indent on; indent-width 0; 
