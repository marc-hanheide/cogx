/*
 * 1394-Based Digital Camera Control Library
 *
 * Written by Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include <cctype>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <stdint.h>
#include <inttypes.h>
#include <iostream>



#include "fw.h"
#include <dc1394/dc1394.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
/* uncomment the following to drop frames to prevent delays */
#define MAX_PORTS   4
#define MAX_CAMERAS 8
#define NUM_BUFFERS 4


using namespace V4R;

dc1394camera_t *cameras[MAX_CAMERAS];
dc1394featureset_t features;
dc1394video_frame_t * frames[MAX_CAMERAS];

/* declarations for video1394 */
char *device_name=NULL;


/* Other declarations */
unsigned long frame_length;
long frame_free;
int frame=0;
int adaptor=-1;

int freeze=0;
int average=0;
int fps;
int res;

const char pWndNames[][20] = {"camera1", "camera2", "camera3", "camera4", "camera5", "camera6"};
/*-----------------------------------------------------------------------
 *  Prints the type of format to standard out
 *-----------------------------------------------------------------------*/
void print_format ( uint32_t format ) {
#define print_case(A) case A: printf(#A ""); break;

    switch ( format ) {
        print_case ( DC1394_VIDEO_MODE_160x120_YUV444 );
        print_case ( DC1394_VIDEO_MODE_320x240_YUV422 );
        print_case ( DC1394_VIDEO_MODE_640x480_YUV411 );
        print_case ( DC1394_VIDEO_MODE_640x480_YUV422 );
        print_case ( DC1394_VIDEO_MODE_640x480_RGB8 );
        print_case ( DC1394_VIDEO_MODE_640x480_MONO8 );
        print_case ( DC1394_VIDEO_MODE_640x480_MONO16 );
        print_case ( DC1394_VIDEO_MODE_800x600_YUV422 );
        print_case ( DC1394_VIDEO_MODE_800x600_RGB8 );
        print_case ( DC1394_VIDEO_MODE_800x600_MONO8 );
        print_case ( DC1394_VIDEO_MODE_1024x768_YUV422 );
        print_case ( DC1394_VIDEO_MODE_1024x768_RGB8 );
        print_case ( DC1394_VIDEO_MODE_1024x768_MONO8 );
        print_case ( DC1394_VIDEO_MODE_800x600_MONO16 );
        print_case ( DC1394_VIDEO_MODE_1024x768_MONO16 );
        print_case ( DC1394_VIDEO_MODE_1280x960_YUV422 );
        print_case ( DC1394_VIDEO_MODE_1280x960_RGB8 );
        print_case ( DC1394_VIDEO_MODE_1280x960_MONO8 );
        print_case ( DC1394_VIDEO_MODE_1600x1200_YUV422 );
        print_case ( DC1394_VIDEO_MODE_1600x1200_RGB8 );
        print_case ( DC1394_VIDEO_MODE_1600x1200_MONO8 );
        print_case ( DC1394_VIDEO_MODE_1280x960_MONO16 );
        print_case ( DC1394_VIDEO_MODE_1600x1200_MONO16 );

    default:
        dc1394_log_error ( "Unknown format\n" );
        exit ( 1 );
    }

}

std::string timevalToString_YYYY_MM_DD__hh_mm_ss_mls (const timeval &rTime ) {
    struct tm ct;
    char buf [80];
    memset ( buf, 0, 80 );
    ct = * ( localtime ( ( const time_t* ) &rTime.tv_sec ) );
    strftime ( buf, 80, "%Y-%m-%d--%H-%M-%S", &ct );
    long millisec = ( rTime.tv_usec % 1000000 ) / 1000;
    sprintf ( buf + strlen ( buf ), "--%03ld", millisec );
    return std::string ( buf );
}

#include <boost/date_time/posix_time/posix_time.hpp>
int timevalfromYYYY_MM_DD__hh_mm_ss_mls (const std::string &str, timeval &rTime) {
    using namespace boost::posix_time;
    using namespace boost::gregorian;
    int YYYY=0, MM=0, DD=0, hh=0, mm=0, ss=0, mls=0;
    int readCount = sscanf ( str.c_str(),"%d-%d-%d--%d-%d-%d--%d",&YYYY, &MM, &DD, &hh, &mm, &ss, &mls );
    if (readCount != 7) return 1;
    ptime t(date(YYYY,MM,DD),hours(hh)+minutes(mm)+seconds(ss)+milliseconds(mls));
    ptime timet_start(date(1970,1,1));
    time_duration diff = t - timet_start;
    rTime.tv_sec = diff.ticks()/time_duration::rep_type::res_adjust();
    rTime.tv_usec = diff.fractional_seconds();
    return 0;
}

/*-----------------------------------------------------------------------
 *  Prints the type of print_framerates to standard out
 *-----------------------------------------------------------------------*/
void print_framerates ( uint32_t format ) {
#define print_case(A) case A: printf(#A ""); break;

    switch ( format ) {
        print_case ( DC1394_FRAMERATE_1_875 );
        print_case ( DC1394_FRAMERATE_3_75 );
        print_case ( DC1394_FRAMERATE_7_5 );
        print_case ( DC1394_FRAMERATE_15 );
        print_case ( DC1394_FRAMERATE_30 );
        print_case ( DC1394_FRAMERATE_60 );
        print_case ( DC1394_FRAMERATE_120 );
        print_case ( DC1394_FRAMERATE_240 );

    default:
        dc1394_log_error ( "Unknown print_framerate\n" );
        exit ( 1 );
    }

}



void FW::cleanup ( void ) {
    int i;
    for ( i=0; i < ( int ) mNrOFCameras; i++ ) {
        dc1394_video_set_transmission ( cameras[i], DC1394_OFF );
        dc1394_capture_stop ( cameras[i] );
        dc1394_camera_free(cameras[i]);
    }
}

FW::FW()
        : mWidth(0)
        , mHeight(0)
        , mChannels(0)
        , mNrOFCameras(0)
        , mpImgTmp(NULL)
        , mImagesDequeuIdx(MAX_CAMERAS, 0)
        , mTimestamps(MAX_CAMERAS)
        , mFrameCount(MAX_CAMERAS,0)
        , mFileImages(MAX_CAMERAS, (IplImage *) NULL)
        , mFileCountMin(0) {
}

FW::FW(int modeNr, int fpsNr, std::vector<uint64_t> cameraGuids, bool printModeSelection )
        : mWidth(0)
        , mHeight(0)
        , mChannels(0)
        , mNrOFCameras(0)
        , mpImgTmp(NULL)
        , mImagesDequeuIdx(MAX_CAMERAS, 0)
        , mTimestamps(MAX_CAMERAS)
        , mFrameCount(MAX_CAMERAS,0)
        , mFileImages(MAX_CAMERAS, (IplImage *) NULL)
        , mFileCountMin(0) {
    init ( modeNr, fpsNr, cameraGuids, printModeSelection );
}

FW::FW (const std::vector< std::vector<std::string> > &rFileList)
        : mWidth(0)
        , mHeight(0)
        , mChannels(0)
        , mNrOFCameras(0)
        , mpImgTmp(NULL)
        , mImagesDequeuIdx(rFileList.size(), 0)
        , mTimestamps(rFileList.size())
        , mFileList(rFileList)
        , mFrameCount(rFileList.size(), 0)
        , mFileImages(rFileList.size(), (IplImage *) NULL)
        , mFileCountMin(rFileList[0].size()) {
    init(0,0,std::vector<uint64_t>(0),false);
}

FW::~FW( ) {
    if (!mFileList.empty()) {
        for (unsigned int i = 0; i < mNrOFCameras; i++) {
            if (mFileImages[i] != NULL) cvReleaseImage(&mpImgTmp);
        }
        return;
    }
    if (mpImgTmp != NULL) cvReleaseImageHeader(&mpImgTmp);
    cleanup ( );
}
int FW::init ( const std::vector< std::vector<std::string> > &rFileList ) {
    cleanup ( );
    mImagesDequeuIdx =  std::vector<int> (rFileList.size(), 0);
    mTimestamps = std::vector<timeval> (rFileList.size());
    mFileList = std::vector< std::vector<std::string> > (rFileList);
    mFrameCount = std::vector<unsigned int> (rFileList.size(), 0);
    mFileImages = std::vector<IplImage *> (rFileList.size(), (IplImage *) NULL);
    mFileCountMin = rFileList[0].size();
    return init(0,0,std::vector<uint64_t>(0), false);
}

int FW::init ( int modeNr, int fpsNr, std::vector<uint64_t> cameraGuids, bool printModeSelection ) {
    if (!mFileList.empty()) {
        /// A filecamera is used
        IplImage *pImg = cvLoadImage(mFileList[0][0].c_str(), CV_LOAD_IMAGE_UNCHANGED);
        mWidth = pImg->width;
        mHeight = pImg->height;
        mChannels = pImg->nChannels;
        mColorDataDepth = pImg->depth;
        mNrOFCameras = mFileList.size();
        cvReleaseImageHeader(&pImg);

        for (unsigned int i = 0; i < mNrOFCameras; i++) {
            if (mFileImages[i] != NULL) cvReleaseImage(&mpImgTmp);
            if (mFileList[i].size() < mFileCountMin) {
                mFileCountMin = mFileList[i].size();
            }
        }

        return FW_OK;
    }
    dc1394error_t err = DC1394_SUCCESS;
    std::vector<uint64_t> usedGuids = readCameraGuids();
    if(usedGuids.size() == 0){
        dc1394_log_error ( "No cameras to initalize" );
                return FW_ERROR;
    }
    if (cameraGuids.size() > 0) {
        ///Check if the guids are valid
        for (unsigned int i = 0; i < cameraGuids.size(); i++) {
            bool cameraExist = false;
            for (unsigned int j = 0; j < usedGuids.size(); j++ ) {
                if (cameraGuids[i] == usedGuids[j]) {
                    cameraExist = true;
                }
            }
            if (cameraExist == false) {
                dc1394_log_error ( "No maching cameras guid 0x%llx", cameraGuids[i] );
                return FW_ERROR;
            }
        }
        usedGuids = cameraGuids;
    }

    if (usedGuids.size() >= MAX_CAMERAS) {
        dc1394_log_error ( "More cameras as hardcoded -> MAX_CAMERAS");
        return FW_ERROR;
    }

    dc1394_t *dc1394 = dc1394_new ();
    if ( !dc1394 ) {
      dc1394_log_error ( "Could not init dc1394" );
      return FW_ERROR;
    }
    for (unsigned int i = 0; i < usedGuids.size(); i++) {
        cameras[i] = dc1394_camera_new ( dc1394, usedGuids[i] );
        if ( !cameras[i] ) {
            dc1394_log_warning ( "Failed to initialize camera with guid 0x%llx", usedGuids[i] );
            return FW_ERROR;
        }
    }


    mNrOFCameras = usedGuids.size();

    if ( mNrOFCameras == 0 ) {
        dc1394_log_error ( "No cameras found" );
        exit ( 1 );
    }

    dc1394framerates_t framerates;
    dc1394video_modes_t modes;
    for (int i = 0; i < ( int ) mNrOFCameras; i++ ) {
        err=dc1394_video_get_supported_modes ( cameras[i], &modes );
        DC1394_ERR_RTN ( err,"Could not get list of modes" );
        if (printModeSelection) printf ( "Camera: %i guid: 0x%llx\n", i, cameras[i]->guid);
        for (int j = 0; j < ( int ) modes.num; j++ ) {
            if (printModeSelection) printf ( "%2i : ", j );
            if (printModeSelection) print_format ( modes.modes[j] );
            if (printModeSelection) printf ( "\n" );
            err=dc1394_video_get_supported_framerates ( cameras[i],modes.modes[j], &framerates );
            DC1394_ERR_RTN ( err,"Could not get list of framerates" );
            for (int k = 0; k < ( int ) framerates.num; k++ ) {
                if (printModeSelection) printf ( " %2i : ", k );
                if (printModeSelection) print_framerates ( framerates.framerates[k] );
                if (printModeSelection) printf ( "\n" );
            }
        }
    }
    //dc1394_feature_print_all(&features, stdout);
    res = modes.modes[modeNr ];
    fps = framerates.framerates[fpsNr];

    for (int i = 0; i < ( int ) mNrOFCameras; i++ ) {
        uint32_t val;

        if ( dc1394_video_get_bandwidth_usage(cameras[i], &val) == DC1394_SUCCESS &&
                dc1394_iso_release_bandwidth(cameras[i], val) == DC1394_SUCCESS )
            if (printModeSelection) std::cout << "Succesfully released " << val << " bytes of Bandwidth." << std::endl;

        if ( dc1394_video_get_iso_channel(cameras[i], &val) == DC1394_SUCCESS &&
                dc1394_iso_release_channel(cameras[i], val) == DC1394_SUCCESS )
            if (printModeSelection) std::cout << "Succesfully released ISO channel #" << val << "." <<  std::endl;

        err=dc1394_video_set_iso_speed ( cameras[i], DC1394_ISO_SPEED_400 );
        DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not set ISO speed" );

        err=dc1394_video_set_mode ( cameras[i], ( dc1394video_mode_t ) res );
        DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not set video mode" );

        err=dc1394_video_set_framerate ( cameras[i], ( dc1394framerate_t )  fps );
        DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not set framerate" );

        err=dc1394_capture_setup ( cameras[i], NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT );
        DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera" );

        if (i == 0) {
            err=dc1394_camera_set_broadcast(cameras[i], DC1394_TRUE);
            DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not set camera broadcast" );
        }

        err=dc1394_video_set_transmission ( cameras[i], DC1394_ON );
        DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not start camera iso transmission" );
    }

    DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera" );

    err=dc1394_get_image_size_from_video_mode ( cameras[0], res, &mWidth, &mHeight );
    DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not read image size" );

    err=dc1394_get_color_coding_from_video_mode(cameras[0], (dc1394video_mode_t ) res, (dc1394color_coding_t  *) &mColorCoding);
    DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not read color coding" );

    err=dc1394_get_color_coding_bit_size ( (dc1394color_coding_t) mColorCoding, &mColorCodingBitSize );
    DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not read bit-space used by a pixel" );

    dc1394_get_color_coding_data_depth( (dc1394color_coding_t) mColorCoding, &mColorDataDepth );
    DC1394_ERR_CLN_RTN ( err,cleanup(),"Could not read bits per pixel for a certain color coding" );

    fflush ( stdout );
    if ( mNrOFCameras < 1 ) {
        perror ( "no cameras found :(\n" );
        cleanup();
        exit ( -1 );
    }

    mpImgTmp = cvCreateImageHeader(cvSize ( mWidth, mHeight ), IPL_DEPTH_8U, 1);
    mChannels = mColorCodingBitSize/mColorDataDepth;


    mFrameCount = std::vector<unsigned int> (mNrOFCameras,0);
    return FW_OK;
}
std::vector<uint64_t> FW::readCameraGuids() {
    std::vector<uint64_t> cameraGuids;
    dc1394_t * d;
    dc1394camera_list_t * list;

    dc1394error_t err;

    d = dc1394_new ();
    if ( !d ) {
        cameraGuids.clear();
        return cameraGuids;
    }
    err=dc1394_camera_enumerate ( d, &list );
    if (err != DC1394_SUCCESS) {
        dc1394_log_error ( "Failed to enumerate cameras" );
        cameraGuids.clear();
        return cameraGuids;
    }

    if ( list->num == 0 ) {
        dc1394_log_error ( "No cameras found" );
        cameraGuids.clear();
        return cameraGuids;
    }

    cameraGuids.clear();
    for (int i = 0; i < ( int ) list->num; i++ ) {
        cameraGuids.push_back(list->ids[i].guid);
    }
    dc1394_camera_free_list ( list );
    return cameraGuids;
}
int FW::setFeature(int defFeature, int value) {
    if (!mFileList.empty()) return FW_OK;
    int err;
    for ( int i = 0; i < ( int ) mNrOFCameras; i++ ) {
        err=setFeature(defFeature,value, i);
        if (err != 0) {
            dc1394_log_error ( "Error on camera %i",  i );
            return FW_ERROR;
        }
    }
    return FW_OK;
}

int FW::getFeature(int defFeature, uint32_t *pValue, int cameraId) {
    if (!mFileList.empty()) return FW_OK;
    dc1394error_t err;
    err=dc1394_feature_get_value (cameras[cameraId], (dc1394feature_t) defFeature, pValue);
    if (err != DC1394_SUCCESS ) {
        char pErrMsg[0xFF];
        switch (defFeature) {
        case FEATURE_BRIGHTNESS:
            sprintf(pErrMsg, "Could not get feature BRIGHTNESS");
            break;
        case FEATURE_EXPOSURE:
            sprintf(pErrMsg, "Could not get feature EXPOSURE");
            break;
        case FEATURE_SHARPNESS:
            sprintf(pErrMsg, "Could not get feature SHARPNESS");
            break;
        case FEATURE_WHITE_BALANCE:
            sprintf(pErrMsg, "Could not get feature WHITE_BALANCE");
            break;
        case FEATURE_HUE:
            sprintf(pErrMsg, "Could not get feature HUE");
            break;
        case FEATURE_SATURATION:
            sprintf(pErrMsg, "Could not get feature SATURATION");
            break;
        case FEATURE_GAMMA:
            sprintf(pErrMsg, "Could not get feature GAMMA");
            break;
        case FEATURE_SHUTTER:
            sprintf(pErrMsg, "Could not get feature SHUTTER");
            break;
        case FEATURE_GAIN:
            sprintf(pErrMsg, "Could not get feature GAIN");
            break;
        case FEATURE_IRIS:
            sprintf(pErrMsg, "Could not get feature IRIS");
            break;
        case FEATURE_FOCUS:
            sprintf(pErrMsg, "Could not get feature FOCUS");
            break;
        case FEATURE_TEMPERATURE:
            sprintf(pErrMsg, "Could not get feature TEMPERATURE");
            break;
        case FEATURE_TRIGGER:
            sprintf(pErrMsg, "Could not get feature TRIGGER");
            break;
        case FEATURE_TRIGGER_DELAY:
            sprintf(pErrMsg, "Could not get feature TRIGGER_DELAY");
            break;
        case FEATURE_WHITE_SHADING:
            sprintf(pErrMsg, "Could not get feature WHITE_SHADING");
            break;
        case FEATURE_FRAME_RATE:
            sprintf(pErrMsg, "Could not get feature FRAME_RATE");
            break;
        case FEATURE_ZOOM:
            sprintf(pErrMsg, "Could not get feature ZOOM");
            break;
        case FEATURE_PAN:
            sprintf(pErrMsg, "Could not get feature PAN");
            break;
        case FEATURE_TILT:
            sprintf(pErrMsg, "Could not get feature TILT");
            break;
        case FEATURE_OPTICAL_FILTER:
            sprintf(pErrMsg, "Could not get feature OPTICAL_FILTER");
            break;
        case FEATURE_CAPTURE_SIZE:
            sprintf(pErrMsg, "Could not get feature CAPTURE_SIZE");
            break;
        case FEATURE_CAPTURE_QUALITY:
            sprintf(pErrMsg, "Could not get feature CAPTURE_QUALITY");
            break;
        default:
            sprintf(pErrMsg, "Feature not known");
        }
        DC1394_ERR_CLN_RTN ( err,cleanup(),pErrMsg);
        return FW_ERROR;
    }
    else {
        return FW_OK;
    }
}
int FW::setFeature(int defFeature, int value, int cameraId) {
    if (!mFileList.empty()) return FW_OK;
    dc1394error_t err;
    err=dc1394_feature_set_value (cameras[cameraId], (dc1394feature_t) defFeature, value);
    if (err != DC1394_SUCCESS ) {
        char pErrMsg[0xFF];
        switch (defFeature) {
        case FEATURE_BRIGHTNESS:
            sprintf(pErrMsg, "Could not set feature BRIGHTNESS to %i", value);
            break;
        case FEATURE_EXPOSURE:
            sprintf(pErrMsg, "Could not set feature EXPOSURE to %i", value);
            break;
        case FEATURE_SHARPNESS:
            sprintf(pErrMsg, "Could not set feature SHARPNESS to %i", value);
            break;
        case FEATURE_WHITE_BALANCE:
            sprintf(pErrMsg, "Could not set feature WHITE_BALANCE to %i", value);
            break;
        case FEATURE_HUE:
            sprintf(pErrMsg, "Could not set feature HUE to %i", value);
            break;
        case FEATURE_SATURATION:
            sprintf(pErrMsg, "Could not set feature SATURATION to %i", value);
            break;
        case FEATURE_GAMMA:
            sprintf(pErrMsg, "Could not set feature GAMMA to %i", value);
            break;
        case FEATURE_SHUTTER:
            sprintf(pErrMsg, "Could not set feature SHUTTER to %i", value);
            break;
        case FEATURE_GAIN:
            sprintf(pErrMsg, "Could not set feature GAIN to %i", value);
            break;
        case FEATURE_IRIS:
            sprintf(pErrMsg, "Could not set feature IRIS to %i", value);
            break;
        case FEATURE_FOCUS:
            sprintf(pErrMsg, "Could not set feature FOCUS to %i", value);
            break;
        case FEATURE_TEMPERATURE:
            sprintf(pErrMsg, "Could not set feature TEMPERATURE to %i", value);
            break;
        case FEATURE_TRIGGER:
            sprintf(pErrMsg, "Could not set feature TRIGGER to %i", value);
            break;
        case FEATURE_TRIGGER_DELAY:
            sprintf(pErrMsg, "Could not set feature TRIGGER_DELAY to %i", value);
            break;
        case FEATURE_WHITE_SHADING:
            sprintf(pErrMsg, "Could not set feature WHITE_SHADING to %i", value);
            break;
        case FEATURE_FRAME_RATE:
            sprintf(pErrMsg, "Could not set feature FRAME_RATE to %i", value);
            break;
        case FEATURE_ZOOM:
            sprintf(pErrMsg, "Could not set feature ZOOM to %i", value);
            break;
        case FEATURE_PAN:
            sprintf(pErrMsg, "Could not set feature PAN to %i", value);
            break;
        case FEATURE_TILT:
            sprintf(pErrMsg, "Could not set feature TILT to %i", value);
            break;
        case FEATURE_OPTICAL_FILTER:
            sprintf(pErrMsg, "Could not set feature OPTICAL_FILTER to %i", value);
            break;
        case FEATURE_CAPTURE_SIZE:
            sprintf(pErrMsg, "Could not set feature CAPTURE_SIZE to %i", value);
            break;
        case FEATURE_CAPTURE_QUALITY:
            sprintf(pErrMsg, "Could not set feature CAPTURE_QUALITY to %i", value);
            break;
        default:
            sprintf(pErrMsg, "Feature not known");
        }
        DC1394_ERR_CLN_RTN ( err,cleanup(),pErrMsg);
        return FW_ERROR;
    }
    else {
        return FW_OK;
    }
}

int FW::capture ( const std::vector<IplImage*> &rImages, const std::vector<std::string> *pSaveToFolders) {
    if (rImages.size() < mNrOFCameras) {
        dc1394_log_error ( "Image vector to small");
        return FW_ERROR;
    }
    std::vector<IplImage> imagesHeaders(rImages.size(), *rImages[0]);
    for ( int cameraIdx = 0; cameraIdx < ( int ) mNrOFCameras; cameraIdx++ ) {
        if (dequeue (&imagesHeaders[cameraIdx], cameraIdx )) {
            return FW_ERROR;
        }
    }
    for ( int cameraIdx = 0; cameraIdx < ( int ) mNrOFCameras; cameraIdx++ ) {
        cvCopy(&imagesHeaders[cameraIdx], (CvArr*) rImages[cameraIdx]);
        //memcpy(rImages[i]->imageData , frames[i]->image, rImages[i]->imageSize);
        if (pSaveToFolders != NULL) {
            std::string filename(pSaveToFolders->at(cameraIdx) + timevalToString_YYYY_MM_DD__hh_mm_ss_mls(mTimestamps[cameraIdx]) + std::string(".bmp"));
            cvSaveImage(filename.c_str(), rImages[cameraIdx]);
        }
    }

    for ( int cameraIdx = 0; cameraIdx < ( int ) mNrOFCameras; cameraIdx++ ) {
        if (enqueue (cameraIdx )) {
            return FW_ERROR;
        }
    }

    return FW_OK;
}

int FW::dequeue (IplImage *pImageHeader, int cameraIdx, const std::string *pSaveToFolder ) {
    int err = 1;
    if (mImagesDequeuIdx[cameraIdx] != 0) {
        dc1394_log_error ( "%i images on camera %i are currently dequeued", mImagesDequeuIdx[cameraIdx],  cameraIdx);
    } else {
        if (mFileList.empty()) {
            gettimeofday ( &mTimestamps[cameraIdx], 0 );
            err = dc1394_capture_dequeue ( cameras[cameraIdx], DC1394_CAPTURE_POLICY_WAIT, &frames[cameraIdx] );
            if (err  == DC1394_SUCCESS ) {
                mImagesDequeuIdx[cameraIdx]++;
                mFrameCount[cameraIdx]++;

                pImageHeader->imageData = (char*) frames[cameraIdx]->image;
                pImageHeader->imageDataOrigin = pImageHeader->imageData;
                if (pSaveToFolder != NULL) {
                    std::string filename((*pSaveToFolder) + timevalToString_YYYY_MM_DD__hh_mm_ss_mls(mTimestamps[cameraIdx]) + std::string(".bmp"));
                    cvSaveImage(filename.c_str(), pImageHeader);
                }
            } else {
                dc1394_log_error ( "Failed to dequeue from camera %d", cameraIdx );
                return FW_ERROR;
            }
        } else {
            std::string filename = mFileList[cameraIdx][mFrameCount[cameraIdx]];
            if (timevalfromYYYY_MM_DD__hh_mm_ss_mls(filename.substr(filename.length()-29,25),mTimestamps[cameraIdx])) {
                gettimeofday ( &mTimestamps[cameraIdx], 0 );
            }
            mFileImages[cameraIdx] = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED);
            mImagesDequeuIdx[cameraIdx]++;
            mFrameCount[cameraIdx]++;
            pImageHeader->imageData = mFileImages[cameraIdx]->imageData;
            pImageHeader->imageDataOrigin = pImageHeader->imageData;
            if (mFrameCount[cameraIdx] >= mFileCountMin) {
                mFrameCount[cameraIdx]  = 0;
            }
        }
    }
    return FW_OK;
}

int FW::dequeue ( const std::vector<IplImage*> &rImageHeaders, const std::vector<std::string> *pSaveToFolders ) {
    if (rImageHeaders.size() < mNrOFCameras) {
        dc1394_log_error ( "Image vector to small");
        return FW_ERROR;
    }
    for ( int cameraIdx = 0; cameraIdx < ( int ) mNrOFCameras; cameraIdx++ ) {
        if (dequeue (rImageHeaders[cameraIdx], cameraIdx, NULL )) {
            return FW_ERROR;
        }
    }
    if ((pSaveToFolders != NULL) && (pSaveToFolders->size() > 0) && mFileList.empty()) {
        for ( int cameraIdx = 0; cameraIdx < ( int ) mNrOFCameras; cameraIdx++ ) {
            if (pSaveToFolders != NULL) {
                //std::string filename(pSaveToFolders->at(cameraIdx) + timevalToString_YYYY_MM_DD__hh_mm_ss_mls(mTimestamps[cameraIdx]) + std::string(".bmp"));
                std::string filename(pSaveToFolders->at(cameraIdx) + timevalToString_YYYY_MM_DD__hh_mm_ss_mls(mTimestamps[0]) + std::string(".bmp"));
                cvSaveImage(filename.c_str(), rImageHeaders[cameraIdx]);
            }
        }
    }
    return FW_OK;
}

int FW::enqueue (int cameraIdx ) {
    int err;
    if (mImagesDequeuIdx[cameraIdx] == 0) {
        dc1394_log_error ( "%i images on camera %i are currently dequeued", mImagesDequeuIdx[cameraIdx],  cameraIdx);
        return FW_ERROR;
    } else {

        if (!mFileList.empty()) {
            if (mFileImages[cameraIdx] != NULL) cvReleaseImage(&mFileImages[cameraIdx]);
            mImagesDequeuIdx[cameraIdx]--;
        } else {
            if ( frames[cameraIdx] ) {
                err = dc1394_capture_enqueue ( cameras[cameraIdx], frames[cameraIdx] );
                if (err  != DC1394_SUCCESS ) {
                    dc1394_log_error ( "Failed to enqueue");
                    return FW_ERROR;
                } else {
                    mImagesDequeuIdx[cameraIdx]--;
                }
            }
        }
    }
    return FW_OK;
}

int FW::enqueue ( ) {
    for ( int cameraIdx = 0; cameraIdx < ( int ) mNrOFCameras; cameraIdx++ ) {
        if (enqueue (cameraIdx )) {
            return FW_ERROR;
        }
    }
    return FW_OK;
}


void FW::rgbToYUV422(IplImage *pSrc, IplImage *pDes) {
    int err;
    err = dc1394_convert_to_YUV422((uint8_t*) pSrc->imageData, (uint8_t *) pDes->imageData, pDes->width, pDes->height, DC1394_BYTE_ORDER_YUYV, DC1394_COLOR_CODING_RGB8, 8);

    if (err  != DC1394_SUCCESS ) {
        dc1394_log_error ( "Failed to convert image RGB to YUV422");
    }
}


void FW::bayerTo(IplImage *pSrc, IplImage *pDes, int method, int filter) {
    int err;
    err = dc1394_bayer_decoding_8bit((uint8_t*) pSrc->imageData, (uint8_t *) pDes->imageData, pDes->width, pDes->height, (dc1394color_filter_t) filter, (dc1394bayer_method_t) method);
    if (err  != DC1394_SUCCESS ) {
        dc1394_log_error ( "Failed to convert image Bayer");
    }
}

unsigned int FW::frameCount (int cameraIdx) {
    return mFrameCount[cameraIdx];
}

timeval FW::getTimeLastFrame(int cameraIdx) {
    return mTimestamps[cameraIdx];
}

