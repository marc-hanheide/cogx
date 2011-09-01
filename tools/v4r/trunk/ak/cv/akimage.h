/**
 * @file akimage.h
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef AKIMAGE_H
#define AKIMAGE_H

#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sys/time.h>

using namespace std;

namespace ak {
namespace cv {

class CameraInfo {
public:

    static const int FRAME_NAME_MAX = 0xFF;

    ///DEFAULTS and ERRORS
    static const int FORMAT_WRONG = -1;
    static const int FORMAT_NOTUSED = 0;
    static const int FORMAT_UNKOWN = 1;

    ///MONO
    static const int FORMAT_MONO8 = 10;
    static const int FORMAT_MONO16 = 11;

    /// RGB
    static const int FORMAT_RGB8 = 20;
    static const int FORMAT_BGR8 = 21;
    static const int FORMAT_RGBA8 = 22;
    static const int FORMAT_BGRA8 = 23;

    /// RGBA
    static const int FORMAT_RGB16 = 30;
    static const int FORMAT_BGR16 = 31;
    static const int FORMAT_RGBA16 = 32;
    static const int FORMAT_BGRA16 = 33;

    /// YUV
    static const int FORMAT_YUV422 = 50;
    static const int FORMAT_YUV = 51;

    /// Bayer
    static const int FORMAT_RGGB8 = 100;
    static const int FORMAT_BGGR8 = 101;
    static const int FORMAT_GBRG8 = 102;
    static const int FORMAT_GRBG8 = 103;
    static const int FORMAT_RGGB16 = 104;
    static const int FORMAT_BGGR16 = 105;
    static const int FORMAT_GBRG16 = 106;
    static const int FORMAT_GRBG16 = 107;

    int format;       /// data format
    unsigned long id;       /// id
    char frame[FRAME_NAME_MAX];     /// frame
    timeval tstamp;       /// timestamp
    ::cv::Vec<double,6> motion;     /// motion    vx, vy, vz, wx, wy, wz
    ::cv::Vec<double,6> extrinsic;    /// extrinsic  x,  y,  z, wx, wy, wz
    ::cv::Vec<double,4> intrinsic_src;    /// Intrinsic camera matrix for the raw (distorted) image fx, fy, cx, cy
    ::cv::Vec<double,5> distortions;    /// distortions k1, k2, t1, t2, k3
    /** Intrinsic camera matrix for the undistorted image fx, fy, cx, cy
     * Projection/camera matrix
     *     [fx'  0  cx' Tx]
     *     [ 0  fy' cy' Ty]
     *     [ 0   0   1   0]
     **/
    ::cv::Matx<double,3, 4> intrinsic_des;
    /** Rectification matrix (stereo cameras only)
     * A rotation matrix aligning the camera coordinate system to the ideal
     * stereo image plane so that epipolar lines in both stereo images are
     * parallel.
    **/
    ::cv::Matx<double,3, 3> rectification;

public:
    CameraInfo() {
        clear();
    }
    CameraInfo(const CameraInfo &r) {
        format = r.format;
        id = r.id;
        strcpy(frame, r.frame);
        tstamp = r.tstamp;
        motion = r.motion;
        extrinsic = r.extrinsic;
        intrinsic_src = r.intrinsic_src;
        intrinsic_des = r.intrinsic_des;
        distortions = r.distortions;
    }
    std::string toString() const {
        std::stringstream ss;
        ss << "id = " << id << ", format = " << encodings() << ", frame = " << frame;
        ss << ", tstamp = " << getTimeStr() << " == " << tstamp.tv_sec << " sec, " << tstamp.tv_usec  << " usec" << std::endl;
        ss << std::setprecision (5) << "motion         = [" << motion[0]  << ", " << motion[1]   << ", " << motion[2]  << ", "  << motion[3]  << ", " << motion[4]   << ", " << motion[5] << "]" << std::endl;
        ss << std::setprecision (5) << "extrinsic      = [" << extrinsic[0] << ", " << extrinsic[1]  << ", " << extrinsic[2]  << ", " << extrinsic[3]  << ", " << extrinsic[4]  << ", " << extrinsic[5]  << "]" << std::endl;
        ss << std::setprecision (5) << "intrinsic_src  = [" << intrinsic_src[0] << ", " << intrinsic_src[1]  << ", " << intrinsic_src[2]  << ", " << intrinsic_src[3]  << "]" << std::endl;
        ss << std::setprecision (5) << "intrinsic_des  = [" << intrinsic_des(0,0) << ", " << intrinsic_des(1,1)  << ", " << intrinsic_des(0,2)  << ", " << intrinsic_des(1,2)   << ", " << intrinsic_des(0,3)  << ", " << intrinsic_des(1,3)  << "]" << std::endl;
        ss << std::setprecision (5) << "distortions    = [" << distortions[0] << ", " << distortions[1]  << ", " << distortions[2]  << ", " << distortions[3]  << ", " << distortions[4]  << "]" << std::endl;
        return ss.str();
    }
    void clear() {
        format = FORMAT_NOTUSED;
        id = -1;
        sprintf(frame, "no_frame");
        tstamp.tv_usec = 0, tstamp.tv_sec = 0;
        motion = ::cv::Vec<double, 6>(0,0,0,0,0,0);
        extrinsic = ::cv::Vec<double, 6>(0,0,0,0,0,0);
        distortions = ::cv::Vec<double, 5>(0,0,0,0,0);
        intrinsic_src = ::cv::Vec<double, 4>(0,0,0,0);
        intrinsic_des = ::cv::Matx<double, 3,4>::eye();
        rectification = ::cv::Matx<double, 3,3>::eye();
    }
    /**
     * Sets the time to now
     **/
    void setTimeNow() {
        gettimeofday ( &tstamp, 0 );
    }
    void setTime(unsigned long _sec, unsigned long _msec, unsigned long _usec, unsigned long _nsec) {
        unsigned long usec = (_msec * 1000 + _usec + _nsec/1000);
        tstamp.tv_usec =  usec % 1000000;
        tstamp.tv_sec = (usec) / 1000000 + _sec;
    }
    /**
     * prints the time as string YYYY-MM-DD--HH-MM-SS-MS
     * @param localtime on false it will use gmtime instead of localtime
     **/
    std::string getTimeStr (bool _localtime = true) const {
        struct tm ct;
        char buf [80];
        memset ( buf, 0, 80 );
        try {
            if ( tstamp.tv_sec > 1576800000 /* 2020 */) {
                return "out of range";
            }
            if ( _localtime ) {
                ct = * ( localtime ( ( const time_t* ) &tstamp.tv_sec ) );
            } else {
                ct = * ( gmtime ( ( const time_t* ) &tstamp.tv_sec ) );
            }
        } catch (const std::exception& ex) {
            // AK_LOG_ERROR << "Exception in ShmVar::openOrCreate() " << varName << ": " << ex.what();
            std::cerr << "Exception in CameraInfo::getTimeStr() " << ": " << ex.what();
            return "out of range";
        }
        strftime ( buf, 80, "%Y-%m-%d--%H-%M-%S", &ct );
        sprintf ( buf + strlen ( buf ), "--%03ld", tstamp.tv_usec/1000 );
        return string ( buf );
    }
    void setFrame(const char* frame_) {
        if (strlen(frame_) > (unsigned int) FRAME_NAME_MAX) {
            strncpy (frame, frame_, FRAME_NAME_MAX-1);
        } else {
            strcpy(frame, frame_);
        }
    }
    const char* encodings() const {
        return  encodings(format);
    }
    static const char* encodings(int format) {
        switch (format) {
        case FORMAT_NOTUSED:
            return "not_used";
        case FORMAT_UNKOWN:
            return "unkown";
        case FORMAT_MONO8:
            return "mono8";
        case FORMAT_MONO16:
            return "mono16";
        case FORMAT_RGB8:
            return "rgb8";
        case FORMAT_BGR8:
            return "bgr8";
        case FORMAT_RGBA8:
            return "rgba8";
        case FORMAT_BGRA8:
            return "bgra8";
        case FORMAT_RGB16:
            return "rgb16";
        case FORMAT_BGR16:
            return "bgr16";
        case FORMAT_RGBA16:
            return "rgba16";
        case FORMAT_BGRA16:
            return "bgr16";
        case FORMAT_YUV422:
            return "yuv422";
        case FORMAT_YUV:
            return "yuv";
        case FORMAT_RGGB8:
            return "bayer_rggb8";
        case FORMAT_BGGR8:
            return "bayer_bggr8";
        case FORMAT_GBRG8:
            return "bayer_gbrg8";
        case FORMAT_GRBG8:
            return "bayer_grbg8";
        case FORMAT_RGGB16:
            return "bayer_rggb16";
        case FORMAT_BGGR16:
            return "bayer_bggr16";
        case FORMAT_GBRG16:
            return "bayer_gbrg16";
        case FORMAT_GRBG16:
            return "bayer_grbg16";
        default:
            return "wrong format";;
        }
    }
    static int formats(const char *encoding) {
        if (strcmp(encoding, "not_used") == 0)  return FORMAT_NOTUSED;
        if (strcmp(encoding, "unkown") == 0) return FORMAT_UNKOWN;
        if (strcmp(encoding, "mono8") == 0) return FORMAT_MONO8;
        if (strcmp(encoding, "mono16") == 0) return FORMAT_MONO16;
        if (strcmp(encoding, "rgb8") == 0) return FORMAT_RGB8;
        if (strcmp(encoding, "bgr8") == 0) return FORMAT_BGR8;
        if (strcmp(encoding, "rgba8") == 0) return FORMAT_RGBA8;
        if (strcmp(encoding, "bgra8") == 0) return FORMAT_BGRA8;
        if (strcmp(encoding, "rgb16") == 0) return FORMAT_RGB16;
        if (strcmp(encoding, "bgr16") == 0) return FORMAT_BGR16;
        if (strcmp(encoding, "rgba16") == 0) return FORMAT_RGBA16;
        if (strcmp(encoding, "bgra16") == 0) return FORMAT_BGRA16;
        if (strcmp(encoding, "yuv422") == 0) return FORMAT_YUV422;
        if (strcmp(encoding, "yuv") == 0) return FORMAT_YUV;
        if (strcmp(encoding, "bayer_rggb8") == 0) return FORMAT_RGGB8;
        if (strcmp(encoding, "bayer_bggr8") == 0) return FORMAT_BGGR8;
        if (strcmp(encoding, "bayer_gbrg8") == 0) return FORMAT_GBRG8;
        if (strcmp(encoding, "bayer_grbg8") == 0) return FORMAT_GRBG8;
        if (strcmp(encoding, "bayer_rggb16") == 0) return FORMAT_RGGB16;
        if (strcmp(encoding, "bayer_bggr16") == 0) return FORMAT_BGGR16;
        if (strcmp(encoding, "bayer_gbrg16") == 0) return FORMAT_GBRG16;
        if (strcmp(encoding, "bayer_grbg16") == 0) return FORMAT_GRBG16;
        if (strcmp(encoding, "wrong format") == 0) return FORMAT_WRONG ;
        return FORMAT_WRONG ;
    }
    static int decode_depth(const char *encoding) {
        if (strcmp(encoding, "not_used") == 0)  return IPL_DEPTH_8U;
        if (strcmp(encoding, "unkown") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "mono8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "mono16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "rgb8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "bgr8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "rgba8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "bgra8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "rgb16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "bgr16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "rgba16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "bgra16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "yuv422") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "yuv") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "bayer_rggb8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "bayer_bggr8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "bayer_gbrg8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "bayer_grbg8") == 0) return IPL_DEPTH_8U;
        if (strcmp(encoding, "bayer_rggb16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "bayer_bggr16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "bayer_gbrg16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "bayer_grbg16") == 0) return IPL_DEPTH_16U;
        if (strcmp(encoding, "wrong format") == 0) return IPL_DEPTH_8U;
        return IPL_DEPTH_8U;
    }
    static int decode_nchannes(const char *encoding) {
        if (strcmp(encoding, "not_used") == 0)  return 1;
        if (strcmp(encoding, "unkown") == 0) return 1;
        if (strcmp(encoding, "mono8") == 0) return 1;
        if (strcmp(encoding, "mono16") == 0) return 1;
        if (strcmp(encoding, "rgb8") == 0) return 3;
        if (strcmp(encoding, "bgr8") == 0) return 3;
        if (strcmp(encoding, "rgba8") == 0) return 4;
        if (strcmp(encoding, "bgra8") == 0) return 4;
        if (strcmp(encoding, "rgb16") == 0) return 3;
        if (strcmp(encoding, "bgr16") == 0) return 3;
        if (strcmp(encoding, "rgba16") == 0) return 4;
        if (strcmp(encoding, "bgra16") == 0) return 4;
        if (strcmp(encoding, "yuv422") == 0) return 2;
        if (strcmp(encoding, "yuv") == 0) return 3;
        if (strcmp(encoding, "bayer_rggb8") == 0) return 1;
        if (strcmp(encoding, "bayer_bggr8") == 0) return 1;
        if (strcmp(encoding, "bayer_gbrg8") == 0) return 1;
        if (strcmp(encoding, "bayer_grbg8") == 0) return 1;
        if (strcmp(encoding, "bayer_rggb16") == 0) return 1;
        if (strcmp(encoding, "bayer_bggr16") == 0) return 1;
        if (strcmp(encoding, "bayer_gbrg16") == 0) return 1;
        if (strcmp(encoding, "bayer_grbg16") == 0) return 1;
        if (strcmp(encoding, "wrong format") == 0) return 1;
        return 1;
    }
};

class Image : public IplImage , public CameraInfo {
public:
    /**
    * @brief constructor<br>
    **/
    Image() {
        CameraInfo::clear();
        imageData = imageDataOrigin = NULL;
    }
    Image(CvSize _size, int _depth, int _channels) {
        CameraInfo::clear();
        create(_size, _depth, _channels);
    }
    /** creates header and allocates memory
     * @param size
     * @param depth
     * @param channels
     **/
    void create(CvSize _size, int _depth, int _channels, bool _allocate = true) {
        cvInitImageHeader(ipl(), _size, _depth, _channels);
        if (_allocate) cvCreateData(ipl());
    }

    IplImage *ipl() {
        return this;
    }

    CameraInfo *cameraInfo() {
        return this;
    }

};
}
}

#endif //AKTIMAGE_H
