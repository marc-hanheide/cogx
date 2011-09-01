#ifndef AK_SHM_CVSERIALIZE_HPP
#define AK_SHM_CVSERIALIZE_HPP

#include <opencv2/opencv.hpp>
#include <sys/time.h>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/binary_object.hpp>

namespace boost {
namespace serialization {
template<class archive, typename _Tp, int m, int n>
void serialize(archive &ar, cv::Matx<_Tp, m, n> &value, const unsigned int version) {
    ar & make_nvp("value", value.val);
}
template<class archive, typename _Tp>
void serialize(archive &ar, cv::Point_<_Tp> &value, const unsigned int version) {
    ar & make_nvp("x", value.x);
    ar & make_nvp("y", value.y);
}
template<class archive, typename _Tp>
void serialize(archive &ar, cv::Point3_<_Tp> &value, const unsigned int version) {
    ar & make_nvp("x", value.x);
    ar & make_nvp("y", value.y);
    ar & make_nvp("z", value.z);
}
template<class archive, typename _Tp, int n>
void serialize(archive &ar, cv::Vec<_Tp, n> &value, const unsigned int version) {
    ar & make_nvp("value", value.val);
}
#ifndef BOOST_SERIALZE_TIMEVAL
#define BOOST_SERIALZE_TIMEVAL
template<class archive>
void serialize(archive &ar, timeval &value, const unsigned int version) {
    ar & make_nvp ( "sec", value.tv_sec );
    ar & make_nvp ( "usec", value.tv_usec );
}
#endif // BOOST_SERIALZE_TIMEVAL

#ifndef BOOST_SERIALZE_IPLIMAGE
#define BOOST_SERIALZE_IPLIMAGE
template<class archive>
void serialize(archive &ar, IplImage &value, const unsigned int version) {
    using boost::serialization::make_nvp;
    using boost::serialization::make_binary_object;
    IplImage *img = cvCreateImageHeader(cvSize(0,0), 1, CV_8U);
    if ( archive::is_saving::value ) {
        *img = value;
    }
    ar & make_nvp ( "nSize", img->nSize );
    ar & make_nvp ( "ID", img->ID );
    ar & make_nvp ( "nChannels", img->nChannels );
    ar & make_nvp ( "alphaChannel", img->alphaChannel );
    ar & make_nvp ( "depth", img->depth );
    ar & make_nvp ( "colorModel", img->colorModel );
    ar & make_nvp ( "channelSeq", img->channelSeq );
    ar & make_nvp ( "dataOrder", img->dataOrder );
    ar & make_nvp ( "origin", img->origin );
    ar & make_nvp ( "align", img->align );
    ar & make_nvp ( "width", img->width );
    ar & make_nvp ( "height", img->height );
    // ar & make_nvp ( "roi", img->roi );
    // ar & make_nvp ( "maskROI", img->maskROI );
    // ar & make_nvp ( "imageId", img->imageId );
    // ar & make_nvp ( "tileInfo", img->tileInfo );
    ar & make_nvp ( "imageSize", img->imageSize );
    // ar & make_nvp ( "imageData", img->imageData );
    ar & make_nvp ( "widthStep", img->widthStep );
    ar & make_nvp ( "BorderMode", img->BorderMode );
    ar & make_nvp ( "BorderConst", img->BorderConst );
    // ar & make_nvp ( "imageDataOrigin", img->imageDataOrigin );
    if ( archive::is_saving::value ) {
        ar & make_nvp ( "imageData", make_binary_object ( img->imageData, img->imageSize ) );
    }
    if ( archive::is_loading::value ) {
        if (    ( value.width  != img->width )        || ( value.height        != img->height ) || //
                ( value.depth  != img->depth )        || ( value.nChannels     != img->nChannels ) || //
                ( value.imageSize != img->imageSize ) || ( value.widthStep     != img->widthStep ) ) {
            if (value.imageData != NULL) {
              cvReleaseData ( &value );
            }
            value = *img;
            cvCreateData (&value);
        }
        ar & make_nvp ( "imageData", make_binary_object ( value.imageData, value.imageSize ) );
        value.imageDataOrigin = value.imageData;
    }
    cvReleaseImageHeader(&img);
}
#endif // BOOST_SERIALZE_IPLIMAGE



}
}
#endif
