/**
 * @file akimage.h
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef AKIMAGE_SERIALIZE_H
#define AKIMAGE_SERIALIZE_H

#include <ak/cv/akimage.h>
#include <ak/cv/serialize.hpp>

#include <fstream>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/binary_object.hpp>

namespace boost {
namespace serialization {

#ifndef BOOST_SERIALZE_AKCAMERAINFO
#define BOOST_SERIALZE_AKCAMERAINFO
template<class archive>
void serialize(archive &ar, ak::cv::CameraInfo &value, const unsigned int version) {
    ar & make_nvp ( "format", value.format );
    ar & make_nvp ( "id", value.id );
    ar & make_nvp ( "sec", value.tstamp.tv_sec );
    ar & make_nvp ( "usec", value.tstamp.tv_usec );
    ar & make_nvp ( "frame", value.frame );
    ar & make_nvp ( "motion", value.motion );
    ar & make_nvp ( "extrinsic", value.extrinsic );
    ar & make_nvp ( "intrinsic_src", value.intrinsic_src );
    ar & make_nvp ( "intrinsic_des", value.intrinsic_des );
    ar & make_nvp ( "distortions", value.distortions );
}
#endif // BOOST_SERIALZE_AKCAMERAINFO

#ifndef BOOST_SERIALZE_AKIMAGE
#define BOOST_SERIALZE_AKIMAGE
template<class archive>
void serialize(archive &ar, ak::cv::Image &value, const unsigned int version) {
    ak::cv::CameraInfo &camera_info = *value.cameraInfo();
    IplImage &image = *value.ipl();
    ar & make_nvp ( "camera_info", camera_info );
    ar & make_nvp ( "image", image );
}
#endif // BOOST_SERIALZE_AKIMAGE

} // namespace boost 
} // namespace serialization 

namespace ak {
namespace cv {
  

/**
* @brief reads a image form a xml file
* @param value image
* @param file filename
**/
void readXML (ak::cv::Image &value, const std::string &file ) {
    std::ifstream ifs ( file.c_str() );
    assert ( ifs.good() );
    boost::archive::xml_iarchive xml ( ifs );
    xml >> boost::serialization::make_nvp ( "akimage", value );
}

/**
* @brief writes to a xml file <br>
* @param value image
* @param rFolder
* @param rFile a good file name is the timestamp this->timestamp.fmt_YYYY_MM_DD__hh_mm_ss_mls()
**/
void writeXML (const ak::cv::Image &value, const std::string &rFolder, const std::string &rFile) {
    char pFileXML[0xFF];
    sprintf ( pFileXML,"%s/%s", rFolder.c_str(),rFile.c_str() );
    std::ofstream ofs ( pFileXML );
    assert ( ofs.good() );
    boost::archive::xml_oarchive xml ( ofs );
    xml << boost::serialization::make_nvp ( "akimage", value );
}

} // namespace ak 
} // namespace cv 
#endif //AKIMAGE_SERIALIZE_H
