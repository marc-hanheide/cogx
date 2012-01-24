/**
 * @author Michael Zillich, Andreas Richtsfeld
 * @date Februrary 2009, 2010
 * 
 * TODO convert image size!
 */

#ifndef CONVERT_IMAGE_H
#define CONVERT_IMAGE_H

#include <stdexcept>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Video.hpp"

namespace Video
{

using namespace std;

/**
 * @brief Convert from framework image type to an OpenCV image type (IplImage). \n
 * note: If iplImg is non-null and of appropriate size already, it will be used \n
 * as is. If it is non-null but has wrong size it will be released and \n
 * reallocated. If it is null it will be allocated. \n
 * Of course you are responsible for releasing the iplImg later on.
 */
void convertImageToIpl(const Video::Image & img, IplImage ** iplImg);

/**
 * @brief Convert from framework image type to an OpenCV image. \n
 * note: returned image must be cvRelease'd \n
 * A convenience form of the above.
 * @param image Video image to convert
 * @return Return converted openCv iplImage.
 */
IplImage* convertImageToIpl(const Video::Image & image);

/**
 * @brief Create an IplImage that uses the data from a Video::Image. \n
 * The image data is shared between Video::Image and IplImage. \n
 * WARNING: DO NOT cvReleaseImage() the obtained VideoImage.
 *    Use releaseWrappedImage() instead. \n
 * WARNING: The Video::Image MUST NOT BE DESTROYED while using the
 *    IplImage created with this function.
 * WARNING: At the moment it is not clear if the hack works with
 *    images whose width is not aligned to 4 bytes.
 * WARNING: THIS IS A SEVERE HACK: USE AT YOUR OWN RISK.
 */
IplImage* wrapVideoImage(const Video::Image &image);

/**
 * @brief Release an image obtained with wrapVideoImage(). \n
 * WARNING: DO NOT releaseWrappedImage() if the IplImage was not created 
 *    with wrapVideoImage(). Use cvReleaseImage() instead. \n
 * WARNING: THIS IS A SEVERE HACK: USE AT YOUR OWN RISK.
 */
void releaseWrappedImage(IplImage** pImagePtr);

/**
 * @brief Convert from raw image data to an OpenCV image type (IplImage).
 */
bool convertBytesToIpl(const std::vector<unsigned char>& data, int width, int height, int nchn, IplImage ** iplImg);

/**
 * @brief Convert from raw image data to an OpenCV image type (IplImage).
 * @return Return converted openCv iplImage.
 */
IplImage* convertBytesToIpl(const std::vector<unsigned char>& data, int width, int height, int nchn);

/**
 * @brief Convert from framework image type to an OpenCV grayscale image type \n
 * (IplImage). \n
 * note: returned image must be cvRelease'd
 * @param image Video image.
 * @return Return converted gray iplImage
 */
IplImage* convertImageToIplGray(const Video::Image & image);

/**
 * @brief Convert from framework image type to grayscale contiguous bytes.
 * (IplImage). \n
 */
void convertImageToGrayBytes(const Video::Image & image, std::vector<unsigned char>& data);

/**
 * @brief Converts an OpenCV image type (IplImage) to a system Image format. \n
 * note: If img is of appropriate size already, no extra memory allocation takes \n
 * place.
 * @param iplImg OpenCv IplImage
 * @param img Video Image
 */
void convertImageFromIpl(const IplImage *iplImg, Video::Image &img) throw(runtime_error);
  
  
/**
 * @brief Converts an OpenCV image type (IplImage) to a raw image data (bytes). \n
 * The number of channels remains unchanged.
 * @param iplImg OpenCv IplImage
 * @param (out) data Raw Image data
 */  
void convertIplToBytes(const IplImage *iplImg, std::vector<unsigned char>& data)
	throw(runtime_error);
	
/**
 * @brief Swap red and blue channel.
 * @param img Image
 */
void SwapRedBlueChannel(Video::Image & img);

/**
 * @brief Prune image area.
 * @param iplImg_src Source IplImage with higher resolution
 * @param iplImg_dst Destination IplImage
 * @param width Width of pruning area
 * @param height Height of pruning area
 * @param offsetX Offset of x-coordinate of pruning area
 * @param offsetY Offset of y-coordinate of pruning area
 */
void PruneImageArea(IplImage  *iplImg_src, IplImage & iplImg_dst, int width, int height, int offsetX, int offsetY);

IplImage* crop( IplImage* src,  CvRect& roi);
}

#endif

