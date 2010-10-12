/**
 * @author Michael Zillich
 * @date Februrary 2009
 */

#ifndef OPEN_CV_IMG_SEQ_SERVER_H
#define OPEN_CV_IMG_SEQ_SERVER_H

#include <string>
#include <vector>
#include <stdexcept>
#include <cv.h>
//#include <opencv/cv.h>  -- this is incorrect according to the pkg-config flags
#include "VideoServer.h"

namespace cast
{

/**
 * Video device reading from stored files.
 * A user supplied frame rate simlulates the synchronisation of a real video
 * device.
 */
class OpenCvImgSeqServer : public VideoServer
{
private:
  // filenames, e.g. for 2 cameras:
  // (file0-0 file1-0 file0-1 file1-1 file0-2 file1-2 ... )
  std::vector<std::string> filenames;
  /**
   * raw Ipl images
   */
  std::vector<IplImage*> grabbedImages;
  /**
   * time stamps when Ipl images were captured.
   */
  std::vector<cast::cdl::CASTTime> grabTimes;
  int frameCnt;
  int framerateMillis;
  int width, height;
  /**
   * Whether to loop image sequence or return empty images at end of sequence.
   */
  bool loopSequence;

  // Number of times each frame is repeated; default = 1
  int frameRepeatCnt;
  int frameRepeatPos;

  /**
   * Integer factor for downsampling.
   * NOTE: ignored for now!
   */
  int downsampleFactor;

  /**
   * Initialise with a filename template and frame numbers.
   * If there are several cameras, each camera has its own file template,
   * e.g. img_left_%03d.jpg img_right_%03d.jpg
   * \param file_templates  (in) printf-style template string for filename,
   *                             e.g. "data/img%02d.jpg" for data/img00.jpg, data/img01.jpg
   * \param first (in) first frame number, to start e.g. with data/img04.jpg
   * \param last  (in) last frame number
   * \param inc   (in) frame number increment (default 1)
   */
  void init(const std::vector<std::string> &fileTemplates,
      int first, int last, int inc) throw(std::runtime_error);
  void constructFilenames(const std::vector<std::string> &fileTemplates,
      int first, int last, int inc);
  void obtainImageSize() throw(std::runtime_error);
  int numFrames() {return filenames.size()/getNumCameras();}
  /**
   * Return whether we have ever grabbed any frames.
   */
  bool haveFrames();
  void grabFramesInternal() throw(std::runtime_error);
  void retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame);
  virtual void retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrames(int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);
	virtual void retrieveHRFrames(std::vector<Video::Image> &frames);


public:
  OpenCvImgSeqServer();
  virtual ~OpenCvImgSeqServer();
  virtual void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);
  virtual void grabFrames();
  virtual void getImageSize(int &width, int &height);
  virtual int getFramerateMilliSeconds();
	virtual void changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize);
	virtual bool inFormat7Mode();
	virtual const std::string getServerName();
};

}

#endif

