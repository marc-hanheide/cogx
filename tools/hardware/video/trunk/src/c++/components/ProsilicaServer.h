/**
 * @author Andrzej Pronobis
 * @date IROS'11 deadline
 */

#ifndef PROSILICA_SERVER_H
#define PROSILICA_SERVER_H

#include <string>
#include <vector>
#include <stdexcept>
#include <sys/time.h>
#include "VideoServer.h"
#include "PvApi.h"

namespace cast
{

class ProsilicaServer: public VideoServer
{
private:
	/**
	 * Class for measuring how many things happen per second.
	 * author: Nick Hawes
	 */
	class Timer
	{
	public:
		Timer();
		void increment();
		double getRate() const
		{
			return rate;
		}
		bool rateChange() const
		{
			return sigChange;
		}

	private:
		int count;
		double rate;
		double lastRate;
		double changeThresh;
		timeval startTime;
		bool sigChange;
	};

public:

	ProsilicaServer();
	virtual ~ProsilicaServer();
	virtual void configure(const std::map<std::string, std::string> & _config)
			throw (std::runtime_error);
	virtual void grabFrames();
	virtual void getImageSize(int &width, int &height);
	virtual int getFramerateMilliSeconds();
	virtual void changeFormat7Properties(int width, int height, int offsetX,
			int offsetY, int mode, int paketSize);
	virtual bool inFormat7Mode();
	virtual const std::string getServerName();

	virtual void retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames);
	virtual void retrieveFrames(int width, int height, std::vector<Video::Image> &frames);
	virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);
	virtual void retrieveHRFrames(std::vector<Video::Image> &frames);


private:

	void init();
	bool setCameraAttributes(tPvHandle cam);
	void retrieveFrameInternal(int camIdx, int width, int height,
	    Video::Image &frame);



private:

        /* MTU */
        unsigned int _mtu;

        /** Camera Id */
        unsigned int _camId;

        /** Cam handle. */
        tPvHandle _camHandle;

        /** Frame buffer. */
        tPvFrame _frame;

        /** Mutex for protecting the Pv API. */
        pthread_mutex_t _pvMutex;

        cast::cdl::CASTTime grabTime;

        bool initDone;
};

}

#endif

/*
 /
 * the actual wrapped OpenCV captures.
 /
 std::vector<CvCapture*> captures;

 /
 * pointers to retrieved images, point to internal memory of the capture
 * device, do not delete!
 /
 std::vector<IplImage*> retrievedImages;

 /
 * time stamps when Ipl images were captured.
 /
 std::vector<cast::cdl::CASTTime> grabTimes;

 /
 * format for Bayer to RGB conversion, CV_COLORCVT_MAX for no conversion
 /
 int bayerCvt;

 int framerateMillis;
 CvSize captureSize;

 /
 * Timer to measure actual frame rate.
 /
 Timer timer;

 /
 * get video resolution for given camera
 * @param camIdx which camera
 * @param size video resolution
 /
 void getResolution(int camIdx, CvSize &size);
 /
 * set resolution for given camera
 * @param camIdx which camera
 * @param size requested video resolution, on exit contains the actually
 *        set resolution, which might differ dependig on the cameras
 *        cpabilities
 * @return true if the requested resolution could be set, false if another
 *        reslution was chosen
 /
 bool setResolution(int camIdx, CvSize &size);
 void init(int dev_class, const std::vector<int> &dev_nums,
 const std::string &bayer) throw(std::runtime_error);
 /
 * Converts an IplImage to a system Image format.
 /
 void copyImage(const IplImage *iplImg, Video::Image &img) throw(std::runtime_error);
 bool haveBayer() {return bayerCvt != CV_COLORCVT_MAX;}
 void grabFramesInternal();
 void retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame);
 virtual void retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames);
 virtual void retrieveFrames(int width, int height, std::vector<Video::Image> &frames);
 virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);
 virtual void retrieveHRFrames(std::vector<Video::Image> &frames);

 public:

 // void enableUndistortion(const float dist[4]);
 // void disableUndistortion();
 */
