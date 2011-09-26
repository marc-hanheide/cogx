#include <Ice/Ice.h>

#include <cast/core/CASTComponent.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <PointCloudClient.h>
#include <CDisplayClient.hpp>


#include <highgui.h>
#include <cvaux.h>

namespace cast {

/**
 * Written by Costas Cristofi and Marc Hanheide at Birmingham.
 */
class PCPeopleDetector: public ManagedComponent, public cast::PointCloudClient {
	bool deinterlacing;
	int sleepForSync;
	double xscale, xcentre;
	double trackerThreshold;
	int removeAfterFrames;
	double removeAfterDistance;

	/**
	 * How many frames to use when triggered on demand. Default 5.
	 */
	unsigned int m_numDetectionAttempts;

	/**
	 * Whether to run continuously or not. Default false.
	 */
	bool m_runContinuously;

	cogx::display::CDisplayClient m_display;
public:
	/**
	 * Constructor
	 */
	PCPeopleDetector();

	/**
	 * Destructor
	 */
	~PCPeopleDetector();

	/**
	 * Run the people detector for a single frame.
	 */
	void runDetection();

	/**
	 * Do detection on demand.
	 */
	void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);

protected:

	virtual void configure(const std::map<std::string, std::string> & config);
	virtual void start();
	virtual void runComponent();

protected:

	CvMemStorage * upper_storage;
	CvHaarClassifierCascade * upper_cascade;
	CvMemStorage * face_storage;
	CvHaarClassifierCascade * face_cascade;
	CvMemStorage * fullbody_storage;
	CvHaarClassifierCascade * fullbody_cascade;
};

}
