#include <Ice/Ice.h>

#include <cast/core/CASTComponent.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <PointCloudClient.h>
#include <CDisplayClient.hpp>

#include <highgui.h>
#include <cvaux.h>

namespace cast {

/**
 * Written by Marc Hanheide based on code by Costas Cristofi at Birmingham.
 */
class PCPeopleDetector: public ManagedComponent, public cast::PointCloudClient {

	static const float DETECTWINDOW_PADDING_PROPORTION=0.2;
	static const float FACE_DIAMETER=0.2;

	/**
	 * How many frames to use when triggered on demand. Default 5.
	 */
	unsigned int m_numDetectionAttempts;

	/**
	 * Whether to run continuously or not. Default false.
	 */
	bool m_runContinuously;

	/** maximum distance to see people */
	double maxDist;

	cogx::display::CDisplayClient m_display;
public:
	/**
	 * Constructor
	 */
	PCPeopleDetector();

	/**
	 * Run the people detector for a single frame.
	 */
	void runDetection();

	/**
	 * Do detection on demand.
	 */
	void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);
    static const float MIN_RATIO = 0.7;
    static const float MAX_RATIO = 1.2;

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
