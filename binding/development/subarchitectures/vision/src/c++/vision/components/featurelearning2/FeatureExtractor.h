#ifndef __cast_FEATURE_EXTRACTOR_H__
#define __cast_FEATURE_EXTRACTOR_H__

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>

#include <vector>
#include <map>

//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace Vision; // ROI
using namespace cast;
using namespace boost;
using namespace Matlab; // Matrix

typedef map<string, shared_ptr<const CASTData<Vision::ROI> > > ROIMap;
typedef vector<shared_ptr<const CASTData<Vision::ROI> > > ROIVector;

class FeatureExtractor : public ManagedProcess {
	public:
		FeatureExtractor(const string &_id);
		virtual ~FeatureExtractor();

		virtual void runComponent();
		virtual void configure(map<string,string> & _config);
		virtual void start();
                virtual void stop();

	protected:
		virtual void taskAdopted(const string &_taskID);
		virtual void taskRejected(const string &_taskID);

		void extractFeatures(shared_ptr<const CASTData<Vision::ROI> > _pData);

	private:

		void newROI(const cdl::WorkingMemoryChange & _wmc);
//		void quip(CASTData<ComedyEssentials::Joke> *_pData);
//		string generatePunchline(const string &_setup);

		// Hashtable used to record the tasks we want to carry out
		ROIMap * m_pProposedProcessing;
		ROIVector * m_pROIs;
}; // class FeatureExtractor

#endif
