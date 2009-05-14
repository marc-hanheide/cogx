#ifndef __FL_RECOGNISER_HH__
#define __FL_RECOGNISER_HH__

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <vision/utils/SceneObjectWriter.hpp>

//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace Vision;
using namespace cast;
using namespace boost;
using namespace Matlab; // Matrix

typedef map<string, shared_ptr<const CASTData<Vision::ROI> > > ROIMap;
typedef vector<shared_ptr<const CASTData<Vision::ROI> > > ROIVector;
typedef map<string, shared_ptr<const CASTData<Vision::LearnInstruction> > > LearnInstructionMap;

class Recogniser : public ManagedProcess, public SceneObjectWriter
{
	public:
		Recogniser(const string &_id);
		virtual ~Recogniser();

		virtual void runComponent();
		virtual void configure(map<string,string> & _config);
		virtual void start();
                virtual void stop();

	protected:
		virtual void taskAdopted(const string &_taskID);
		virtual void taskRejected(const string &_taskID);

	private:
		void proposeRecognitionGoals();
		void proposeLearnGoal(const cdl::WorkingMemoryChange& _wmChange);
		void processROI(const cdl::WorkingMemoryChange& _wmChange);
		void removeROI(const cdl::WorkingMemoryChange& _wmChange);
		void recognise(shared_ptr<const CASTData<Vision::ROI> > _roiData);
		void update(shared_ptr<const CASTData<Vision::LearnInstruction> > _liData);
		long getColour(Vision::RecognisedAttributes* _attr);
		long getShape(Vision::RecognisedAttributes* _attr);
                long getSize(Vision::RecognisedAttributes* _attr);

    /**
     * This method is called whenever the internal representation is changed.
     * The default behaviour is to request attribute re-recognition of all ROIs
     * in WM.
     */
		void onRepresentationChange();

    /**
     * Load the initial AV models.
     */
    void loadAVModels();

		map<string, string>* objRoiIDs_;
		ROIMap* recognitionTasks_;
		LearnInstructionMap* learnTasks_;

    std::string m_AVModel;
    int m_matWindow;
}; // class Recogniser

#endif
