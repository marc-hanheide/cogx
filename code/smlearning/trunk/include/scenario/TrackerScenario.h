/** @file TrackerScenario.h
 *
 * Tracker
 *
 * @author	Marek Kopicki (The University Of Birmingham)
 * @author      Sergio Roa (DFKI)
 *
 * @version 1.0
 *
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SMLEARNING_TRACKERSCENARIO_H
#define SMLEARNING_TRACKERSCENARIO_H

// #include <GL/glew.h>

#include <scenario/Scenario.h>
#include <scenario/Polyflap.h>
#include <scenario/Box.h>

#include <boost/function.hpp>

// TODO a proper fix to the warnings!
#include <scenario/TrackerThread.h>
#include <TomGine/tgFont.h>
#include <smltools/tracker_tools.h>
#include <metalearning/GNGSMRegion.h>

#include <Golem/Phys/Application.h>
#include <Golem/Device/Katana300/Katana300.h>

// using namespace std;
// using namespace golem;
namespace po = boost::program_options;

namespace smlearning {

class TrackerScenario : public Scenario
{
public:
	
	
	/** Object description */
	class Desc : public Scenario::Desc {
		
	public:		
		/** Constructs description object */
		Desc() 
		{
			Desc::setToDefault();
		}
		/** Tracker config file */
		std::string trackerConfig;
		/** Camera calibration file */
		std::string cameraCalibrationFile;
		/** Pose calibration file */
		std::string poseCalibrationFile;
		/** font */
		TomGine::tgFont font;
		/** Checks if the description is valid. */
		virtual bool isValid() const 
		{
			return (Scenario::Desc::isValid());
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			Scenario::Desc::setToDefault();
				
			// camera calibration
			cameraCalibrationFile = "cam.cal";
			// pose calibration
			poseCalibrationFile = "pose.cal";
			// tracker
			trackerConfig = "tracking.ini";
		}

	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(TrackerScenario, golem::Object::Ptr, golem::Scene&)
		
	};
	
	/** cto, such that Scenario can only be created with(in) a scene (context) [requirement of golem]*/
	TrackerScenario(golem::Scene&);
	/** destructor */
	~TrackerScenario ();
	/** set experiment default values */
	virtual void init(boost::program_options::variables_map vm);
	/** Creates Capture from description. */
	bool create(const TrackerScenario::Desc& desc);
	// /** Runs main task */
	// virtual void tracking();
	/** Finish */
	virtual void finish();
	/** Run experiment */
	virtual void run(int argc, char* argv[]);
	/** get class name */
	static string getName () { return "TrackerScenario"; }
protected:
	// /** Releases resources */
	// virtual void release() {};
	/** select a random action */
	virtual void chooseAction ();
	/** calculate the start coordinates of the arm */
	virtual void calculateStartCoordinates ();
	/** Describe the experiment trajectory */
	virtual void initMovement();
	/** Renders the object. */
        virtual void render();
	/** (Post)processing function called AFTER every physics simulation step and before rendering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** write data chunk (used in postprocess function) */
	virtual void writeChunk (LearningData::Chunk& chunk);
	/** experiment initialization (setting arm and object) */
	void initExperiment ();
	/** close arm gripper */
	void closeGripper(Katana300Arm &arm);
	/** object object pose in Golem simulation */
	void updateObjectPoseSimulation ();
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	/** Description */
	TrackerScenario::Desc desc;
	// /** Tracker thread */
	TrackerThread* tracker_th;
	/** Event handling (Pause) */
	golem::Event evContinue;
	/** katana arm */
	Katana300Arm* pKatana300Arm;
	/** default object bounds */
	golem::Bounds::SeqPtr objectLocalBounds;
	/** current iteration */
	int iteration;


};

class TrackerScenarioPredictionSingle : public TrackerScenario
{
public:
	/** Object description */
	class Desc : public TrackerScenario::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(TrackerScenarioPredictionSingle, golem::Object::Ptr, golem::Scene&)

	public:
		/** Constructs description object */
		Desc() {
			TrackerScenario::Desc::setToDefault();
		}
	};
	/** cto, such that Scenario can only be created with(in) a scene (context) [requirement of golem]*/
	TrackerScenarioPredictionSingle(golem::Scene&);
	/** destructor */
	~TrackerScenarioPredictionSingle () {};
	/** Run experiment */
	virtual void run(int argc, char* argv[]);
	/** set experiment default values */
	virtual void init(boost::program_options::variables_map vm);
protected:
	// /** Renders the object. */
	virtual void render();
	/** (Post)processing function called AFTER every physics simulation step and before rendering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** update error in output prediction */
	void updateAvgError ();
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	/** normalization function */
	boost::function<float (const float&, const float&, const float&)> normalization;
	/** denormalization function */
	boost::function<float (const float&, const float&, const float&)> denormalization;
	/** Encapsulation of CrySSMEx components */
	CrySSMEx cryssmex;
	/** method for feature selection */
	unsigned int featureSelectionMethod;
	/** average error in prediction */
	vector<double> avgerrors;


};

class TrackerScenarioPredictionEnsemble : public TrackerScenarioPredictionSingle
{
public:
	/** Object description */
	class Desc : public TrackerScenarioPredictionSingle::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(TrackerScenarioPredictionEnsemble, golem::Object::Ptr, golem::Scene&)

	public:
		/** Constructs description object */
		Desc() {
			TrackerScenarioPredictionSingle::Desc::setToDefault();
		}
	};
	/** cto, such that Scenario can only be created with(in) a scene (context) [requirement of golem]*/
	TrackerScenarioPredictionEnsemble(golem::Scene&);
	/** destructor */
	~TrackerScenarioPredictionEnsemble () {};
	/** set experiment default values */
	virtual void init(boost::program_options::variables_map vm);
protected:
	/** (Post)processing function called AFTER every physics simulation step and before rendering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** Describe the experiment trajectory and update current Region */
	virtual void initMovement();
	/** map of indexed sensorimotor regions */
	GNGSMRegion::RegionsMap regions;
	/** current context Region */
	GNGSMRegion* currentRegion;

};

bool XMLData(TrackerScenario::Desc&, XMLContext*, Context*);
bool XMLData(TrackerScenarioPredictionSingle::Desc&, XMLContext*, Context*);
bool XMLData(TrackerScenarioPredictionEnsemble::Desc&, XMLContext*, Context*);

/** TrackerScenarioApp */
class TrackerScenarioApp : public golem::Application {
public:
	/** Program options and main app initialization */
	virtual int main(int argc, char *argv[]);
protected:
	/** Runs TrackerScenarioApp */
	virtual void run(int argc, char *argv[]);
	/** options map */
	boost::program_options::variables_map vm;
	/** options description */
	boost::program_options::options_description prgOptDesc;
};


} // namespace smlearning

#endif /* SMLEARNING_TRACKERSCENARIO_H */
