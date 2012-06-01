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

// TODO a proper fix to the warnings!
#include <Tracker/Tracker.h>
#include <TomGine/tgFont.h>
#include <GLWindow/GLWindow.h>

#include <Golem/Phys/Application.h>

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
		TomGine::tgFont *font;
		string fontName;
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
			trackerConfig = "Tracking.ini";
			// font
			fontName = "/usr/local/bin/SMLearning/FreeSerifBold.ttf";
			font = new TomGine::tgFont (fontName.c_str());
		}

	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(TrackerScenario, golem::Object::Ptr, golem::Scene&)
		
	};
	
	/** cto, such that Scenario can only be created with(in) a scene (context) [requirement of golem]*/
	TrackerScenario(golem::Scene&);
	/** destructor */
	~TrackerScenario () {}
	/** set experiment default values */
	virtual void init(boost::program_options::variables_map vm);
	/** Creates Capture from description. */
	bool create(const TrackerScenario::Desc& desc);
	/** Runs main task */
	virtual void main();
	/** Finish */
	virtual void finish();
	/** Run experiment */
	void run(int argc, char* argv[]);
	/** get class name */
	static string getName () { return "TrackerScenario"; }
protected:
	// /** Renders the object. */
	// virtual void render() {};
	// /** Releases resources */
	// virtual void release() {};
	// /** select a random action */
	// virtual void chooseAction () {};
	// /** calculate the start coordinates of the arm */
	// virtual void calculateStartCoordinates() {};
	// /** Describe the experiment trajectory */
	// virtual void initMovement() {};
	// /** (Post)processing function called AFTER every physics simulation step and before rendering. */
	// virtual void postprocess(golem::SecTmReal elapsedTime){};
	/** Description */
	TrackerScenario::Desc desc;
	/** Tracker */
	// golem::shared_ptr<Tracking::Tracker> m_tracker;
	Tracking::Tracker* m_tracker;
	/** Object model */
	TomGine::tgModel m_object;
	/* The path to load and save the ply model */
	std::string m_plypath;
	/** Captured camera Image */
	IplImage* _img;
	TomGine::tgPose m_track_pose, m_initialPose;
	/** Tracker window */
	golem::shared_ptr<blortGLWindow::GLWindow> glWindow;
	/** Model id */
	int m_trackpred_id, m_ground_id, m_track_id;
	/** is the current state of the object movement (fast / slow / still ) */
	Tracking::movement_state _movement;
	/** is the current quality of object visibilty ( ok / occluded / lost / locked ) */
	Tracking::quality_state _quality;
	/** is the current confidence of the model ( good / fair / bad ) */
	Tracking::confidence_state _confidence;
	/** Event handling */
	golem::Event ev;
	bool _quit;

};

bool XMLData(TrackerScenario::Desc&, XMLContext*, Context*);

/** TrackerScenarioApp */
class TrackerScenarioApp : public golem::Application {
protected:
	/** Runs TrackerScenarioApp */
	virtual void run(int argc, char *argv[]);
};


} // namespace smlearning

#endif /* SMLEARNING_TRACKERSCENARIO_H */
