/** @file Scenario.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include "Prediction/Scenario.h"

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Experiment::Experiment(golem::Scene &scene) : Creator(scene) {
	Experiment::setToDefault();
}

void Experiment::run(Performer &performer) {
	if (!isValid()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR,
			"Experiment::run(): invalid experiment description"
		);
		return;
	}
	
	const U32 numOfScenarios = (U32)scenarios.size();
	const U32 totalNumOfRuns = numOfRuns*numOfScenarios;

	for (U32 i = 0; i < totalNumOfRuns; i++) {
		if (scene.getUniverse().interrupted())
			return;
		
		const U32 n = randomRuns ? rand.next()%numOfScenarios : i%numOfScenarios;
		Scenario &scenario = scenarios[n];

		if (!scenario.isValid()) {
			context.getMessageStream()->write(Message::LEVEL_ERROR,
				"Experiment::run(): invalid scenario description"
			);
			continue;
		}
		
		context.getMessageStream()->write(Message::LEVEL_INFO,
			"#%.4d: scenario: %s", i + 1, scenario.name.c_str()
		);
		performer.run(scenario);
	}
}


//------------------------------------------------------------------------------
