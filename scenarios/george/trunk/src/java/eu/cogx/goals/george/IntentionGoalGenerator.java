package eu.cogx.goals.george;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.GeneralGoalMotive;
import nu.xom.Document;
import nu.xom.Nodes;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

public class IntentionGoalGenerator extends
		AbstractBeliefMotiveGenerator<GeneralGoalMotive, dBelief> {

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */

	protected IntentionGoalGenerator(Class<GeneralGoalMotive> motiveClass,
			Class<dBelief> beliefClass) {
		super("fact", GeneralGoalMotive.class, dBelief.class);
	}

	@Override
	protected GeneralGoalMotive checkForAddition(WorkingMemoryAddress addr,
			dBelief newEntry) {
		if (newEntry.estatus instanceof AttributedEpistemicStatus) {
			// only attributed beliefs contain intentions
			Document xmlDoc = IceXMLSerializer.toXomDom(newEntry);
			// get intention
			// TODO: something smart has to be done here...
			// I have to access from the struct
			//  ... which object is referred to (the GroundedBelief)
			//  ... which concept is given (Colour, Shape)
			//  ... have to check whether this is new information or not
			Nodes nodes = xmlDoc.query("/*[estatus/agent/text()=\"me\"]/id/text()");
			GeneralGoalMotive goal = new GeneralGoalMotive();
			super.fillDefault(goal);
			goal.referenceEntry = addr;
			goal.goal = new Goal(-1, "(and)", false);
			return goal;
		}
		return null;
	}

	@Override
	protected GeneralGoalMotive checkForUpdate(dBelief newEntry,
			GeneralGoalMotive motive) {
		motive.updated = getCASTTime();
		// initially this costs are taken as -1, corresponding to an ultimate
		// goal.
		motive.costs = -1;
		motive.informationGain = 1;

		motive.goal = new Goal(computeImportance(motive),
				"(been-used-for-learning-sample '" + "DUMMY" + "' "
						+ "CONCEPT_TO_BE_INFERRED_FROM_INTENTION" + ")", false);
		log("updated goal to " + motive.goal.goalString);
		return motive;
	}

	private float computeImportance(GeneralGoalMotive motive) {
		return -1f;
	}

}
