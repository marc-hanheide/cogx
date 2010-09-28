package eu.cogx.goals.george;

import java.util.ArrayList;
import java.util.List;

import motivation.components.generators.AbstractIntentionMotiveGenerator;
import motivation.slice.GeneralGoalMotive;
import nu.xom.Document;
import nu.xom.Element;
import nu.xom.Node;
import nu.xom.Nodes;
import VisionData.VisualObject;
import autogen.Planner.Goal;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class IntentionGoalGenerator extends
		AbstractIntentionMotiveGenerator<GeneralGoalMotive, Intention> {

	public final static String XPATH_SELECT_FORMULAS = "/*[estatus/attribagents/string/text()='human']/content/values/values/*/val/forms";
	public final static String XPATH_SELECT_REFERRED_OBJECT = "*[op/text()='LingRef']/form/prop/text()";
	public final static String XPATH_SELECT_COLOR = "*[op/text()='Color']/form/prop/text()";
	public final static String XPATH_SELECT_SHAPE = "*[op/text()='Shape']/form/prop/text()";

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */

	public IntentionGoalGenerator() {
		super("fact", GeneralGoalMotive.class, Intention.class);
	}

	@Override
	protected GeneralGoalMotive checkForAddition(WorkingMemoryAddress addr,
			Intention newEntry) {
		if (newEntry.estatus instanceof AttributedEpistemicStatus) {
			// only attributed beliefs contain intentions
			Document xmlDoc = IceXMLSerializer.toXomDom(newEntry);
			// get intention
			// TODO: something smart has to be done here...
			// I have to access from the struct
			// ... which object is referred to (the GroundedBelief)
			// ... which concept is given (Colour, Shape)
			// ... have to check whether this is new information or not

			Element attributedForms = (Element) queryFirst(xmlDoc,
					XPATH_SELECT_FORMULAS);
			if (attributedForms == null) {
				getLogger().warn(
						"unable to find expected formulas in "
								+ xmlDoc.toString());
				return null;
			}
			log("found belief with forms in it: " + attributedForms.toString());
			String objectId = queryFirst(attributedForms,
					XPATH_SELECT_REFERRED_OBJECT).getValue();
			if (objectId == null) {
				getLogger().warn(
						"unable to referred object in formulas "
								+ attributedForms.toString());
				return null;
			}

			// HACK TO FIND A VALID OBJECT
			List<GroundedBelief> entries = new ArrayList<GroundedBelief>();
			try {
				getMemoryEntries(GroundedBelief.class, entries,"binder");
				for (GroundedBelief belief : entries) {
					if (belief.type.equals(SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(VisualObject.class))) {
						objectId = belief.id;
						log("found an object grounded belief to fake reference resolution");
					}
				}
			} catch (UnknownSubarchitectureException e) {
				logException(e);
			}
			// END OF HACK

			String concept = "";
			String prop = "";

			Node color = queryFirst(attributedForms, XPATH_SELECT_COLOR);
			Node shape = queryFirst(attributedForms, XPATH_SELECT_SHAPE);
			if (color != null) {
				concept = "Color";
				prop = color.getValue();
			} else if (shape != null) {
				concept = "Shape";
				prop = shape.getValue();
			}
			log("attributed information to generate goal for concept "
					+ concept + ", prop= " + prop);

			GeneralGoalMotive goal = new GeneralGoalMotive();
			super.fillDefault(goal);
			goal.referenceEntry = addr;
			goal.goal = new Goal(-1, "(been-used-for-learning-sample '"
					+ objectId + "' " + concept + ")", false);
			log("goal generated: " + goal.goal.goalString);
			return goal;
		}
		return null;
	}

	private Node queryFirst(Node node, String xpath) {
		Nodes res = node.query(xpath);
		if (res.size() != 1)
			return null;
		else
			return res.get(0);
	}

	@Override
	protected GeneralGoalMotive checkForUpdate(Intention newEntry,
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
