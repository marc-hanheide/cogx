package motivation.components.generators;

import java.util.Map;
import java.util.HashMap;
import java.util.Map.Entry;

import motivation.slice.TutorInitiativeMotive;
import nu.xom.Document;
import nu.xom.Node;
import nu.xom.Nodes;
import autogen.Planner.Goal;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

@SuppressWarnings("serial")
public class DoraIntentionGoalGenerator extends
		AbstractIntentionMotiveGenerator<TutorInitiativeMotive, Intention> {

	public final static String XPATH_SELECT_PRECONDITIONS = "/*/content/*/preconditions/forms/*";
	public final static String XPATH_SELECT_POSTCONDITIONS = "/*/content/*/preconditions/forms/*";
	public final static String XPATH_SELECT_POINTER_PREFIX = "form/pointer";
	private static final Map<String, String> DLG_TO_PLANNER_MAPPING = new HashMap<String, String>() {
		{
			put("box", "cornflakes");
		}
	};

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */

	public DoraIntentionGoalGenerator() {
		super(TutorInitiativeMotive.class, Intention.class);
	}

	@Override
	protected TutorInitiativeMotive checkForAddition(WorkingMemoryAddress addr,
			Intention newEntry) {
		if (newEntry.estatus instanceof AttributedEpistemicStatus) {
			// only attributed beliefs contain intentions
			Document xmlDoc = IceXMLSerializer.toXomDom(newEntry);
			// get intention

			Nodes preconditions = xmlDoc.query(XPATH_SELECT_PRECONDITIONS);
			WorkingMemoryAddress preBelief = findAttributedPrecondition(preconditions);

			CASTIndependentFormulaDistributionsBelief<dBelief> preAttributedBelief;
			try {
				preAttributedBelief = CASTIndependentFormulaDistributionsBelief
						.create(dBelief.class, getMemoryEntry(preBelief,
								dBelief.class));
				String concept = null;
				// look for "unknown" values in all the properties:
				for (Entry<String, FormulaDistribution> f : preAttributedBelief
						.getContent().entrySet()) {
					String mostLikely = f.getValue().getDistribution()
							.getMostLikely().getProposition();
					if (mostLikely.equals("unknown")) {
						concept = f.getKey();
						break;
					}
				}
				if (concept == null) {
					getLogger()
							.warn(
									"the concept in the attributed belief was none of the ones we know about.");
					return null;
				}
				log("inferred concept from attributed belief: " + concept);
				if (concept.equals("location")) {
					FormulaDistribution objecttypeFD = preAttributedBelief
							.getContent().get("objecttype");
					if (objecttypeFD == null) {
						getLogger()
								.warn(
										"couldn't find any referring pointer, most likelz ref resolution didn't work");
						return null;
					}

					String objectType = objecttypeFD.getDistribution()
							.getMostLikely().getProposition();

					objectType = DLG_TO_PLANNER_MAPPING.get(objectType);

					TutorInitiativeMotive goal = new TutorInitiativeMotive();
					super.fillDefault(goal);
					
					goal.referenceEntry = addr;
					// (and (exists (?o - visualobject) (and (= (label ?o)
					// cereal_box) (kval dora (is-in ?o))))
					String robotBel = getRobotBeliefAddr().id;
					goal.goal = new Goal(-1,
							"(exists (?o - visualobject) (and (= (label ?o) "
									+ objectType + ") (kval '" + robotBel
									+ "' (is-in ?o))))", false);
					log("goal generated: " + goal.goal.goalString);
					return goal;
				} else {
					return null;
				}
			} catch (CASTException e) {
				logException(e);
			}

		}
		return null;
	}

	private WorkingMemoryAddress findAttributedPrecondition(Nodes preconditions) {
		for (int i = 0; i < preconditions.size(); i++) {
			Node n = preconditions.get(i);
			ModalFormula mf = IceXMLSerializer.fromXMLString(n.toXML(),
					ModalFormula.class);
			if (!mf.op.equals("belief"))
				continue;
			WorkingMemoryAddress pf = ((PointerFormula) mf.form).pointer;
			CASTIndependentFormulaDistributionsBelief<dBelief> preAttributedBelief;
			try {
				preAttributedBelief = CASTIndependentFormulaDistributionsBelief
						.create(dBelief.class,
								getMemoryEntry(pf, dBelief.class));
				if (preAttributedBelief.isAttributed()) {
					return pf;
				}
			} catch (CASTException e) {
				logException(e);
				return null;
			}
		}
		return null;
	}

	@Override
	protected TutorInitiativeMotive checkForUpdate(Intention newEntry,
			TutorInitiativeMotive motive) {
		motive.updated = getCASTTime();
		// initially this costs are taken as -1, corresponding to an ultimate
		// goal.
		motive.costs = -1;
		motive.informationGain = 1;

		log("updated goal to " + motive.goal.goalString);
		return motive;
	}

}
