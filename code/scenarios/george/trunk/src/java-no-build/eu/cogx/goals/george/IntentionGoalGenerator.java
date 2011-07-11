package eu.cogx.goals.george;

import org.cognitivesystems.binder.POINTERLABEL;

import motivation.components.generators.AbstractIntentionMotiveGenerator; 
import motivation.slice.TutorInitiativeMotive;
import nu.xom.Document;
import nu.xom.Node;
import nu.xom.Nodes;

import autogen.Planner.Goal;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;

import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

public class IntentionGoalGenerator extends
		AbstractIntentionMotiveGenerator<TutorInitiativeMotive, Intention> {

	public final static String XPATH_SELECT_PRECONDITIONS = "/*/content/*/*/preconditions/forms/*";
	public final static String XPATH_SELECT_POSTCONDITIONS = "/*/content/*/*/postconditions/forms/*";
	public final static String XPATH_SELECT_POINTER_PREFIX = "form/pointer";
	private final static String[] CONCEPTS_TO_PARSE = new String[] { "color",
			"shape" };
	public final static String XPATH_SELECT_NEGATION = "/*/content/distribs/entry/*/*/*/*/*/negForm";		

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */

	public IntentionGoalGenerator() {
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
			if (preBelief==null)
				return null;
			CASTIndependentFormulaDistributionsBelief<dBelief> preAttributedBelief;
			try {			
				dBelief belief = getMemoryEntry(preBelief, dBelief.class);
				preAttributedBelief = CASTIndependentFormulaDistributionsBelief
						.create(dBelief.class, belief);
				String concept = null;
				for (String c : CONCEPTS_TO_PARSE) {
					if (preAttributedBelief.getContent().get(c) != null)
						concept = c;
				}
				if (concept == null) {
					getLogger()
							.warn(
									"the concept in the attributed belief was none of the ones we know about.");
					return null;
				}
				
				String prefix="";
				Document xmlAttr = IceXMLSerializer.toXomDom(belief);
				Nodes negations = xmlAttr.query(XPATH_SELECT_NEGATION);
			
				
				if (negations.size() == 1) {
					prefix="un";			
					
				}
				
				log("inferred concept from attributed belief: " + concept);
				FormulaDistribution aboutPointerFD = preAttributedBelief
						.getContent().get(POINTERLABEL.value); 
				if (aboutPointerFD == null) {
					getLogger()
							.warn(
									"couldn't find any referring pointer, most likely ref resolution didn't work");
					return null;
				}
				WMPointer referredPrivate = WMPointer
						.create(aboutPointerFD.getDistribution()
								.getMostLikely().get());
				log("got referred private belief: "
						+ referredPrivate.getVal().id);

				TutorInitiativeMotive goal = new TutorInitiativeMotive();
				super.fillDefault(goal);
				goal.referenceEntry = addr;
				goal.goal = new Goal(-1, "(" + concept.toLowerCase()
						+ "-" + prefix + "learned '" + referredPrivate.getVal().id + "')",
						false);
				log("goal generated: " + goal.goal.goalString);

				//HACK unlearning always has a gain of 1
				if (negations.size() == 1) {
					goal.informationGain = 1;				
				}
				//END HACK

				return goal;
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
