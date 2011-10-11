package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.atoms.AddressContentAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.AgentAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.BeliefContentAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.IntentionIDAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.NewBeliefAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.StringContentAtom;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomInterpreter;
import java.util.List;
import org.apache.log4j.Logger;

public class InterpretedUserIntentionProofInterpreter
extends AbstractWellFormedTestingProofInterpreter<InterpretedUserIntention> {

	public InterpretedUserIntentionProofInterpreter(Logger logger) {
		super(logger);
	}

	@Override
	protected InterpretedUserIntention interpretWithoutWellFormednessTest(List<ModalisedAtom> matoms, double confidence) {
		final InterpretedUserIntention iui = new InterpretedUserIntention();
		double prob = AbducerUtils.weightToProb(confidence);
		getLogger().debug("confidence=" + confidence + "; therefore prob=" + prob);
		iui.setConfidence(prob);

		ModalisedAtomInterpreter<IntentionIDAtom, Runnable> itIntID
				= new ModalisedAtomInterpreter<IntentionIDAtom, Runnable>(new IntentionIDAtom.Matcher()) {

			@Override
			public Runnable actOn(IntentionIDAtom matchResult) {
				final WorkingMemoryAddress wma = matchResult.getAddress();
				if (wma == null) {
					return null;
				}
				return new Runnable() {
					@Override
					public void run() {
						getLogger().debug("setting working memory address");
						iui.setAddress(wma);
					}
				};
			}

		};

		ModalisedAtomInterpreter<AgentAtom, Runnable> itAgName
				= new ModalisedAtomInterpreter<AgentAtom, Runnable>(new AgentAtom.Matcher()) {

			@Override
			public Runnable actOn(AgentAtom matchResult) {
				final String agName = matchResult.getAgentName();
				if (agName == null) {
					return null;
				}
				return new Runnable() {
					@Override
					public void run() {
						getLogger().debug("setting agent");
						iui.setAgent(agName);
					}
				};
			}

		};

		ModalisedAtomInterpreter<StringContentAtom, Runnable> itStringContent
				= new ModalisedAtomInterpreter<StringContentAtom, Runnable>(new StringContentAtom.Matcher()) {

			@Override
			public Runnable actOn(StringContentAtom matchResult) {
				final String key = matchResult.getKey();
				final String value = matchResult.getValue();
				if (key == null || value == null) {
					return null;
				}
				if (!iui.getAddress().equals(matchResult.getIntentionWMA())) {
					return null;
				}

				return new Runnable() {
					@Override
					public void run() {
						getLogger().debug("setting key-value pair for string content: \"" + key + "\" -> \"" + value + "\"");
						iui.addStringContent(key, value);
					}
				};
			}

		};

		ModalisedAtomInterpreter<AddressContentAtom, Runnable> itAddressContent
				= new ModalisedAtomInterpreter<AddressContentAtom, Runnable>(new AddressContentAtom.Matcher()) {

			@Override
			public Runnable actOn(AddressContentAtom matchResult) {
				final String key = matchResult.getKey();
				final WorkingMemoryAddress value = matchResult.getValue();
				if (key == null || value == null) {
					return null;
				}
				return new Runnable() {
					@Override
					public void run() {
						getLogger().debug("setting key-value pair for address content: \"" + key + "\" -> " + wmaToString(value));
						iui.addAddressContent(key, value);
					}
				};
			}

		};

		ModalisedAtomInterpreter<NewBeliefAtom, Runnable> itBeliefEpst
				= new ModalisedAtomInterpreter<NewBeliefAtom, Runnable>(new NewBeliefAtom.Matcher()) {

			@Override
			public Runnable actOn(NewBeliefAtom matchResult) {
				final WorkingMemoryAddress beliefWma = matchResult.getBeliefAddress();
				final EpistemicStatus epst = matchResult.getEpistemicStatus();
				if (beliefWma == null || epst == null) {
					return null;
				}
				return new Runnable() {
					@Override
					public void run() {
						getLogger().debug("setting epistemic status for " + wmaToString(beliefWma) + ": " + BeliefIntentionUtils.epistemicStatusToString(epst) + "\"");
						iui.setBeliefEpistemicStatus(beliefWma, epst);
					}
				};
			}

		};

		ModalisedAtomInterpreter<BeliefContentAtom, Runnable> itBeliefContent
				= new ModalisedAtomInterpreter<BeliefContentAtom, Runnable>(new BeliefContentAtom.Matcher()) {

			@Override
			public Runnable actOn(BeliefContentAtom matchResult) {
				final WorkingMemoryAddress beliefWma = matchResult.getBeliefAddress();
				final String key = matchResult.getKey();
				final dFormula value = matchResult.getValue();
				if (beliefWma == null || key == null || value == null) {
					return null;
				}
				return new Runnable() {
					@Override
					public void run() {
						getLogger().debug("setting belief content for " + wmaToString(beliefWma) + ": \"" + key + "\" -> \"" + BeliefIntentionUtils.dFormulaToString(value) + "\"");
						iui.addBeliefContent(beliefWma, key, value);
					}
				};
			}

		};

		fireForFirst(matoms, itIntID);
		fireForFirst(matoms, itAgName);
		fireForAll(matoms, itStringContent);
		fireForAll(matoms, itAddressContent);
		fireForAll(matoms, itBeliefEpst);
		fireForAll(matoms, itBeliefContent);

		return iui;
	}

	private void fireForFirst(List<ModalisedAtom> matoms, ModalisedAtomInterpreter<?, Runnable> it) {
		for (ModalisedAtom matom : matoms) {
			Runnable result = it.matchAndActOn(matom);
			if (result != null) {
				result.run();
				return;
			}
		}
	}

	private void fireForAll(List<ModalisedAtom> matoms, ModalisedAtomInterpreter<?, Runnable> it) {
		for (ModalisedAtom matom : matoms) {
			Runnable result = it.matchAndActOn(matom);
			if (result != null) {
				result.run();
			}
		}
	}

	public static String wmaToString(WorkingMemoryAddress wma) {
		return "[" + wma.id + "," + wma.subarchitecture + "]";
	}

}
