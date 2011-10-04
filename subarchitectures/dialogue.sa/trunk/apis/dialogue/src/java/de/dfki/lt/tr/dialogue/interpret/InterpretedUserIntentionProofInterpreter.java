package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.atoms.AgentAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.IntentionIDAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.StringContentAtom;
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
	protected InterpretedUserIntention interpretWithoutWellFormednessTest(List<ModalisedAtom> matoms) {
		final InterpretedUserIntention iui = new InterpretedUserIntention();

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
				return new Runnable() {
					@Override
					public void run() {
						getLogger().debug("setting key-value pair");
						iui.addStringContent(key, value);
					}
				};
			}

		};

		fireForFirst(matoms, itIntID);
		fireForFirst(matoms, itAgName);
		fireForAll(matoms, itStringContent);

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

}
