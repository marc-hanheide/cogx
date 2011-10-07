package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.dialogue.interpret.MatcherUtils;
import de.dfki.lt.tr.dialogue.interpret.TermParsingException;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.List;

public class QUDOpenAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "wh_question_under_discussion";

	private final WorkingMemoryAddress qudAddr;
	private final String nominal;
	private final WorkingMemoryAddress thisIntAddr;
	private final String feature;

	public QUDOpenAtom(WorkingMemoryAddress qudAddr, String nominal, WorkingMemoryAddress thisIntAddr, String feature) {
		this.qudAddr = qudAddr;
		this.nominal = nominal;
		this.thisIntAddr = thisIntAddr;
		this.feature = feature;
	}

	public WorkingMemoryAddress getQUDAddress() {
		return qudAddr;
	}
	public String getNominal() {
		return nominal;
	}

	public WorkingMemoryAddress getIntentionAddress() {
		return thisIntAddr;
	}

	public String getFeature() {
		return feature;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(new Modality[] {
					Modality.Truth
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					qudAddr == null ? TermAtomFactory.var("QUDAddr") : ConversionUtils.workingMemoryAddressToTerm(qudAddr),
					nominal == null ? TermAtomFactory.var("Nom") : TermAtomFactory.term(nominal),
					thisIntAddr == null ? TermAtomFactory.var("IntAddr") : ConversionUtils.workingMemoryAddressToTerm(thisIntAddr),
					feature == null ? TermAtomFactory.var("Feature") : TermAtomFactory.term(feature)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<QUDOpenAtom> {

		@Override
		public QUDOpenAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.contains(PRED_SYMBOL) && matom.a.args.size() == 4) {

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress qudAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(0));
					String nominal = MatcherUtils.parseTermToString(args.get(1));
					WorkingMemoryAddress intAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(2));
					String feature = MatcherUtils.parseTermToString(args.get(3));
					return new QUDOpenAtom(qudAddr, nominal, intAddr, feature);
				}
				catch (TermParsingException ex) {
					return null;
				}

			}
			return null;
		}
		
	}

}
