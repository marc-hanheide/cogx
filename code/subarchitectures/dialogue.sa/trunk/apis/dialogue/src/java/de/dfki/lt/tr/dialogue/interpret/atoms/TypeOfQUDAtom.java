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

public class TypeOfQUDAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "type_of_question_under_discussion";

//	private final String speaker;
//	private final String hearer;
	private final WorkingMemoryAddress qudAddr;
//	private final String nominal;
	private final WorkingMemoryAddress thisIntAddr;
//	private final String feature;
//	private final String value;
//	private final String polarity;
	private final String type;

	public TypeOfQUDAtom(WorkingMemoryAddress qudAddr, WorkingMemoryAddress thisIntAddr, String type) {
//		this.speaker = speaker;
//		this.hearer = hearer;
		this.qudAddr = qudAddr;
//		this.nominal = nominal;
		this.thisIntAddr = thisIntAddr;
//		this.feature = feature;
//		this.value = value;
//		this.polarity = polarity;
		this.type = type;
	}

	public WorkingMemoryAddress getQUDAddress() {
		return qudAddr;
	}

	public WorkingMemoryAddress getIntentionAddress() {
		return thisIntAddr;
	}

	public String getType() {
		return type;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(new Modality[] {
					Modality.Truth
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
//					speaker == null ? TermAtomFactory.var("Speaker") : TermAtomFactory.term(speaker),
//					hearer == null ? TermAtomFactory.var("Hearer") : TermAtomFactory.term(hearer),
					qudAddr == null ? TermAtomFactory.var("QUDAddr") : ConversionUtils.workingMemoryAddressToTerm(qudAddr),
//					nominal == null ? TermAtomFactory.var("Nom") : TermAtomFactory.term(nominal),
					thisIntAddr == null ? TermAtomFactory.var("IntAddr") : ConversionUtils.workingMemoryAddressToTerm(thisIntAddr),
//					feature == null ? TermAtomFactory.var("Feature") : TermAtomFactory.term(feature),
//					value == null ? TermAtomFactory.var("Value") : TermAtomFactory.term(value),
//					polarity == null ? TermAtomFactory.var("Polarity") : TermAtomFactory.term(polarity)
					type == null ? TermAtomFactory.var("Type") : TermAtomFactory.term(type)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<TypeOfQUDAtom> {

		@Override
		public TypeOfQUDAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.contains(PRED_SYMBOL) && matom.a.args.size() == 3) {

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress qudAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(0));
					WorkingMemoryAddress intAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(1));
					String type = MatcherUtils.parseTermToString(args.get(2));
					return new TypeOfQUDAtom(qudAddr, intAddr, type);
				}
				catch (TermParsingException ex) {
					return null;
				}

			}
			return null;
		}
		
	}

}
