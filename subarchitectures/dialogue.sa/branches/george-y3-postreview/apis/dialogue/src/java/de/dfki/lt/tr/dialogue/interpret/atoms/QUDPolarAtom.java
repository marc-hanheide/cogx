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

public class QUDPolarAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "polar_question_under_discussion";

	private final String speaker;
	private final String hearer;
	private final WorkingMemoryAddress qudAddr;
	private final String nominal;
	private final WorkingMemoryAddress thisIntAddr;
	private final String feature;
	private final String value;
	private final String polarity;

	public QUDPolarAtom(String speaker, String hearer, WorkingMemoryAddress qudAddr, String nominal, WorkingMemoryAddress thisIntAddr, String feature, String value, String polarity) {
		this.speaker = speaker;
		this.hearer = hearer;
		this.qudAddr = qudAddr;
		this.nominal = nominal;
		this.thisIntAddr = thisIntAddr;
		this.feature = feature;
		this.value = value;
		this.polarity = polarity;
	}

	public String getSpeaker() {
		return speaker;
	}

	public String getHearer() {
		return hearer;
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

	public String getValue() {
		return value;
	}

	public String getPolarity() {
		return polarity;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(new Modality[] {
					Modality.Truth
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					speaker == null ? TermAtomFactory.var("Speaker") : TermAtomFactory.term(speaker),
					hearer == null ? TermAtomFactory.var("Hearer") : TermAtomFactory.term(hearer),
					qudAddr == null ? TermAtomFactory.var("QUDAddr") : ConversionUtils.workingMemoryAddressToTerm(qudAddr),
					nominal == null ? TermAtomFactory.var("Nom") : TermAtomFactory.term(nominal),
					thisIntAddr == null ? TermAtomFactory.var("IntAddr") : ConversionUtils.workingMemoryAddressToTerm(thisIntAddr),
					feature == null ? TermAtomFactory.var("Feature") : TermAtomFactory.term(feature),
					value == null ? TermAtomFactory.var("Value") : TermAtomFactory.term(value),
					polarity == null ? TermAtomFactory.var("Polarity") : TermAtomFactory.term(polarity)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<QUDPolarAtom> {

		@Override
		public QUDPolarAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.contains(PRED_SYMBOL) && matom.a.args.size() == 8) {

				List<Term> args = matom.a.args;

				try {
					String speaker = MatcherUtils.parseTermToString(args.get(0));
					String hearer = MatcherUtils.parseTermToString(args.get(1));
					WorkingMemoryAddress qudAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(2));
					String nominal = MatcherUtils.parseTermToString(args.get(3));
					WorkingMemoryAddress intAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(4));
					String feature = MatcherUtils.parseTermToString(args.get(5));
					String value = MatcherUtils.parseTermToString(args.get(6));
					String polarity = MatcherUtils.parseTermToString(args.get(7));
					return new QUDPolarAtom(speaker, hearer, qudAddr, nominal, intAddr, feature, value, polarity);
				}
				catch (TermParsingException ex) {
					return null;
				}

			}
			return null;
		}
		
	}

}
