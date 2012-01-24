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

public class GenerateReferringExpressionAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "reference_generation";

	private final WorkingMemoryAddress beliefAddr;
	private final Boolean shortNP;
	private final Boolean spatialRelation;
	private final String refEx;
	private final List<String> disabledProps;

	public GenerateReferringExpressionAtom(WorkingMemoryAddress beliefAddr, Boolean shortNP, Boolean spatialRelation, String refEx, List<String> disabledProps) {
		this.beliefAddr = beliefAddr;
		this.shortNP = shortNP;
		this.spatialRelation = spatialRelation;
		this.refEx = refEx;
		this.disabledProps = disabledProps;
	}

	public WorkingMemoryAddress getBeliefAddress() {
		return beliefAddr;
	}

	public Boolean getHasShortNP() {
		return shortNP;
	}

	public Boolean getHasSpatialRelation() {
		return spatialRelation;
	}

	public String getLocation() {
		return refEx;
	}

	public List<String> getDisabledProps() {
		return disabledProps;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Generation
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					beliefAddr == null ? TermAtomFactory.var("BeliefAddr") : ConversionUtils.workingMemoryAddressToTerm(beliefAddr),
					shortNP == null ? TermAtomFactory.var("ShortNP") : TermAtomFactory.term(shortNP == true ? "yes" : "no"),
					spatialRelation == null ? TermAtomFactory.var("SpatialRelation") : TermAtomFactory.term(spatialRelation == true ? "yes" : "no"),
					refEx == null ? TermAtomFactory.var("RefEx") : TermAtomFactory.term(refEx),
					disabledProps == null ? TermAtomFactory.var("DisabledProperties") : ConversionUtils.listStringsToTerm(disabledProps)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<GenerateReferringExpressionAtom> {

		@Override
		public GenerateReferringExpressionAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 5) {

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress beliefAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(0));
					Boolean hasShortNP = MatcherUtils.parseTermToBoolean(args.get(1));
					Boolean hasSpatialRelation = MatcherUtils.parseTermToBoolean(args.get(2));
					String refEx = MatcherUtils.parseTermToString(args.get(3));
					List<String> disabledProps = MatcherUtils.parseTermToListOfStrings(args.get(4));

					return new GenerateReferringExpressionAtom(beliefAddr, hasShortNP, hasSpatialRelation, refEx, disabledProps);
				}
				catch (TermParsingException ex) {
					return null;
				}
			}

			return null;
		}

	}

}
