package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;

public class GenerateReferringExpressionAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "reference_generation";

	private final WorkingMemoryAddress beliefAddr;
	private final Boolean shortNP;
	private final Boolean spatialRelation;
	private final String refEx;

	public GenerateReferringExpressionAtom(WorkingMemoryAddress beliefAddr, Boolean shortNP, Boolean spatialRelation, String refEx) {
		this.beliefAddr = beliefAddr;
		this.shortNP = shortNP;
		this.spatialRelation = spatialRelation;
		this.refEx = refEx;
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
					refEx == null ? TermAtomFactory.var("RefEx") : TermAtomFactory.term(refEx)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<GenerateReferringExpressionAtom> {

		@Override
		public GenerateReferringExpressionAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 4) {

				Term beliefAddrTerm = matom.a.args.get(0);
				Term hasShortNPTerm = matom.a.args.get(1);
				Term hasSpatialRelationTerm = matom.a.args.get(2);
				Term refExTerm = matom.a.args.get(3);

				WorkingMemoryAddress beliefAddr = null;
				Boolean hasShortNP = null;
				Boolean hasSpatialRelation = null;
				String location = null;

				if (beliefAddrTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) beliefAddrTerm;
					beliefAddr = ConversionUtils.termToWorkingMemoryAddress(beliefAddrTerm);
					if (beliefAddr == null) {
						// unparseable!
						return null;
					}
				}

				try {
					hasShortNP = termToBoolean(hasShortNPTerm);
					hasSpatialRelation = termToBoolean(hasSpatialRelationTerm);
				}
				catch (TermParsingException ex) {
					return null;
				}

				if (refExTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) refExTerm;
					if (isConstTerm(ft)) {
						location = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				return new GenerateReferringExpressionAtom(beliefAddr, hasShortNP, hasSpatialRelation, location);

			}

			return null;
		}

	}

	public static Boolean termToBoolean(Term t) throws TermParsingException {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (isConstTerm(ft)) {
				String functor = ft.functor;
				if (functor.equals("yes")) {
					return true;
				}
				else if (functor.equals("no")) {
					return false;
				}
				else {
					throw new TermParsingException("Boolean", t);
				}
			}
			else {
				throw new TermParsingException("Boolean", t);
			}
		}
		else {
			// it's a variable
			return null;
		}
	}

	public static class TermParsingException extends Exception {
		public TermParsingException(String expected, Term t) {
			super("cannot convert " + PrettyPrint.termToString(t) + " to " + expected);
		}
	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}
