package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;

public class GenerateReferringExpressionAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "reference_generation";

	private final WorkingMemoryAddress beliefAddr;
	private final String category;
	private final String relation;
	private final String location;

	public GenerateReferringExpressionAtom(WorkingMemoryAddress beliefAddr, String category, String relation, String location) {
		this.beliefAddr = beliefAddr;
		this.category = category;
		this.relation = relation;
		this.location = location;
	}

	public WorkingMemoryAddress getBeliefAddress() {
		return beliefAddr;
	}

	public String getCategory() {
		return category;
	}

	public String getRelation() {
		return relation;
	}

	public String getLocation() {
		return location;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Generation
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					beliefAddr == null ? TermAtomFactory.var("BeliefAddr") : ConversionUtils.workingMemoryAddressToTerm(beliefAddr),
					category == null ? TermAtomFactory.var("Category") : TermAtomFactory.term(category),
					relation == null ? TermAtomFactory.var("Relation") : TermAtomFactory.term(relation),
					location == null ? TermAtomFactory.var("Location") : TermAtomFactory.term(location)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<GenerateReferringExpressionAtom> {

		@Override
		public GenerateReferringExpressionAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 4) {

				Term beliefAddrTerm = matom.a.args.get(0);
				Term categoryTerm = matom.a.args.get(1);
				Term relationTerm = matom.a.args.get(2);
				Term locationTerm = matom.a.args.get(3);

				WorkingMemoryAddress beliefAddr = null;
				String category = null;
				String relation = null;
				String location = null;

				if (beliefAddrTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) beliefAddrTerm;
					beliefAddr = ConversionUtils.termToWorkingMemoryAddress(beliefAddrTerm);
					if (beliefAddr == null) {
						// unparseable!
						return null;
					}
				}

				if (categoryTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) categoryTerm;
					if (isConstTerm(ft)) {
						category = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				if (relationTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) relationTerm;
					if (isConstTerm(ft)) {
						relation = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				if (locationTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) locationTerm;
					if (isConstTerm(ft)) {
						location = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				return new GenerateReferringExpressionAtom(beliefAddr, category, relation, location);

			}

			return null;
		}

	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}
