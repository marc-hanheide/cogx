package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;

public class BeliefContentAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "belief_content";

	private final WorkingMemoryAddress beliefAddr;
	private final String key;
	private final dFormula value;

	public BeliefContentAtom(WorkingMemoryAddress wma, String key, dFormula value) {
		this.beliefAddr = wma;
		this.key = key;
		this.value = value;
	}

	public WorkingMemoryAddress getBeliefAddress() {
		return beliefAddr;
	}

	public String getKey() {
		return key;
	}

	public dFormula getValue() {
		return value;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Intention
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					beliefAddr == null ? TermAtomFactory.var("BeliefID") : ConversionUtils.workingMemoryAddressToTerm(beliefAddr),
					key == null ? TermAtomFactory.var("Key") : TermAtomFactory.term(key),
					value == null ? TermAtomFactory.var("Formula") : ConversionUtils.stateFormulaToTerm(value)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<BeliefContentAtom> {

		@Override
		public BeliefContentAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 3) {

				Term beliefAddrTerm = matom.a.args.get(0);
				Term keyTerm = matom.a.args.get(1);
				Term valueTerm = matom.a.args.get(2);

				WorkingMemoryAddress wma = null;
				String key = null;
				dFormula value = null;

				if (beliefAddrTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) beliefAddrTerm;
					wma = ConversionUtils.termToWorkingMemoryAddress(beliefAddrTerm);
					if (wma == null) {
						// unparseable!
						return null;
					}
				}

				if (keyTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) keyTerm;
					if (isConstTerm(ft)) {
						key = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				if (valueTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) valueTerm;
					value = ConversionUtils.uniTermToFormula(ft);
					if (value == null) {
						// unparseable!
						return null;
					}
				}

				return new BeliefContentAtom(wma, key, value);

			}

			return null;
		}

	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}