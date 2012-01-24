package de.dfki.lt.tr.dialogue.interpret.atoms;

import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.Atom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.LinkedList;
import java.util.List;

public class AssertedReferenceAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "is_reference";

	private final String nominal;
	private final Term referentTerm;
	private final Term epstTerm;

	private AssertedReferenceAtom(String nominal, Term referentTerm, Term epstTerm) {
		this.nominal = nominal;
		this.referentTerm = referentTerm;
		this.epstTerm = epstTerm;
	}

	public String getNominal() {
		return nominal;
	}

	public Term getReferentTerm() {
		return referentTerm;
	}

	public Term getEpStTerm() {
		return epstTerm;
	}

	public static AssertedReferenceAtom newAssertedReferenceAtom(String nominal, dFormula referent, EpistemicStatus epst) {
		return new AssertedReferenceAtom(
				nominal,
				ConversionUtils.stateFormulaToTerm(referent),
				ConversionUtils.epistemicStatusToTerm(epst));
	}

	public static AssertedReferenceAtom fromModalisedAtom(ModalisedAtom matom) {
		if (matom.a.predSym.equals(PRED_SYMBOL) && matom.a.args.size() == 3) {
			Term nomTerm = matom.a.args.get(0);
			Term varTerm = matom.a.args.get(1);
			Term epstTerm = matom.a.args.get(2);
			if (nomTerm instanceof FunctionTerm) {
				String nominal = ((FunctionTerm) nomTerm).functor;
				return new AssertedReferenceAtom(nominal, varTerm, epstTerm);
			}
			else {
				return null;
			}
		}
		else {
			return null;
		}
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		List<Modality> m = new LinkedList<Modality>();
		m.add(Modality.Understanding);

		List<Term> args = new LinkedList<Term>();
		args.add(TermAtomFactory.term(nominal));
		args.add(referentTerm);
		args.add(epstTerm);

		return new ModalisedAtom(m, new Atom(PRED_SYMBOL, args));
	}

	public static class Matcher implements ModalisedAtomMatcher<AssertedReferenceAtom> {

		@Override
		public AssertedReferenceAtom match(ModalisedAtom matom) {
			return fromModalisedAtom(matom);
		}

	}

}
