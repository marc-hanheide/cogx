package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.Atom;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import java.util.LinkedList;
import java.util.List;

public class AssertedReferenceAtom {

	private final Term nominalTerm;
	private final Term referentTerm;
	private final Term epstTerm;

	private AssertedReferenceAtom(Term nominalTerm, Term referentTerm, Term epstTerm) {
		this.nominalTerm = nominalTerm;
		this.referentTerm = referentTerm;
		this.epstTerm = epstTerm;
	}

	public Term getNominalTerm() {
		return nominalTerm;
	}

	public Term getReferentTerm() {
		return referentTerm;
	}

	public Term getEpStTerm() {
		return epstTerm;
	}

	public static AssertedReferenceAtom fromModalisedAtom(ModalisedAtom matom) {
		if (matom.a.predSym.equals(ConversionUtils.predsym_IS_REFERENCE) && matom.a.args.size() == 3) {
			Term nomTerm = matom.a.args.get(0);
			Term varTerm = matom.a.args.get(1);
			Term epstTerm = matom.a.args.get(2);
			return new AssertedReferenceAtom(nomTerm, varTerm, epstTerm);
		}
		else {
			return null;
		}
	}

	public ModalisedAtom toModalisedAtom() {
		List<Modality> m = new LinkedList<Modality>();
		m.add(Modality.Understanding);

		List<Term> args = new LinkedList<Term>();
		args.add(nominalTerm);
		args.add(referentTerm);
		args.add(epstTerm);

		return new ModalisedAtom(m, new Atom(ConversionUtils.predsym_IS_REFERENCE, args));
	}

}
