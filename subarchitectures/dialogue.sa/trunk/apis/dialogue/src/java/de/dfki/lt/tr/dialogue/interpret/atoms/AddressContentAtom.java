package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.dialogue.interpret.MatcherUtils;
import de.dfki.lt.tr.dialogue.interpret.TermParsingException;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.List;

public class AddressContentAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "address_content";

	private final WorkingMemoryAddress wma;
	private final String key;
	private final WorkingMemoryAddress value;

	public AddressContentAtom(WorkingMemoryAddress wma, String key, WorkingMemoryAddress value) {
		this.wma = wma;
		this.key = key;
		this.value = value;
	}

	public WorkingMemoryAddress getIntentionWMA() {
		return wma;
	}

	public String getKey() {
		return key;
	}

	public WorkingMemoryAddress getValue() {
		return value;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Intention
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					wma == null ? TermAtomFactory.var("IntID") : ConversionUtils.workingMemoryAddressToTerm(wma),
					key == null ? TermAtomFactory.var("Key") : TermAtomFactory.term(key),
					value == null ? TermAtomFactory.var("Value") : ConversionUtils.workingMemoryAddressToTerm(value)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<AddressContentAtom> {

		@Override
		public AddressContentAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 3) {

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress wma = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(0));
					String key = MatcherUtils.parseTermToString(args.get(1));
					WorkingMemoryAddress value = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(2));

					return new AddressContentAtom(wma, key, value);
				}
				catch (TermParsingException ex) {
					return null;
				}
			}

			return null;
		}

	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}