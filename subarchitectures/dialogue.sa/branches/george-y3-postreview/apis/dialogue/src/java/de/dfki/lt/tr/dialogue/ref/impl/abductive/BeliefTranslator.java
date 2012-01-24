package de.dfki.lt.tr.dialogue.ref.impl.abductive;

import de.dfki.lt.tr.dialogue.ref.impl.abductive.AbductiveReferenceResolution;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.infer.abducer.lang.Antecedent;
import de.dfki.lt.tr.infer.abducer.lang.AssumableAntecedent;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.NamedAssumabilityFunction;
import de.dfki.lt.tr.infer.abducer.lang.NullAssumabilityFunction;
import de.dfki.lt.tr.infer.abducer.lang.Rule;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.AbstractMap;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class BeliefTranslator {

	public static final ArrayList<Modality> mod = new ArrayList(Arrays.asList(new Modality[] { Modality.Belief }));

	public static double THRESHOLD_MIN = 0.01f;
	public static double THRESHOLD_MAX = 1.0f;

	public List<ModalisedAtom> facts_epst = new ArrayList<ModalisedAtom>();
	public List<ModalisedAtom> facts_ancestor = new ArrayList<ModalisedAtom>();
	public List<ModalisedAtom> facts_offspring = new ArrayList<ModalisedAtom>();
	public List<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>> assf_belief_exist = new ArrayList<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>>();
	public List<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>> assf_world_exist = new ArrayList<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>>();
	public List<List<ModalisedAtom>> disjoints = new ArrayList<List<ModalisedAtom>>();
	public List<Rule> rules = new ArrayList<Rule>();

	public void addBelief(WorkingMemoryAddress addr, dBelief bel) {

		Term belIdTerm = ConversionUtils.workingMemoryAddressToTerm(addr);

		// epistemic status
		facts_epst.add(
				TermAtomFactory.modalisedAtom(mod,
					TermAtomFactory.atom("epistemic_status",
						new Term[] {
							belIdTerm,
							ConversionUtils.epistemicStatusToTerm(bel.estatus)
						} )));

		// belief_exist
		double exist_prob = 1.0;
		if (bel.frame instanceof SpatioTemporalFrame) {
			SpatioTemporalFrame stf = (SpatioTemporalFrame) bel.frame;
			exist_prob = stf.existProb;
		}
		ModalisedAtom belexMAtom = TermAtomFactory.modalisedAtom(mod,
				TermAtomFactory.atom("b",
					new Term[] {
						belIdTerm
					} ));

		if (exist_prob >= THRESHOLD_MIN && exist_prob <= THRESHOLD_MAX) {

			assf_belief_exist.add(
					new AbstractMap.SimpleImmutableEntry<ModalisedAtom, Double>(
						belexMAtom,
						new Double(exist_prob)));

			// ancestors
			if (bel.hist instanceof CASTBeliefHistory && ((CASTBeliefHistory)bel.hist).ancestors.size() > 0) {
				CASTBeliefHistory chist = (CASTBeliefHistory) bel.hist;
				WorkingMemoryPointer wmp = chist.ancestors.get(0);
				facts_ancestor.add(
						TermAtomFactory.modalisedAtom(mod,
							TermAtomFactory.atom("ancestor",
								new Term[] {
									belIdTerm,
									ConversionUtils.workingMemoryAddressToTerm(wmp.address),
									TermAtomFactory.term("type" + wmp.type)
								} )));
			}
			else {
				facts_ancestor.add(
						TermAtomFactory.modalisedAtom(mod,
							TermAtomFactory.atom("ancestor",
								new Term[] {
									belIdTerm,
									belIdTerm,
									TermAtomFactory.term("type::unknown")
								} )));
			}

			// examine the contents
			if (bel.content instanceof CondIndependentDistribs) {
				CondIndependentDistribs cdd = (CondIndependentDistribs) bel.content;

				for (ProbDistribution pd : cdd.distribs.values()) {
					if (pd instanceof BasicProbDistribution) {
						BasicProbDistribution bpd = (BasicProbDistribution)pd;

						List<ModalisedAtom> ddpart = new ArrayList<ModalisedAtom>();

						if (bpd.values instanceof FormulaValues) {
							FormulaValues fv = (FormulaValues) bpd.values;
							int idx = 1;
							for (FormulaProbPair fp : fv.values) {

								if (fp.prob >= THRESHOLD_MIN && fp.prob <= THRESHOLD_MAX) {

									// world_exist
									String worldId = bpd.key + idx;
									Term worldIdTerm = TermAtomFactory.term(worldId);

									ModalisedAtom worldexMAtom = TermAtomFactory.modalisedAtom(mod,
												TermAtomFactory.atom("w",
													new Term[] {
														belIdTerm,
														worldIdTerm
													} ));

									assf_world_exist.add(
										new AbstractMap.SimpleImmutableEntry<ModalisedAtom, Double>(
											worldexMAtom,
											new Double(fp.prob)));

									// disjoints
									ddpart.add(worldexMAtom);

									// rules
									expandFormulaToRules(bpd.key, fp.val, rules, belIdTerm, belexMAtom, worldexMAtom);

									idx++;
								}
							}
						}
						disjoints.add(ddpart);
					}
				}
			}
		}
	}

	private void expandFormulaToRules(String modality, dFormula val, List<Rule> args, Term belIdTerm, ModalisedAtom belexMAtom, ModalisedAtom worldexMAtom) {
		Term valTerm = null;

		if (val instanceof ComplexFormula && ((ComplexFormula)val).op == BinaryOp.conj) {
			ComplexFormula cF = (ComplexFormula)val;
			for (dFormula subF : cF.forms) {
				expandFormulaToRules(modality, subF, args, belIdTerm, belexMAtom, worldexMAtom);
			}
		}

		if (val instanceof ElementaryFormula) {
			ElementaryFormula eF = (ElementaryFormula) val;
			String prop = eF.prop.replaceAll("[^a-zA-Z0-9_\\-:]", "");
			valTerm = TermAtomFactory.term(prop);
		}

		if (val instanceof FloatFormula) {
			FloatFormula fF = (FloatFormula) val;
			String discrete = "zero";
			if (fF.val != 0.0) {
				discrete = "nonzero";
			}
			valTerm = TermAtomFactory.term(discrete);
		}

		if (val instanceof PointerFormula) {
			PointerFormula pF = (PointerFormula) val;
			valTerm = ConversionUtils.workingMemoryAddressToTerm(pF.pointer);
		}

		if (valTerm != null) {
			Rule r = new Rule();
			r.head = TermAtomFactory.modalisedAtom(mod,
					TermAtomFactory.atom(modality,
						new Term[] {
							belIdTerm,
							valTerm
						}));

			r.ante = new ArrayList<Antecedent>();
			r.ante.add(new AssumableAntecedent(worldexMAtom, new NamedAssumabilityFunction(AbductiveReferenceResolution.WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME)));
			r.ante.add(new AssumableAntecedent(belexMAtom, new NamedAssumabilityFunction(AbductiveReferenceResolution.BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME)));

			rules.add(r);
		}
	}

	public String toRulefileContents() {
		StringBuilder sb = new StringBuilder();
		List<String> args = new ArrayList<String>();

		sb.append("% epistemic statuses\n");
		for (ModalisedAtom ma : facts_epst) {
			args.add(PrettyPrint.modalisedAtomToString(ma) + ".");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");

		sb.append("% belief existence assumability function\n");
		args.clear();
		for (AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double> ass : assf_belief_exist) {
			args.add("(" + PrettyPrint.modalisedAtomToString(ass.getKey()) + ") = p(" + ass.getValue() + ")");
		}
		sb.append(AbductiveReferenceResolution.BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME + " = [\n\t").append(join(",\n\t", args)).append("\n].\n");
		sb.append("\n");

		sb.append("% belief ancestors\n");
		args.clear();
		for (ModalisedAtom ma : facts_ancestor) {
			args.add(PrettyPrint.modalisedAtomToString(ma) + ".");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");

/*
		sb.append("% belief offsprings\n");
		args.clear();
		for (ModalisedAtom ma : facts_offspring) {
			args.add(PrettyPrint.modalisedAtomToString(ma) + ".");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");
 */

		sb.append("% world existence assumability function\n");
		args.clear();
		for (AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double> ass : assf_world_exist) {
			args.add("(" + PrettyPrint.modalisedAtomToString(ass.getKey()) + ") = p(" + ass.getValue() + ")");
		}
		sb.append(AbductiveReferenceResolution.WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME + " = [\n\t").append(join(",\n\t", args)).append("\n].\n");
		sb.append("\n");

		sb.append("% disjoint declarations\n");
		args.clear();
		for (List<ModalisedAtom> dd : disjoints) {
			List<String> ss = new ArrayList<String>();
			for (ModalisedAtom d : dd) {
				ss.add(PrettyPrint.modalisedAtomToString(d));
			}
			args.add("disjoint([" + join(", ", ss) + "]).");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");

		sb.append("% rules\n");
		args.clear();
		for (Rule r : rules) {
			List<String> ss = new ArrayList<String>();
			for (int i = 0; i < r.ante.size(); i++) {
				if (r.ante.get(i) instanceof AssumableAntecedent) {
					AssumableAntecedent a = (AssumableAntecedent) r.ante.get(i);

					if (a.f instanceof NullAssumabilityFunction) {
						ss.add(PrettyPrint.modalisedAtomToString(a.matom));
					}
					if (a.f instanceof NamedAssumabilityFunction) {
						NamedAssumabilityFunction naf = (NamedAssumabilityFunction) a.f;
						ss.add(PrettyPrint.modalisedAtomToString(a.matom) + " / " + naf.name);
					}
				}
			}
			if (!ss.isEmpty()) {
				args.add(PrettyPrint.modalisedAtomToString(r.head) + " <- " + join(", ", ss) + ".");
			}
		}
		sb.append(join("\n", args)).append("\n");

		return sb.toString();
	}

	// Sun should burn in hell for not having such a function in the standard library!
	public static String join(String separator, List<String> args) {
	    StringBuilder sb = new StringBuilder();
	    if (args != null) {
			Iterator<String> iter = args.iterator();
			while (iter.hasNext()) {
				sb.append(iter.next());
				if (iter.hasNext()) {
					sb.append(separator);
				}
			}
		}
		return sb.toString();
    }

}
