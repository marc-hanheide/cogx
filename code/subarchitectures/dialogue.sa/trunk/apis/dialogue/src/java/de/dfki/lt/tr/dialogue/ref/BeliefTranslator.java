package de.dfki.lt.tr.dialogue.ref;

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
import de.dfki.lt.tr.infer.weigabd.MercuryUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.Antecedent;
import de.dfki.lt.tr.infer.weigabd.slice.AssumableAntecedent;
import de.dfki.lt.tr.infer.weigabd.slice.ModalisedAtom;
import de.dfki.lt.tr.infer.weigabd.slice.Modality;
import de.dfki.lt.tr.infer.weigabd.slice.NamedAssumabilityFunction;
import de.dfki.lt.tr.infer.weigabd.slice.NullAssumabilityFunction;
import de.dfki.lt.tr.infer.weigabd.slice.Rule;
import de.dfki.lt.tr.infer.weigabd.slice.Term;
import java.util.AbstractMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class BeliefTranslator {

	public static final Modality[] mod = new Modality[] { Modality.Belief };
	public static final String BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME = "belief_exist";
	public static final String WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME = "world_exist";

	public List<ModalisedAtom> facts_epst = new LinkedList<ModalisedAtom>();
	public List<ModalisedAtom> facts_ancestor = new LinkedList<ModalisedAtom>();
	public List<ModalisedAtom> facts_offspring = new LinkedList<ModalisedAtom>();
	public List<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>> assf_belief_exist = new LinkedList<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>>();
	public List<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>> assf_world_exist = new LinkedList<AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double>>();
	public List<List<ModalisedAtom>> disjoints = new LinkedList<List<ModalisedAtom>>();
	public List<Rule> rules = new LinkedList<Rule>();

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

		assf_belief_exist.add(
				new AbstractMap.SimpleImmutableEntry<ModalisedAtom, Double>(
					belexMAtom,
					new Double(exist_prob)));

		// ancestors and offsprings
		if (bel.hist instanceof CASTBeliefHistory) {
			CASTBeliefHistory chist = (CASTBeliefHistory) bel.hist;
			for (WorkingMemoryPointer wmp : chist.ancestors) {
				facts_ancestor.add(
						TermAtomFactory.modalisedAtom(mod,
							TermAtomFactory.atom("ancestor",
								new Term[] {
									belIdTerm,
									ConversionUtils.workingMemoryAddressToTerm(wmp.address),
									TermAtomFactory.term(wmp.type)
								} )));
			}
			for (WorkingMemoryPointer wmp : chist.offspring) {
				facts_offspring.add(
						TermAtomFactory.modalisedAtom(mod,
							TermAtomFactory.atom("offspring",
								new Term[] {
									belIdTerm,
									ConversionUtils.workingMemoryAddressToTerm(wmp.address),
									TermAtomFactory.term(wmp.type)
								} )));
			}
		}

		// examine the contents
		if (bel.content instanceof CondIndependentDistribs) {
			CondIndependentDistribs cdd = (CondIndependentDistribs) bel.content;

			for (ProbDistribution pd : cdd.distribs.values()) {
				if (pd instanceof BasicProbDistribution) {
					BasicProbDistribution bpd = (BasicProbDistribution)pd;

					List<ModalisedAtom> ddpart = new LinkedList<ModalisedAtom>();

					if (bpd.values instanceof FormulaValues) {
						FormulaValues fv = (FormulaValues) bpd.values;
						int idx = 1;
						for (FormulaProbPair fp : fv.values) {

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
					disjoints.add(ddpart);
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
			valTerm = TermAtomFactory.term(eF.prop);
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

			r.ante = new Antecedent[] {
				new AssumableAntecedent(worldexMAtom, new NamedAssumabilityFunction(WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME)),
				new AssumableAntecedent(belexMAtom, new NamedAssumabilityFunction(BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME))
			};

			rules.add(r);
		}
	}

	public String toRulefileContents() {
		StringBuilder sb = new StringBuilder();
		List<String> args = new LinkedList<String>();

		sb.append("% epistemic statuses\n");
		for (ModalisedAtom ma : facts_epst) {
			args.add(MercuryUtils.modalisedAtomToString(ma) + ".");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");

		sb.append("% belief existence assumability function\n");
		args.clear();
		for (AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double> ass : assf_belief_exist) {
			args.add("(" + MercuryUtils.modalisedAtomToString(ass.getKey()) + ") = p(" + ass.getValue() + ")");
		}
		sb.append(BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME + " = [\n\t").append(join(",\n\t", args)).append("\n].\n");
		sb.append("\n");

		sb.append("% belief ancestors\n");
		args.clear();
		for (ModalisedAtom ma : facts_ancestor) {
			args.add(MercuryUtils.modalisedAtomToString(ma) + ".");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");

		sb.append("% belief offsprings\n");
		args.clear();
		for (ModalisedAtom ma : facts_offspring) {
			args.add(MercuryUtils.modalisedAtomToString(ma) + ".");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");

		sb.append("% world existence assumability function\n");
		args.clear();
		for (AbstractMap.SimpleImmutableEntry<ModalisedAtom,Double> ass : assf_world_exist) {
			args.add("(" + MercuryUtils.modalisedAtomToString(ass.getKey()) + ") = p(" + ass.getValue() + ")");
		}
		sb.append(WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME + " = [\n\t").append(join(",\n\t", args)).append("\n].\n");
		sb.append("\n");

		sb.append("% disjoint declarations\n");
		args.clear();
		for (List<ModalisedAtom> dd : disjoints) {
			List<String> ss = new LinkedList<String>();
			for (ModalisedAtom d : dd) {
				ss.add(MercuryUtils.modalisedAtomToString(d));
			}
			args.add("disjoint([" + join(", ", ss) + "]).");
		}
		sb.append(join("\n", args)).append("\n");
		sb.append("\n");

		sb.append("% rules\n");
		args.clear();
		for (Rule r : rules) {
			List<String> ss = new LinkedList<String>();
			for (int i = 0; i < r.ante.length; i++) {
				if (r.ante[i] instanceof AssumableAntecedent) {
					AssumableAntecedent a = (AssumableAntecedent) r.ante[i];

					if (a.f instanceof NullAssumabilityFunction) {
						ss.add(MercuryUtils.modalisedAtomToString(a.matom));
					}
					if (a.f instanceof NamedAssumabilityFunction) {
						NamedAssumabilityFunction naf = (NamedAssumabilityFunction) a.f;
						ss.add(MercuryUtils.modalisedAtomToString(a.matom) + " / " + naf.name);
					}
				}
			}
			if (!ss.isEmpty()) {
				args.add(MercuryUtils.modalisedAtomToString(r.head) + " <- " + join(", ", ss) + ".");
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
