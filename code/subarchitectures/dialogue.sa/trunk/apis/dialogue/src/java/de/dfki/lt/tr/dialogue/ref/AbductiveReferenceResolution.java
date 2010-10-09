// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots
// Miroslav Janicek (miroslav.janicek@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

package de.dfki.lt.tr.dialogue.ref;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.slice.ref.NominalRef;
import de.dfki.lt.tr.dialogue.slice.ref.RefHypo;
import de.dfki.lt.tr.infer.weigabd.AbductionEngineConnection;
import de.dfki.lt.tr.infer.weigabd.MercuryUtils;
import de.dfki.lt.tr.infer.weigabd.ProofUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.Atom;
import de.dfki.lt.tr.infer.weigabd.slice.DisjointDeclaration;
import de.dfki.lt.tr.infer.weigabd.slice.FileReadErrorException;
import de.dfki.lt.tr.infer.weigabd.slice.FunctionTerm;
import de.dfki.lt.tr.infer.weigabd.slice.MarkedQuery;
import de.dfki.lt.tr.infer.weigabd.slice.ModalisedAtom;
import de.dfki.lt.tr.infer.weigabd.slice.Modality;
import de.dfki.lt.tr.infer.weigabd.slice.ProofWithCost;
import de.dfki.lt.tr.infer.weigabd.slice.SyntaxErrorException;
import de.dfki.lt.tr.infer.weigabd.slice.Term;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleImmutableEntry;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 *
 * @author Miroslav Janicek
 */
public class AbductiveReferenceResolution {

	public boolean logging = true;
	private AbductionEngineConnection refresEngine = null;
	private AbductionEngineConnection intentionEngine = null;
	private String dumpfile;
	private String appendfile;
	public static final String REFERENCE_RESOLUTION_ENGINE = "ReferenceResolution";

	public AbductiveReferenceResolution(String dumpfile_, String appendfile_, AbductionEngineConnection intentionEngine_) {
		init();
		dumpfile = dumpfile_;
		appendfile = appendfile_;
		intentionEngine = intentionEngine_;
	}

	private void init() {
		refresEngine = new AbductionEngineConnection();
		refresEngine.connectToServer("AbducerServer", "default -p 10000");
		refresEngine.bindToEngine(REFERENCE_RESOLUTION_ENGINE);
		refresEngine.getProxy().clearContext();
	}

	public NominalRef resolvePresupposition(String nom, Map<String, String> fvPairs) {
		NominalRef result = new NominalRef();
		result.nominal = nom;

		refresEngine.getProxy().clearRules();
		refresEngine.getProxy().clearFacts();
		refresEngine.getProxy().clearDisjointDeclarations();
		refresEngine.getProxy().clearAssumabilityFunction("belief_exist");
		refresEngine.getProxy().clearAssumabilityFunction("world_exist");
		loadFile(dumpfile);
		loadFile(appendfile);

		ModalisedAtom g = TermAtomFactory.modalisedAtom(
				new Modality[] { },
				TermAtomFactory.atom("resolves_main", new Term[] {
					TermAtomFactory.term(nom),
					TermAtomFactory.var("BId"),
//					TermAtomFactory.var("EpSt"),
					propertiesToListTerm(fvPairs)
				}));

		List<RefHypo> hypos = new LinkedList<RefHypo>();
		ProofWithCost[] proof = allAbductiveProofs(ProofUtils.newUnsolvedProof(g));

		Map<String, Set<ModalisedAtom>> disj = new HashMap<String, Set<ModalisedAtom>>();

		if (proof != null) {

			Map<AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>, Double> combined_hypos = new HashMap<AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>, Double>();
			double total_mass = 0.0;

			for (int i = 0; i < proof.length; i++) {
				ModalisedAtom rma = extractResolvesMAtom(proof[i].proof);
				if (rma != null) {
					AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress> hypo = extractHypothesis(rma);
					double mass = 0.0;
					if (combined_hypos.containsKey(hypo)) {
						mass = combined_hypos.get(hypo);
					}
					double deltaMass = Math.exp(-proof[i].cost);
					mass += deltaMass;
					combined_hypos.put(hypo, mass);
					total_mass += deltaMass;
				}
			}

			for (AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress> hypo : combined_hypos.keySet()) {
				double prob = combined_hypos.get(hypo) / total_mass;
				ModalisedAtom rma = TermAtomFactory.modalisedAtom(new Modality[] {Modality.Understanding},
						TermAtomFactory.atom("resolves_to_belief", new Term[] {
							TermAtomFactory.term(hypo.getKey()),
							ConversionUtils.workingMemoryAddressToTerm(hypo.getValue())
						} ));

				if (intentionEngine != null) {
					log("adding reference hypothesis: " + MercuryUtils.modalisedAtomToString(rma) + " @ p=" + prob);
					intentionEngine.getProxy().addAssumable("reference_resolution", rma, (float) -Math.log(prob));

					String n = ((FunctionTerm)rma.a.args[0]).functor;
					Set<ModalisedAtom> dj = disj.get(n);
					if (dj != null) {
						dj.add(rma);
					}
					else {
						dj = new HashSet<ModalisedAtom>();
						dj.add(rma);
						disj.put(n, dj);
					}
				}

				RefHypo hypox = new RefHypo();
//				hypox.prob = Math.exp(-proof[i].cost);
//				hypox.beliefId = extractBeliefId(proof[i].proof);
//				if (hypox.beliefId != null) {
//					hypos.add(hypox);
//				}
			}
		}

		// add disjoint declarations
		if (intentionEngine != null) {
			for (String n : disj.keySet()) {
				Set<ModalisedAtom> dj = disj.get(n);
				DisjointDeclaration dd = new DisjointDeclaration();
				dd.atoms = dj.toArray(new ModalisedAtom[0]);
				log("adding a disjoint declaration for " + n + " (" + dj.size() + " entries)");
				intentionEngine.getProxy().addDisjointDeclaration(dd);
			}
		}

		result.hypos = hypos.toArray(new RefHypo[0]);
		return result;
	}

	public static ModalisedAtom extractResolvesMAtom(MarkedQuery[] qs) {
		for (MarkedQuery q : qs) {
			ModalisedAtom ma = q.atom;
			if (ma.m.length > 0 && ma.m[0] == Modality.Understanding && ma.a.predSym.equals("resolves_to_belief")) {
//				log(MercuryUtils.modalisedAtomToString(ma));
//				ModalisedAtom copy = TermAtomFactory.modalisedAtom(new Modality[] {Modality.Understanding}, TermAtomFactory.atom("resolves_to_belief", new Term[] {ma.a.args[0], ma.a.args[1]}));
//				return copy;
				return ma;
			}
		}
		return null;
	}

	public static AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress> extractHypothesis(ModalisedAtom matom) {
		assert (matom.a.args.length == 2);
		assert (matom.a.args[0] instanceof FunctionTerm);
		String nom = ((FunctionTerm) matom.a.args[0]).functor;
		WorkingMemoryAddress wma = ConversionUtils.termToWorkingMemoryAddress(matom.a.args[1]);
		return new AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>(nom, wma);
	}

	private static String extractBeliefId(MarkedQuery[] qs) {
		for (MarkedQuery q : qs) {
			Atom a = q.atom.a;
			if (a.predSym.equals("resolves_to_belief") && a.args[1] instanceof FunctionTerm) {
				FunctionTerm ft = (FunctionTerm)a.args[1];
				return ft.functor;
			}
		}
		return null;
	}

	private ProofWithCost[] allAbductiveProofs(MarkedQuery[] goal) {
		String listGoalsStr = "";
		for (int i = 0; i < goal.length; i++) {
			listGoalsStr += MercuryUtils.modalisedAtomToString(goal[i].atom);
			if (i < goal.length - 1) listGoalsStr += ", ";
		}
		log("proving: [" + listGoalsStr + "]");

		refresEngine.getProxy().startProving(goal);
		ProofWithCost[] result = refresEngine.getProxy().getProofs(500);
		if (result.length > 0) {
			log("found " + result.length + " alternatives");
			return result;
		}
		else {
			return null;
		}
	}

	private Term propertiesToListTerm(Map<String, String> fvPairs) {
		FunctionTerm start = TermAtomFactory.term("[]");
		for (String featName : fvPairs.keySet()) {
			FunctionTerm newHead = new FunctionTerm();
			newHead.functor = "[|]";
			newHead.args = new FunctionTerm[2];
			newHead.args[0] = TermAtomFactory.term(
					featName.toLowerCase(),
					new Term[] { TermAtomFactory.term(fvPairs.get(featName).toLowerCase())}
			);
			newHead.args[1] = start;
			start = newHead;
		}
		return start;
	}

	/**
	 * Load a file with rules and facts for the abducer.
	 *
	 * @param file file name
	 */
	public void loadFile(String file) {
		try {
			refresEngine.getProxy().loadFile(file);
		}
		catch (FileReadErrorException ex) {
			log("file read error: " + ex.filename);
		}
		catch (SyntaxErrorException ex) {
			log("syntax error: " + ex.error + " in " + ex.filename + " on line " + ex.line);
		}
	}

	private void log(String str) {
		if (logging)
			System.out.println("\033[32m[RefRes]\t" + str + "\033[0m");
	}
}
