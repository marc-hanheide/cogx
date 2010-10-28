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
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.dialogue.interpret.AbducerUtils;
import de.dfki.lt.tr.dialogue.interpret.BeliefFormulaFactory;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.dialogue.slice.ref.NominalEpistemicReference;
import de.dfki.lt.tr.dialogue.slice.ref.NominalEpistemicReferenceHypothesis;
import de.dfki.lt.tr.infer.weigabd.AbductionEngineConnection;
import de.dfki.lt.tr.infer.weigabd.ProofUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.Atom;
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
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 *
 * @author Miroslav Janicek
 */
public class AbductiveReferenceResolution {

	private int timeout;
	public boolean logging = true;
	private AbductionEngineConnection refresEngine = null;
	private String dumpfile;
	private String appendfile;

	private String abd_serverName = "";
	private String abd_endpoints = "";

	public static final String REFERENCE_RESOLUTION_ENGINE = "ReferenceResolution";

	public static final String BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME = "belief_exist";
	public static final String WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME = "world_exist";

	public AbductiveReferenceResolution(String servername, String endpoints, String dumpfile_, String appendfile_, int timeout_) {
		abd_serverName = servername;
		abd_endpoints = endpoints;
		init();
		dumpfile = dumpfile_;
		appendfile = appendfile_;
		timeout = timeout_;
	}

	private void init() {
		refresEngine = new AbductionEngineConnection();
		refresEngine.connectToServer(abd_serverName, abd_endpoints);
		refresEngine.bindToEngine(REFERENCE_RESOLUTION_ENGINE);
		refresEngine.getProxy().clearContext();
	}

	public List<NominalEpistemicReferenceHypothesis> resolvePresupposition(String nom, Map<String, String> fvPairs) {

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
					TermAtomFactory.var("EpSt"),
					propertiesToListTerm(fvPairs)
				}));

		List<ProofWithCost> proofs = AbducerUtils.allAbductiveProofs(refresEngine, ProofUtils.newUnsolvedProof(g), timeout);

		Map<AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>, Double> combined_hypos = new HashMap<AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>, Double>();
		Map<AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>, EpistemicStatus> epistemic_statuses = new HashMap<AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>, EpistemicStatus>();

		double total_mass = 0.0;
		// extract referential hypotheses from proofs
		for (ProofWithCost pwc : proofs) {
			ModalisedAtom rma = extractResolvesMAtom(pwc.proof);
			if (rma != null) {
				AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress> hypo = extractHypothesis(rma);
				EpistemicStatus epst = extractEpistemicStatus(rma);
				double mass = 0.0;
				if (combined_hypos.containsKey(hypo)) {
					mass = combined_hypos.get(hypo);
				}
				double deltaMass = Math.exp(-pwc.cost);
				mass += deltaMass;
				total_mass += deltaMass;

				combined_hypos.put(hypo, mass);
				epistemic_statuses.put(hypo, epst);
			}
		}

		List<NominalEpistemicReferenceHypothesis> result = new LinkedList<NominalEpistemicReferenceHypothesis>();

		// create the hypotheses
		for (AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress> hypo : combined_hypos.keySet()) {
			double prob = combined_hypos.get(hypo) / total_mass;

			NominalReference nref = new NominalReference(hypo.getKey(), BeliefFormulaFactory.newPointerFormula(hypo.getValue()));
			NominalEpistemicReference eref = new NominalEpistemicReference(nref, epistemic_statuses.get(hypo));
			NominalEpistemicReferenceHypothesis nhypo = new NominalEpistemicReferenceHypothesis(eref, prob);
			result.add(nhypo);
		}
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
		assert (matom.a.args.length == 3);
		assert (matom.a.args[0] instanceof FunctionTerm);
		String nom = ((FunctionTerm) matom.a.args[0]).functor;
		WorkingMemoryAddress wma = ConversionUtils.termToWorkingMemoryAddress(matom.a.args[1]);
		return new AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>(nom, wma);
	}

	public static EpistemicStatus extractEpistemicStatus(ModalisedAtom matom) {
		assert (matom.a.args.length == 3);
		assert (matom.a.args[2] instanceof FunctionTerm);
		return ConversionUtils.termToEpistemicStatus((FunctionTerm) matom.a.args[2]);
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
