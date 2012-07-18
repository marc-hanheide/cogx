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

package de.dfki.lt.tr.dialogue.ref.impl.abductive;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.dialogue.interpret.AbducerUtils;
import de.dfki.lt.tr.dialogue.util.BeliefFormulaFactory;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.MarkedQuery;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleImmutableEntry;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.apache.log4j.Logger;

/**
 *
 * @author Miroslav Janicek
 */
@Deprecated
public class AbductiveReferenceResolution
implements ReferenceResolver {

	private final Logger logger;
	private final int timeout;
	public boolean logging = true;
	private final AbductionEngineConnection engineConnection;
	private final String dumpfile;
	private final String appendfile;

	private final String abd_serverName;
	private final String abd_endpoints;

	public static final String REFERENCE_RESOLUTION_ENGINE = "ReferenceResolution";

	public static final String BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME = "belief_exist";
	public static final String WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME = "world_exist";

	public AbductiveReferenceResolution(Logger logger, String servername, String endpoints, String dumpfile_, String appendfile_, int timeout_) {
		this.logger = logger;
		abd_serverName = servername;
		abd_endpoints = endpoints;
		dumpfile = dumpfile_;
		appendfile = appendfile_;
		timeout = timeout_;

		engineConnection = new AbductionEngineConnection();
		engineConnection.connectToServer(abd_serverName, abd_endpoints);
		engineConnection.bindToEngine(REFERENCE_RESOLUTION_ENGINE);
		engineConnection.getEngineProxy().clearContext();
	}

/*
	public List<NominalEpistemicReferenceHypothesis> resolvePresupposition(String nom, Map<String, String> fvPairs) {

		engineConnection.getEngineProxy().clearRules();
		engineConnection.getEngineProxy().clearFacts();
		engineConnection.getEngineProxy().clearDisjointDeclarations();
		engineConnection.getEngineProxy().clearAssumabilityFunction(BELIEF_EXIST_ASSUMABILITY_FUNCTION_NAME);
		engineConnection.getEngineProxy().clearAssumabilityFunction(WORLD_EXIST_ASSUMABILITY_FUNCTION_NAME);
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

		List<ProofWithCost> proofs = AbducerUtils.allAbductiveProofs(engineConnection, ProofUtils.newUnsolvedProof(g), timeout);

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
*/

	public static ModalisedAtom extractResolvesMAtom(List<MarkedQuery> qs) {
		for (MarkedQuery q : qs) {
			ModalisedAtom ma = q.atom;
			if (!ma.m.isEmpty() && ma.m.get(0) == Modality.Understanding && ma.a.predSym.equals("resolves_to_belief")) {
//				log(MercuryUtils.modalisedAtomToString(ma));
//				ModalisedAtom copy = TermAtomFactory.modalisedAtom(new Modality[] {Modality.Understanding}, TermAtomFactory.atom("resolves_to_belief", new Term[] {ma.a.args[0], ma.a.args[1]}));
//				return copy;
				return ma;
			}
		}
		return null;
	}

	public static AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress> extractHypothesis(ModalisedAtom matom) {
		assert (matom.a.args.size() == 3);
		assert (matom.a.args.get(0) instanceof FunctionTerm);
		String nom = ((FunctionTerm) matom.a.args.get(0)).functor;
		WorkingMemoryAddress wma = ConversionUtils.termToWorkingMemoryAddress(matom.a.args.get(1));
		return new AbstractMap.SimpleImmutableEntry<String, WorkingMemoryAddress>(nom, wma);
	}

	public static EpistemicStatus extractEpistemicStatus(ModalisedAtom matom) {
		assert (matom.a.args.size() == 3);
		assert (matom.a.args.get(2) instanceof FunctionTerm);
		return ConversionUtils.termToEpistemicStatus((FunctionTerm) matom.a.args.get(2));
	}

	private Term propertiesToListTerm(Map<String, String> fvPairs) {
		FunctionTerm start = TermAtomFactory.term("[]");
		for (String featName : fvPairs.keySet()) {
			FunctionTerm newHead = new FunctionTerm();
			newHead.functor = "[|]";
			newHead.args = new ArrayList<Term>();

			newHead.args.add(TermAtomFactory.term(
					featName.toLowerCase(),
					new Term[] { TermAtomFactory.term(fvPairs.get(featName).toLowerCase())}
			));
			newHead.args.add(start);
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
			engineConnection.getEngineProxy().loadFile(file);
		}
		catch (FileReadErrorException ex) {
			logger.error("file read error: " + ex.filename);
		}
		catch (SyntaxErrorException ex) {
			logger.error("syntax error: " + ex.error + " in " + ex.filename + " on line " + ex.line);
		}
	}

	@Override
	public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
		throw new UnsupportedOperationException("This class is obsolete and unsupported.");
	}
}
