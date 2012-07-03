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

package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.interpret.atoms.AssertedReferenceAtom;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.NoOpNominalRemapper;
import de.dfki.lt.tr.infer.abducer.lang.DisjointDeclaration;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.NullAssumabilityFunction;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.lang.VariableTerm;
import de.dfki.lt.tr.infer.abducer.proof.AssertedQuery;
import de.dfki.lt.tr.infer.abducer.proof.MarkedQuery;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import de.dfki.lt.tr.infer.abducer.proof.UnsolvedQuery;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.apache.log4j.Logger;

/**
 * The class encapsulating the recognition and realisation of communicative
 * intentions.
 *
 * Note that the design is not optimal as it was dictated by the need to
 * use one connection to an inference engine. Most likely it will need to
 * be entirely rewritten.
 *
 * @author Miroslav Janicek
 */
public class IntentionRecognition {

	private final Logger logger;
	private final AbductionEngineConnection abd_recog;

//	public static Counter counter = new Counter("ir");
	public int timeout;

	public static final String INTENTION_RECOGNITION_ENGINE = "IntentionRecognition";

	private final ProofConvertor pconv;

	/**
	 * Initialise the abducer and prepare for action.
	 */
	public IntentionRecognition(String serverName, String endpoints, ProofConvertor pconv, int timeout, Logger logger) {

		assert(logger != null);

		this.timeout = timeout;		

		abd_recog = new AbductionEngineConnection();
		abd_recog.connectToServer(serverName, endpoints);
		abd_recog.bindToEngine(INTENTION_RECOGNITION_ENGINE);
		abd_recog.getEngineProxy().clearContext();

		this.logger = logger;
		this.pconv = pconv;
    }

	public void loadFile(String name) {
		AbducerUtils.loadFile(abd_recog, name);
	}

	public ProofConvertor getProofConvertor() {
		return pconv;
	}

	/**
	 * Try to recognise the communicative intention and underlying beliefs
	 * behind an utterance, represented by a PackedLFs object.
	 *
	 * @param plf the utterance
	 * @return recognised intentions and beliefs when successful, null if error occurred
	 */
	public IntentionRecognitionResult logicalFormToInterpretation(LogicalForm lf, TimeInterval ival) {
		for (ModalisedAtom fact : AbducerUtils.lfToFacts(new Modality[] {Modality.Truth}, lf, NoOpNominalRemapper.INSTANCE)) {
			abd_recog.getEngineProxy().addFact(fact);
		}

		ModalisedAtom g = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(IntentionManagementConstants.humanAgent),
					TermAtomFactory.term(IntentionManagementConstants.thisAgent),
					TermAtomFactory.term(lf.root.nomVar)
				}));

		ProofWithCost pwc = AbducerUtils.bestAbductiveProof(abd_recog, ProofUtils.newUnsolvedProof(g), timeout);
		if (pwc != null) {
			List<ReferenceResolutionRequest> rcs = ConversionUtils.extractReferenceRequests(pconv.getReferenceResolutionRequestExtractor(), lf, pwc.proof, ival);
			return pconv.proofToIntentionRecognitionResult(lf, pwc, lf.preferenceScore, ival, rcs);
		}
		else {
			logger.info("no proof found");
			return null;
		}
	}

	public IntentionRecognitionResult reinterpret(IntentionRecognitionResult irr, ReferenceResolutionResult rr) {

		logger.info("reinterpreting");

		List<ProofWithCost> new_pwcs = new ArrayList<ProofWithCost>();

		for (ProofWithCost pwc : irr.getProofs()) {

			List<MarkedQuery> old_proof = pwc.proof;
			List<MarkedQuery> new_proof = new ArrayList<MarkedQuery>();

			for (MarkedQuery mq : old_proof) {
				List<MarkedQuery> to_add = new ArrayList<MarkedQuery>();
				to_add.add(mq);

				if (mq instanceof AssertedQuery) {
					AssertedReferenceAtom aratom = AssertedReferenceAtom.fromModalisedAtom(mq.atom);
					if (aratom != null) {
						
						Term varTerm = aratom.getReferentTerm();
//						Term epstTerm = aratom.getEpStTerm();

						if (varTerm instanceof VariableTerm) {
							String nom = aratom.getNominal();
							if (rr.nom.equals(nom)) {
								to_add = new ArrayList<MarkedQuery>();

								UnsolvedQuery new_mq = new UnsolvedQuery(aratom.toModalisedAtom(), new NullAssumabilityFunction());
	/*
								UnsolvedQuery resolves_mq = new UnsolvedQuery(
									TermAtomFactory.modalisedAtom(
										new Modality[] {Modality.Understanding},
										TermAtomFactory.atom("resolves_to_epobject", new Term[] {
											nomTerm,
											varTerm,
											TermAtomFactory.var("EpSt")
										})),
									new NamedAssumabilityFunction("reference_resolution"));

								to_add.add(resolves_mq);
	 */
								to_add.add(new_mq);
							}
						}
					}
				}
				new_proof.addAll(to_add);
			}

			new_pwcs.addAll(AbducerUtils.allAbductiveProofs(abd_recog, new_proof, timeout));
		}

		Collections.sort(new_pwcs, new ProofWithCostComparator());

		if (!new_pwcs.isEmpty()) {
			logger.info("got " + new_pwcs.size() + " proofs after reinterpretation");
			IntentionRecognitionResult new_irr = new IntentionRecognitionResult(irr.getLogicalForm(), irr.getInterval(), new_pwcs);
			return new_irr;
		}
		else {
			logger.info("no proofs after reinterpretation");
			return null;
		}
	}

	public void updateReferenceResolution(ReferenceResolutionResult rr) {

		logger.info("updating reference resolution");

		abd_recog.getEngineProxy().clearAssumabilityFunction("reference_resolution");
		Map<String, Set<ModalisedAtom>> disj = new HashMap<String, Set<ModalisedAtom>>();

		String nom = rr.nom;
		for (EpistemicReferenceHypothesis hypo : rr.hypos) {

			ModalisedAtom rma = AssertedReferenceAtom.newAssertedReferenceAtom(nom, hypo.referent, hypo.epst).toModalisedAtom();

			logger.debug("adding reference hypothesis: " + PrettyPrint.modalisedAtomToString(rma) + " @ p=" + hypo.score);
			abd_recog.getEngineProxy().addAssumable("reference_resolution", rma, (float) -Math.log(hypo.score));

			Set<ModalisedAtom> dj = disj.get(nom);
			if (dj != null) {
				dj.add(rma);
			}
			else {
				dj = new HashSet<ModalisedAtom>();
				dj.add(rma);
			}
			disj.put(nom, dj);
		}

		for (String n : disj.keySet()) {
			Set<ModalisedAtom> dj = disj.get(n);
			DisjointDeclaration dd = new DisjointDeclaration();
			dd.atoms = new ArrayList<ModalisedAtom>(dj);
			logger.debug("adding a disjoint declaration for " + n + " (" + dj.size() + " entries)");
			abd_recog.getEngineProxy().addDisjointDeclaration(dd);
		}
	}

	public void clearContext() {
		abd_recog.getEngineProxy().clearContext();
	}

}
