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

import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.weigabd.AbductionEngineConnection;
import de.dfki.lt.tr.infer.weigabd.MercuryUtils;
import de.dfki.lt.tr.infer.weigabd.ProofUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.FileReadErrorException;
import de.dfki.lt.tr.infer.weigabd.slice.MarkedQuery;
import de.dfki.lt.tr.infer.weigabd.slice.ProofWithCost;
import de.dfki.lt.tr.infer.weigabd.slice.ModalisedAtom;
import de.dfki.lt.tr.infer.weigabd.slice.Modality;
import de.dfki.lt.tr.infer.weigabd.slice.NoProofException;
import de.dfki.lt.tr.infer.weigabd.slice.ProveResult;
import de.dfki.lt.tr.infer.weigabd.slice.SyntaxErrorException;
import de.dfki.lt.tr.infer.weigabd.slice.Term;
import java.util.LinkedList;
import java.util.List;

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
public class IntentionManagement {

	public boolean logging = true;
	private AbductionEngineConnection abd;

//	public static Counter counter = new Counter("ir");
	public IdentifierGenerator idGen;

	public static final String thisAgent = "robot";
	public static final String humanAgent = "human";

	public static final String discRefModality = "lingref";
	public static final String stateModality = "state";
	public static final String beliefLinkModality = "belief";

	/**
	 * Initialise the abducer and prepare for action.
	 */
    public IntentionManagement(IdentifierGenerator idGen_) {
		this.idGen = idGen_;
		init();
    }

	private void init() {
		abd = new AbductionEngineConnection();
		abd.connectToServer("AbducerServer", "default -p 10000");
		abd.bindToEngine("IntentionManagementEngine");
		abd.getProxy().clearContext();
	}

	/**
	 * Try to recognise the communicative intention and underlying beliefs
	 * behind an utterance, represented by a PackedLFs object.
	 *
	 * @param plf the utterance
	 * @return recognised intentions and beliefs when successful, null if error occurred
	 */
	public RecognisedIntention logicalFormToEpistemicObjects(LogicalForm lf) {
//		log("expanding LF into facts");
		for (ModalisedAtom fact : AbducerUtils.lfToFacts(new Modality[] {Modality.Truth}, lf)) {
//			log("  add fact: " + MercuryUtils.modalisedAtomToString(fact));
			abd.getProxy().addFact(fact);
		}

		ModalisedAtom g = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(humanAgent),
					TermAtomFactory.term(thisAgent),
					TermAtomFactory.term(lf.root.nomVar)
				}));

		MarkedQuery[] proof = abductiveProof(ProofUtils.newUnsolvedProof(g));
		if (proof != null) {
			return ConversionUtils.proofToEpistemicObjects(idGen, humanAgent, proof);
		}
		else {
			log("no proof found");
			return null;
		}
	}

	/**
	 * Try to find a proto-LF that realises the intention and the underlying
	 * beliefs.
	 *
	 * @param itn communicative intention
	 * @param bels beliefs needed by itn
	 * @return a proto-LF if successful, null if not
	 */
	public ContentPlanningGoal epistemicObjectsToProtoLF(Intention itn, List<dBelief> bels) {

		// update the abduction context
		for (ModalisedAtom mf : ConversionUtils.intentionToFacts(itn)) {
			abd.getProxy().addFact(mf);
		}
		for (dBelief b : bels) {
			for (ModalisedAtom mf : ConversionUtils.beliefToFacts(b)) {
				abd.getProxy().addFact(mf);
			}
		}

		ModalisedAtom g = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Generation,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(thisAgent),
					TermAtomFactory.term(humanAgent),
					TermAtomFactory.term(itn.id)
				}));

		MarkedQuery[] proof = abductiveProof(ProofUtils.newUnsolvedProof(g));
		if (proof != null) {
			return proofToProtoLF(proof);
		}
		else {
			log("no proof found");
			return null;
		}
	}

	private ContentPlanningGoal proofToProtoLF(MarkedQuery[] proof) {
		ModalisedAtom[] imfs = ProofUtils.filterStripByModalityPrefix(
				ProofUtils.stripMarking(ProofUtils.filterAssumed(proof)),
				new Modality[] {Modality.Truth});
		LogicalForm lf = AbducerUtils.factsToLogicalForm(imfs, "dn1_1");
		ContentPlanningGoal plf = new ContentPlanningGoal();
		plf.cpgid = idGen.newIdentifier();
		plf.lform = lf;
		return plf;
	}

	private MarkedQuery[] abductiveProof(MarkedQuery[] goal) {
		String listGoalsStr = "";
		for (int i = 0; i < goal.length; i++) {
			listGoalsStr += MercuryUtils.modalisedAtomToString(goal[i].atom);
			if (i < goal.length - 1) listGoalsStr += ", ";
		}
		log("proving: [" + listGoalsStr + "]");

		abd.getProxy().startProving(goal);
		ProofWithCost[] result = abd.getProxy().getProofs(250);
		if (result.length > 0) {
			log("found " + result.length + " proofs, picking the best one");
			return result[0].proof;
		}
		else {
			return null;
		}
	}

	/**
	 * Load a file with rules and facts for the abducer.
	 *
	 * @param file file name
	 */
	public void loadFile(String file) {
		try {
			abd.getProxy().loadFile(file);
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
			System.out.println("\033[32m[IR]\t" + str + "\033[0m");
	}

}
