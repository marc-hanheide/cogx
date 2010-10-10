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

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.infer.weigabd.AbductionEngineConnection;
import de.dfki.lt.tr.infer.weigabd.ProofUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.FileReadErrorException;
import de.dfki.lt.tr.infer.weigabd.slice.MarkedQuery;
import de.dfki.lt.tr.infer.weigabd.slice.ModalisedAtom;
import de.dfki.lt.tr.infer.weigabd.slice.Modality;
import de.dfki.lt.tr.infer.weigabd.slice.SyntaxErrorException;
import de.dfki.lt.tr.infer.weigabd.slice.Term;
import java.util.List;

public class IntentionRealization {

	private int timeout;
	public boolean logging = true;
	private AbductionEngineConnection abd_realize;

//	public static Counter counter = new Counter("ir");
	public IdentifierGenerator idGen;

	public static final String INTENTION_REALIZATION_ENGINE = "IntentionRealization";

	/**
	 * Initialise the abducer and prepare for action.
	 */
    public IntentionRealization(IdentifierGenerator idGen_, int timeout_) {
		this.idGen = idGen_;
		this.timeout = timeout_;
		init();
    }

	private void init() {
		abd_realize = new AbductionEngineConnection();
		abd_realize.connectToServer("AbducerServer", "default -p 10000");
		abd_realize.bindToEngine(INTENTION_REALIZATION_ENGINE);
		abd_realize.getProxy().clearContext();
	}

	/**
	 * Try to find a proto-LF that realises the intention and the underlying
	 * beliefs.
	 *
	 * @param it communicative intention
	 * @param bels beliefs needed by itn
	 * @return a proto-LF if successful, null if not
	 */
	public ContentPlanningGoal epistemicObjectsToProtoLF(WorkingMemoryAddress itWma, Intention it, List<dBelief> bels) {

		Term itIdTerm = ConversionUtils.workingMemoryAddressToTerm(itWma);

		// update the abduction context
		for (ModalisedAtom mf : ConversionUtils.intentionToFacts(itIdTerm, it)) {  // XXX
			abd_realize.getProxy().addFact(mf);
		}
/*
		for (dBelief b : bels) {
			for (ModalisedAtom mf : ConversionUtils.beliefToFacts(b)) {
				abd_realize.getProxy().addFact(mf);
			}
		}
*/

		ModalisedAtom g = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Generation,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(IntentionManagementConstants.thisAgent),
					TermAtomFactory.term(IntentionManagementConstants.humanAgent),
					itIdTerm
				}));

		MarkedQuery[] proof = AbducerUtils.bestAbductiveProof(abd_realize, ProofUtils.newUnsolvedProof(g), 250);
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

	public void clearContext() {
		abd_realize.getProxy().clearContext();
	}

	/**
	 * Load a file with rules and facts for the abducer.
	 *
	 * @param file file name
	 */
	public void loadFile(String file) {
		try {
			abd_realize.getProxy().loadFile(file);
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
