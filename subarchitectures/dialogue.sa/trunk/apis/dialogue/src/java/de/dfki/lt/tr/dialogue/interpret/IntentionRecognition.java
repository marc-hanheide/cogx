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

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
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

	public boolean logging = true;
	private AbductionEngineConnection abd_recog;

//	public static Counter counter = new Counter("ir");
	public IdentifierGenerator idGen;

	public static final String INTENTION_RECOGNITION_ENGINE = "IntentionRecognition";

	/**
	 * Initialise the abducer and prepare for action.
	 */
    public IntentionRecognition(IdentifierGenerator idGen_) {
		this.idGen = idGen_;
		init();
    }

	private void init() {
		abd_recog = new AbductionEngineConnection();
		abd_recog.connectToServer("AbducerServer", "default -p 10000");
		abd_recog.bindToEngine(INTENTION_RECOGNITION_ENGINE);
		abd_recog.getProxy().clearContext();
	}

	/**
	 * Try to recognise the communicative intention and underlying beliefs
	 * behind an utterance, represented by a PackedLFs object.
	 *
	 * @param plf the utterance
	 * @return recognised intentions and beliefs when successful, null if error occurred
	 */
	public RecognisedIntention logicalFormToEpistemicObjects(LogicalForm lf) {
		for (ModalisedAtom fact : AbducerUtils.lfToFacts(new Modality[] {Modality.Truth}, lf)) {
			abd_recog.getProxy().addFact(fact);
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

		MarkedQuery[] proof = AbducerUtils.bestAbductiveProof(abd_recog, ProofUtils.newUnsolvedProof(g), 250);
		if (proof != null) {
			return ConversionUtils.proofToEpistemicObjects(idGen, IntentionManagementConstants.humanAgent, proof);
		}
		else {
			log("no proof found");
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
			abd_recog.getProxy().loadFile(file);
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
