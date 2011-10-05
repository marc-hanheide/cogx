package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import java.util.List;
import org.apache.log4j.Logger;

public class RobotsIntentionToActProofInterpreter
extends AbstractWellFormedTestingProofInterpreter<RobotsIntentionToAct> {

	public RobotsIntentionToActProofInterpreter(Logger logger) {
		super(logger);
	}

	@Override
	protected RobotsIntentionToAct interpretWithoutWellFormednessTest(List<ModalisedAtom> matoms) {
		throw new UnsupportedOperationException("Not supported yet.");
	}

	
}
