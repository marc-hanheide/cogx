package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import java.util.List;
import org.apache.log4j.Logger;

public class RobotCommunicativeActionProofInterpreter
extends AbstractWellFormedTestingProofInterpreter<RobotCommunicativeAction> {

	public RobotCommunicativeActionProofInterpreter(Logger logger) {
		super(logger);
	}

	@Override
	protected RobotCommunicativeAction interpretWithoutWellFormednessTest(List<ModalisedAtom> matoms) {
		RobotCommunicativeAction ra = new RobotCommunicativeAction();
		LogicalForm lf = AbducerUtils.factsToLogicalForm(matoms, "dn1_1");
		ra.setProtoLF(lf);
		return ra;
	}

	
}
