package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpreter;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import java.util.List;
import org.apache.log4j.Logger;

public abstract class AbstractWellFormedTestingProofInterpreter<T extends WellFormedTestable>
implements ProofInterpreter<T> {

	private final Logger logger;

	public AbstractWellFormedTestingProofInterpreter(Logger logger) {
		this.logger = logger;
	}

	protected final Logger getLogger() {
		return logger;
	}

	protected abstract T interpretWithoutWellFormednessTest(List<ModalisedAtom> matoms, double cost);

	@Override
	public final T interpret(List<ModalisedAtom> matoms, double cost) {
		logger.debug("going to interpret the proof: " + PrettyPrint.listOfModalisedAtomsToString(matoms));
		
		T result = interpretWithoutWellFormednessTest(matoms, cost);

		if (result == null) {
			logger.warn("interpretation is null, will be scrapped");
			return null;
		}
		else {
			try {
				result.assertWellFormed();
				logger.info("interpretation seems to be well-formed");
				return result;
			}
			catch (WellFormednessException ex) {
				logger.warn("interpretation is not well-formed (will be scrapped): " + ex.getMessage());
				return null;
			}
		}
	}
	
}
