package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;

public interface ContextUpdate {

	public void doUpdate(AbductionEnginePrx engine);

}
