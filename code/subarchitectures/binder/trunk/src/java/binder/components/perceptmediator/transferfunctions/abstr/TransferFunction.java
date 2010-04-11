/**
 * 
 */
package binder.components.perceptmediator.transferfunctions.abstr;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;

/**
 * @author marc
 *
 */
public interface TransferFunction<From extends Ice.ObjectImpl,To extends Belief>   {
	public void transform(From f, To t) throws BeliefException, InterruptedException;

	public PerceptBelief createBelief(String id, WorkingMemoryAddress srcAddr, String type, CASTTime curTime) throws BeliefException;

}
