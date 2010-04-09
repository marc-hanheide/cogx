/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import cast.cdl.CASTTime;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import binder.arch.BinderException;


/**
 * @author marc
 *
 */
public interface TransferFunction<From extends Ice.ObjectImpl,To extends Belief>   {
	public void transform(From f, To t) throws BinderException, InterruptedException;

	public PerceptBelief createBelief(String id, CASTTime curTime) throws BinderException, BeliefException;
}
