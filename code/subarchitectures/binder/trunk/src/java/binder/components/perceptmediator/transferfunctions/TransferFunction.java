/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import cast.cdl.CASTTime;
import binder.arch.BinderException;
import binder.autogen.beliefs.Belief;
import binder.autogen.beliefs.PerceptBelief;

/**
 * @author marc
 *
 */
public interface TransferFunction<From extends Ice.ObjectImpl,To extends Belief>   {
	public void transform(From f, To t) throws BinderException, InterruptedException;

	public PerceptBelief createBelief(String id, CASTTime curTime) throws BinderException;
}
