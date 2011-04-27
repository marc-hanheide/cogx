package execution.components;

import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import execution.slice.actions.BeliefPlusStringAction;
import execution.slice.actions.SingleBeliefAction;

public abstract class BeliefBasedPlanExecutionMediator extends
		PlanExecutionMediator {

	
	
	protected WorkingMemoryAddress addressFromFormula(dFormula _form) {
		WMPointer beliefPtr = WMPointer.create(_form);
		return beliefPtr.getVal();
	}
	
	/**
	 * @param _arguments
	 * @param action
	 * @return
	 */
	protected <T extends SingleBeliefAction> SingleBeliefAction beliefPointerToBeliefAction(
			dFormula _argument, T action) {
		action.beliefAddress = addressFromFormula(_argument);
		return action;
	}

	protected <T extends SingleBeliefAction> T createSingleBeliefAction(
			Class<T> _cls, dFormula _belief) throws CASTException {
		T action = newActionInstance(_cls);
		return _cls.cast(beliefPointerToBeliefAction(_belief, action));
	}

	protected String stringFromElementaryFormula(ElementaryFormula _elemForm) {
		GenericFormula<ElementaryFormula> elementaryFormula = GenericFormula
				.create(ElementaryFormula.class, _elemForm);
		return elementaryFormula.getProposition();
	}

	protected <T extends BeliefPlusStringAction> T createBeliefPlusStringAction(
			Class<T> _cls, dFormula _belief, dFormula _value)
			throws CASTException {
		T action = newActionInstance(_cls);
		action.value = stringFromElementaryFormula((ElementaryFormula) _value);
		return _cls.cast(beliefPointerToBeliefAction(_belief, action));
	}

}
