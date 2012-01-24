package execution.util;

import execution.slice.Action;


public interface ActionExecutorFactory<ActionClass extends Action> {
	public ActionExecutor<ActionClass> getActionExecutor();
}
