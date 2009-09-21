package execution.util;

import autogen.Planner.Action;
import cast.CASTException;

public interface ActionConverter {

	public execution.slice.Action toSystemAction(Action _plannedAction) throws CASTException;
	
}
