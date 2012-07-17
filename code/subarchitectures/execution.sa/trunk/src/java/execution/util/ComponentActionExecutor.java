package execution.util;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import execution.slice.Action;

public abstract class ComponentActionExecutor<ActionClass extends Action>
		implements ActionExecutor<ActionClass> {

	private final ManagedComponent m_component;

	private boolean m_hasCompleted = false;

	public void debug(Object _o, Throwable _t) {
		m_component.debug(_o, _t);
	}

	public void debug(Object _o) {
		m_component.debug(_o);
	}

	public void log(Object _o, Throwable _t) {
		m_component.log(_o, _t);
	}

	public void logException(Object _message, Throwable _t) {
		m_component.logException(_message, _t);
	}

	public void logException(Throwable _t) {
		m_component.logException(_t);
	}

	public void println(Object _o, Throwable _t) {
		m_component.println(_o, _t);
	}

	public ComponentActionExecutor(ManagedComponent _component) {
		m_component = _component;
	}

	protected ManagedComponent getComponent() {
		return m_component;
	}

	protected void log(Object _o) {
		m_component.log(_o);
	}

	protected void println(Object _o) {
		m_component.println(_o);
	}
	
	protected void warn(Object _o) {
		m_component.getLogger().warn(_o, m_component.getLogAdditions());
	}

	protected WorkingMemoryAddress newWorkingMemoryAddress() {
		return new WorkingMemoryAddress(getComponent().newDataID(),
				getComponent().getSubarchitectureID());
	}

	public void setCompletion(boolean _hasCompleted) {
		m_hasCompleted = _hasCompleted;
	}

	@Override
	public boolean hasCompleted() {
		return m_hasCompleted;
	}

}
