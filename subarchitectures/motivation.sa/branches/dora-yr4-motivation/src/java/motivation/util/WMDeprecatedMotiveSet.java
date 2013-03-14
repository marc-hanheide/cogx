/**
 * 
 */
package motivation.util;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import Ice.ObjectImpl;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntrySet;
import castutils.castextensions.WMEntrySet.ChangeHandler;

/**
 * @author marc
 * 
 */
@Deprecated
public class WMDeprecatedMotiveSet extends WMEntrySet implements ChangeHandler {

	protected ChangeHandler externalHandler;

	/*
	 * (non-Javadoc)
	 * 
	 * @seemotivation.util.WMEntrySet#setHandler(motivation.util.WMEntrySet.
	 * ChangeHandler)
	 */
	@Override
	public void setHandler(ChangeHandler handler) {
		externalHandler = handler;
	}

	Map<MotiveStateTransition, ChangeHandler> stateChangeReceivers;

	public static class MotiveStateTransition {

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.lang.Object#equals(java.lang.Object)
		 */
		@Override
		public boolean equals(Object arg0) {
			if (!(arg0 instanceof MotiveStateTransition))
				return super.equals(arg0);

			MotiveStateTransition other = (MotiveStateTransition) arg0;
			return to.equals(other.to) && from.equals(other.from);
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.lang.Object#hashCode()
		 */
		@Override
		public int hashCode() {
			return from.hashCode() & to.hashCode();
		}

		/**
		 * @param from
		 * @param to
		 */
		public MotiveStateTransition(MotiveStatus from, MotiveStatus to) {
			super();
			if (from == null)
				this.from = MotiveStatus.WILDCARD;
			else
				this.from = from;
			if (to == null)
				this.to = MotiveStatus.WILDCARD;
			else
				this.to = to;
		}

		/**
		 * @return the from
		 */
		public MotiveStatus getFrom() {
			return from;
		}

		/**
		 * @return the to
		 */
		public MotiveStatus getTo() {
			return to;
		}

		private MotiveStatus from;
		private MotiveStatus to;
	}

	protected WMDeprecatedMotiveSet(ManagedComponent c) {
		super(c);
		stateChangeReceivers = new HashMap<MotiveStateTransition, ChangeHandler>();
		super.setHandler(this);
	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMDeprecatedMotiveSet create(ManagedComponent c) {
		WMDeprecatedMotiveSet s = new WMDeprecatedMotiveSet(c);
		s.addType(Motive.class);
		return s;
	}

	public void setStateChangeHandler(MotiveStateTransition status,
			ChangeHandler handler) {
		stateChangeReceivers.put(status, handler);
	}

	public void setStateChangeHandler(MotiveStateTransition status,
			final WorkingMemoryChangeReceiver handler) {
		stateChangeReceivers.put(status, new WMEntrySet.ChangeHandler() {

			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, ObjectImpl> map,
					WorkingMemoryChange wmc, ObjectImpl newMotive,
					ObjectImpl oldMotive) throws CASTException {
				handler.workingMemoryChanged(wmc);

			}
		});
	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMDeprecatedMotiveSet create(ManagedComponent c,
			final Class<? extends Motive> specificType) {
		WMDeprecatedMotiveSet s = new WMDeprecatedMotiveSet(c);
		s.addType(specificType);
		return s;
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newObj, ObjectImpl oldObj)
			throws CASTException {
		if (externalHandler != null)
			externalHandler.entryChanged(map, wmc, newObj, oldObj);
		Motive newMotive = (Motive) newObj;
		Motive oldMotive = (Motive) oldObj;
		MotiveStatus fromState = MotiveStatus.WILDCARD;
		MotiveStatus toState = MotiveStatus.WILDCARD;

		if (wmc.operation != WorkingMemoryOperation.DELETE)
			toState = newMotive.status;
		if (wmc.operation != WorkingMemoryOperation.ADD)
			fromState = oldMotive.status;

		Set<ChangeHandler> handlersToCall = new HashSet<ChangeHandler>();

		ChangeHandler enterReceiver = null;

		// call all handlers, including wildcards
		component.debug("status transition check " + fromState + " -> " + toState);
		enterReceiver = stateChangeReceivers.get(new MotiveStateTransition(
				fromState, toState));
		if (enterReceiver != null) {
			component.log("call " + fromState + " -> " + toState + " with "
					+ fromState + " -> " + toState);
			handlersToCall.add(enterReceiver);
		}
		// if state has not changed, don't call the wildcard handlers
		if (!fromState.equals(toState)) {
			enterReceiver = stateChangeReceivers.get(new MotiveStateTransition(
					fromState, MotiveStatus.WILDCARD));
			if (enterReceiver != null) {
				component.log("call " + fromState + " -> *" + " with "
						+ fromState + " -> " + toState);
				handlersToCall.add(enterReceiver);
			}

			enterReceiver = stateChangeReceivers.get(new MotiveStateTransition(
					MotiveStatus.WILDCARD, toState));
			if (enterReceiver != null) {
				component.log("call * -> " + toState + " with " + fromState
						+ " -> " + toState);
				handlersToCall.add(enterReceiver);
			}
		}
		for (ChangeHandler h : handlersToCall) {
			component.log("call handler");
			h.entryChanged(map, wmc, newMotive, oldMotive);
		}

	}

	public void setState(MotiveStatus status, Collection<Motive> motives) {
		for (Motive m : motives) {
			try {
				component.getMemoryEntry(m.thisEntry, Motive.class);
				m.status = status;
				component.overwriteWorkingMemory(m.thisEntry, m);
			} catch (CASTException e) {
				component.println("*** CASTException in setState " + e.message);
			}
		}
	}

	public Map<WorkingMemoryAddress, Motive> getMapByStatus(MotiveStatus status) {
		Map<WorkingMemoryAddress, Motive> result = new HashMap<WorkingMemoryAddress, Motive>();
		for (Entry<WorkingMemoryAddress, ObjectImpl> o : super.entrySet()) {
			Motive m = (Motive) o.getValue();
			if (m.status.equals(status))
				result.put(o.getKey(), m);
		}
		return result;
	}

	public Map<WorkingMemoryAddress, Motive> getMapByType(
			Class<? extends Motive> className) {
		Map<WorkingMemoryAddress, Motive> result = new HashMap<WorkingMemoryAddress, Motive>();
		for (Entry<WorkingMemoryAddress, ObjectImpl> o : super.entrySet()) {
			if (o.getValue().getClass().equals(className)) {
				Motive m = (Motive) o.getValue();
				result.put(o.getKey(), m);
			}
		}
		return result;
	}

}
