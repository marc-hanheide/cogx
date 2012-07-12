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
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMView;

/**
 * @author marc
 * 
 */
public class WMMotiveView extends WMView<Motive> implements
		WMView.ChangeHandler<Motive> {

	public static class MotiveStateTransition {

		private MotiveStatus from;

		private MotiveStatus to;

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

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.lang.Object#hashCode()
		 */
		@Override
		public int hashCode() {
			return from.hashCode() & to.hashCode();
		}
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMMotiveView create(ManagedComponent c) {
		WMMotiveView s = new WMMotiveView(c);
		return s;
	}

	protected ChangeHandler<Motive> externalHandler;

	Map<MotiveStateTransition, ChangeHandler<Motive>> stateChangeReceivers;

	protected WMMotiveView(ManagedComponent c) {
		super(c, Motive.class, new ExceptAllFilter<Motive>());
		stateChangeReceivers = new HashMap<MotiveStateTransition, ChangeHandler<Motive>>();
		super.registerHandler(this);
	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
			WorkingMemoryChange wmc, Motive newObj, Motive oldObj)
			throws CASTException {
		if (externalHandler != null) {
			externalHandler.entryChanged(map, wmc, newObj, oldObj);
		}

		Motive newMotive = newObj;
		Motive oldMotive = oldObj;
		MotiveStatus fromState = MotiveStatus.WILDCARD;
		MotiveStatus toState = MotiveStatus.WILDCARD;

		if (wmc.operation != WorkingMemoryOperation.DELETE) {
			toState = newMotive.status;
		}
		if (wmc.operation != WorkingMemoryOperation.ADD) {
			fromState = oldMotive.status;
		}

		Set<ChangeHandler<Motive>> handlersToCall = new HashSet<ChangeHandler<Motive>>();

		ChangeHandler<Motive> enterReceiver = null;

		// call all handlers, including wildcards
		component.debug("status transition check " + fromState + " -> "
				+ toState);
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
		for (ChangeHandler<Motive> h : handlersToCall) {
			component.log("call handler");
			h.entryChanged(map, wmc, newMotive, oldMotive);
		}

	}

	public Map<WorkingMemoryAddress, Motive> getMapByStatus(MotiveStatus status) {
		Map<WorkingMemoryAddress, Motive> result = new HashMap<WorkingMemoryAddress, Motive>();
		for (Entry<WorkingMemoryAddress, Motive> o : super.entrySet()) {
			Motive m = o.getValue();
			if (m.status.equals(status))
				result.put(o.getKey(), m);
		}
		return result;
	}

	public Map<WorkingMemoryAddress, Motive> getMapByType(
			Class<? extends Motive> className) {
		Map<WorkingMemoryAddress, Motive> result = new HashMap<WorkingMemoryAddress, Motive>();
		for (Entry<WorkingMemoryAddress, Motive> o : super.entrySet()) {
			if (o.getValue().getClass().equals(className)) {
				Motive m = o.getValue();
				result.put(o.getKey(), m);
			}
		}
		return result;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @seemotivation.util.WMEntrySet#setHandler(motivation.util.WMEntrySet.
	 * ChangeHandler)
	 */
	@Override
	public void setHandler(ChangeHandler<Motive> handler) {
		externalHandler = handler;
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

	public void setStateChangeHandler(MotiveStateTransition status,
			ChangeHandler<Motive> handler) {
		stateChangeReceivers.put(status, handler);
	}

	public void setStateChangeHandler(MotiveStateTransition status,
			final WorkingMemoryChangeReceiver handler) {
		stateChangeReceivers.put(status, new ChangeHandler<Motive>() {

			@Override
			public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
					WorkingMemoryChange wmc, Motive newEntry, Motive oldEntry)
					throws CASTException {
				handler.workingMemoryChanged(wmc);

			}
		});
	}

}
