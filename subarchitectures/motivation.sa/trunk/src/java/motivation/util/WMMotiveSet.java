/**
 * 
 */
package motivation.util;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import Ice.ObjectImpl;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMEntrySet.ChangeHandler;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class WMMotiveSet extends WMEntrySet implements ChangeHandler {

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
			if (((MotiveStateTransition) arg0).from == null
					&& this.from == null
					&& ((MotiveStateTransition) arg0).to == null
					&& this.to == null)
				return true;
			if (((MotiveStateTransition) arg0).from == null
					&& this.from != null)
				return false;
			if (((MotiveStateTransition) arg0).to == null && this.to != null)
				return false;
			if (((MotiveStateTransition) arg0).from == null
					&& this.from == null)
				return to.equals(((MotiveStateTransition) arg0).to);
			if (((MotiveStateTransition) arg0).to == null && this.to == null)
				return from.equals(((MotiveStateTransition) arg0).from);
			return from.equals(((MotiveStateTransition) arg0).from)
					&& to.equals(((MotiveStateTransition) arg0).to);
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.lang.Object#hashCode()
		 */
		@Override
		public int hashCode() {
			if (from == null && to == null)
				return 0;
			if (from == null)
				return to.hashCode();
			if (to == null)
				return from.hashCode();
			return from.hashCode() & to.hashCode();
		}

		/**
		 * @param from
		 * @param to
		 */
		public MotiveStateTransition(MotiveStatus from, MotiveStatus to) {
			super();
			this.from = from;
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

	protected WMMotiveSet(ManagedComponent c) {
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
	public static WMMotiveSet create(ManagedComponent c) {
		WMMotiveSet s = new WMMotiveSet(c);
		s.addType(Motive.class);
		return s;
	}

	public void setStateChangeHandler(MotiveStateTransition status,
			ChangeHandler handler) {
		stateChangeReceivers.put(status, handler);
	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMMotiveSet create(ManagedComponent c,
			final Class<? extends Motive> specificType) {
		WMMotiveSet s = new WMMotiveSet(c);
		s.addType(specificType);
		return s;
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

	@Override
	public void motiveChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newObj, ObjectImpl oldObj) {
		if (externalHandler != null)
			externalHandler.motiveChanged(map, wmc, newObj, oldObj);
		Motive newMotive = (Motive) newObj;
		Motive oldMotive = (Motive) oldObj;
		MotiveStatus fromState = null;
		MotiveStatus toState = null;

		if (wmc.operation != WorkingMemoryOperation.DELETE)
			toState = newMotive.status;
		if (wmc.operation != WorkingMemoryOperation.ADD)
			fromState = oldMotive.status;

		Set<ChangeHandler> handlersToCall = new HashSet<ChangeHandler>();

		ChangeHandler enterReceiver = null;

		// call all handlers, also those that a wildcards

		// if state has not changed, simply return
		if (fromState != null && toState != null)
			if (fromState.equals(toState))
				return;

		enterReceiver = stateChangeReceivers.get(new MotiveStateTransition(
				fromState, toState));
		if (enterReceiver != null)
			handlersToCall.add(enterReceiver);

		enterReceiver = stateChangeReceivers.get(new MotiveStateTransition(
				fromState, null));
		if (enterReceiver != null)
			handlersToCall.add(enterReceiver);

		enterReceiver = stateChangeReceivers.get(new MotiveStateTransition(
				null, toState));
		if (enterReceiver != null)
			handlersToCall.add(enterReceiver);

		for (ChangeHandler h : handlersToCall) {
			h.motiveChanged(map, wmc, newMotive, oldMotive);
		}


	}

	public Set<Motive> getSubsetByStatus(MotiveStatus status) {
		Set<Motive> result = new HashSet<Motive>();
		for (ObjectImpl o : super.values()) {
			Motive m = (Motive) o;
			if (m.status.equals(status)) 
				result.add(m);
		}
		return result;
	}

}
