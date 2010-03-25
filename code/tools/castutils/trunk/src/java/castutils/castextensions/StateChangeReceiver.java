package castutils.castextensions;

import java.util.HashMap;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryReaderComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * Creates and registers a receiver which is triggered when a working memory
 * entry transitions from one state to another. State change is determined by
 * comparing the result of the provided Accessor to the before and after values.
 * Once an entry has passed from returning before to returning after, the
 * provided receiver is called with this final change. If
 * _allowIntermediateChanges is set to true then any sequences of changes from
 * before to after are allowed. If this value is set to false then the state
 * change is only counted if the next different value seen after before is
 * after. Instances of this class should not be explicitly added as a change receiver used addChangeFilter; undefined behaviour will insue. 
 * 
 * @author nah
 * 
 */
public class StateChangeReceiver<EntryType extends Ice.Object, MemberType>
		implements WorkingMemoryChangeReceiver {

	

	private class StateTransitionMonitor {

		boolean m_startTransition;

		public StateTransitionMonitor(WorkingMemoryAddress _address)
				throws DoesNotExistOnWMException,
				UnknownSubarchitectureException {
			m_startTransition = false;

			// read current state of the entry, this should be an early,
			// preferably first, version
			MemberType val = readValue(_address);
			m_startTransition = checkEquals(val, m_before);
		}

		/**
		 * @param val
		 * @param _m_before
		 * @return
		 */
		private boolean checkEquals(MemberType val, MemberType _m_before) {
			if (val == null) {
				if (_m_before == null) {
					return true;
				}
			} else {
				if (val.equals(_m_before)) {
					return true;
				}
			}
			return false;
		}

		/**
		 * @param _address
		 * @throws DoesNotExistOnWMException
		 * @throws UnknownSubarchitectureException
		 */
		private MemberType readValue(WorkingMemoryAddress _address)
				throws DoesNotExistOnWMException,
				UnknownSubarchitectureException {
			EntryType entry = m_component
					.getMemoryEntry(_address, m_entryClass);
			return m_accessor.access(entry);
		}

		public boolean checkForTransition(WorkingMemoryAddress _address)
				throws DoesNotExistOnWMException,
				UnknownSubarchitectureException {
			MemberType value = readValue(_address);

			if (!m_startTransition) {
				m_startTransition = checkEquals(value, m_before);
			} else {
				if (checkEquals(value, m_after)) {
					m_startTransition = false;
					return true;
				}
				else if(!m_allowIntermediateChanges && !checkEquals(value, m_before)) {
					m_startTransition = false;
				}
			}
			return false;

		}
	}

	private final WorkingMemoryReaderComponent m_component;
	private final HashMap<WorkingMemoryAddress, StateTransitionMonitor> m_transitionMonitors;
	private final Class<EntryType> m_entryClass;
	private final Accessor<EntryType,MemberType> m_accessor;
	private final MemberType m_before;
	private final MemberType m_after;
	private final WorkingMemoryChangeReceiver m_receiver;
	private boolean m_allowIntermediateChanges;

	public StateChangeReceiver(WorkingMemoryReaderComponent _component,
			Class<EntryType> _type, Accessor<EntryType,MemberType> _accessor, MemberType _before,
			MemberType _after, boolean _allowIntermediateChanges,
			WorkingMemoryChangeReceiver _receiver) {

		m_component = _component;
		m_transitionMonitors = new HashMap<WorkingMemoryAddress, StateTransitionMonitor>();
		m_entryClass = _type;
		m_accessor = _accessor;
		m_before = _before;
		m_after = _after;
		m_receiver = _receiver;

		m_component.addChangeFilter(ChangeFilterFactory
				.createGlobalTypeFilter(_type), this);
	}

	public StateChangeReceiver(WorkingMemoryReaderComponent _component,
			WorkingMemoryAddress _address, Class<EntryType> _type,
			Accessor<EntryType,MemberType> _accessor, MemberType _before, MemberType _after,
			boolean _allowIntermediateChanges,
			WorkingMemoryChangeReceiver _receiver) {

		m_component = _component;
		m_transitionMonitors = new HashMap<WorkingMemoryAddress, StateTransitionMonitor>();
		m_entryClass = _type;
		m_accessor = _accessor;
		m_before = _before;
		m_after = _after;
		m_receiver = _receiver;
		m_allowIntermediateChanges = _allowIntermediateChanges;
		m_component.addChangeFilter(ChangeFilterFactory
				.createAddressFilter(_address), this);
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {

		if (_wmc.operation == WorkingMemoryOperation.DELETE) {
			m_transitionMonitors.remove(_wmc.address);
		} else {
			StateTransitionMonitor monitor = m_transitionMonitors
					.get(_wmc.address);
			if (monitor == null) {
				m_transitionMonitors.put(_wmc.address,
						new StateTransitionMonitor(_wmc.address));
			} else {
				if (monitor.checkForTransition(_wmc.address)) {
					m_receiver.workingMemoryChanged(_wmc);
				}
			}
		}
	}

	/**
	 * Remove filters added by this calls. Just calls
	 * m_component.removeChangeFilter(this) so the component can do this itself too.
	 * 
	 * @throws SubarchitectureComponentException
	 */
	public void remove() throws SubarchitectureComponentException {
		m_component.removeChangeFilter(this);
	}

}
