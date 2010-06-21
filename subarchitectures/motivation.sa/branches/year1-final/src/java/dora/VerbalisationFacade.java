package dora;

import java.util.Hashtable;
import java.util.Map;

import motivation.util.castextensions.Accessor;
import motivation.util.castextensions.StateChangeReceiver;
import motivation.util.castextensions.WMEntrySet;
import motivation.util.castextensions.WMEntrySet.ChangeHandler;
import Ice.ObjectImpl;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

import comsys.datastructs.comsysEssentials.ContentPlanningGoal;
import comsys.datastructs.lf.LogicalForm;
import comsys.lf.utils.LFUtils;

/**
 * Essentially a set of of
 * 
 * @author nah
 * 
 */
public class VerbalisationFacade {

	private String m_comsysSA;
	private final ManagedComponent m_component;
	private final Map<Class<? extends Ice.ObjectImpl>, WMEntrySet> m_deleteBuffers;

	public VerbalisationFacade(ManagedComponent _component) {
		m_component = _component;
		m_deleteBuffers = new Hashtable<Class<? extends Ice.ObjectImpl>, WMEntrySet>();
	}

	public void configure(Map<String, String> args) {
		m_comsysSA = "comsys";
		if (args.containsKey("--comsys")) {
			m_comsysSA = args.get("--comsys");
		}
	}

	protected LogicalForm lfForCannedText(String _cannedText) {
		if (_cannedText.contains(" ")) {
			_cannedText = _cannedText.replaceAll(" ", "_");
		}
		String lfGoal = CASTUtils.concatenate("@d1:dvp(c-goal ^ <CannedText>",
				_cannedText, " )");
		// m_component.log(lfGoal);
		return LFUtils.convertFromString(lfGoal);
	}

	/**
	 * @param _text
	 */
	protected void verbaliseCannedText(String _text) {
		if (!_text.isEmpty()) {
			LogicalForm _greetingLF = lfForCannedText(_text);
			verbaliseLF(_greetingLF);
		}
	}

	/**
	 * @param _greetingLF
	 */
	private void verbaliseLF(LogicalForm _greetingLF) {
		ContentPlanningGoal _cpGoal = new ContentPlanningGoal(m_component
				.newDataID(), _greetingLF);
		try {
			m_component.addToWorkingMemory(new WorkingMemoryAddress(m_component
					.newDataID(), m_comsysSA), _cpGoal);
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Verbalise some canned text on state transition.
	 * 
	 * @param <EntryType>
	 * @param <MemberType>
	 * @param _entryClass
	 * @param _accessor
	 * @param _before
	 * @param _after
	 * @param _cannedText
	 */
	public <EntryType extends Ice.ObjectImpl, MemberType> void verbaliseCannedTextOnStateTransition(
			Class<EntryType> _entryClass,
			Accessor<EntryType, MemberType> _accessor, MemberType _before,
			MemberType _after, final String _cannedText) {

		new StateChangeReceiver<EntryType, MemberType>(m_component,
				_entryClass, _accessor, _before, _after, false,
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_cannedText);
					}
				});

	}

	public <EntryType extends Ice.ObjectImpl, MemberType> void verbaliseOnStateTransition(
			final Class<EntryType> _entryClass,
			Accessor<EntryType, MemberType> _accessor, MemberType _before,
			MemberType _after, final TextGenerator<EntryType> _generator) {

		new StateChangeReceiver<EntryType, MemberType>(m_component,
				_entryClass, _accessor, _before, _after, false,
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_generator.toText(m_component
								.getMemoryEntry(_arg0.address, _entryClass)));
					}
				});

	}

	public <EntryType extends Ice.ObjectImpl> void verbaliseOnAddition(
			final Class<EntryType> _entryClass,
			final TextGenerator<EntryType> _generator) {

		m_component.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				_entryClass, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_generator.toText(m_component
								.getMemoryEntry(_arg0.address, _entryClass)));
					}
				});
	}

	public <EntryType extends Ice.ObjectImpl> void verbaliseCannedTextOnAddition(
			final Class<EntryType> _entryClass, final String _text) {

		m_component.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				_entryClass, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_text);
					}
				});
	}

	public <EntryType extends Ice.ObjectImpl> void verbaliseOnOverwrite(
			final Class<EntryType> _entryClass,
			final TextGenerator<EntryType> _generator) {

		m_component.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				_entryClass, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_generator.toText(m_component
								.getMemoryEntry(_arg0.address, _entryClass)));
					}
				});
	}

	public <EntryType extends Ice.ObjectImpl> void verbaliseCannedTextOnOverwrite(
			final Class<EntryType> _entryClass, final String _text) {

		m_component.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				_entryClass, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_text);
					}
				});
	}

	/**
	 * Verbalise when an object of this type is deleted. The generator is passed
	 * the last available instance of the deleted object. Keeps track of objects
	 * using a WMEntrySet.
	 * 
	 * @param <EntryType>
	 * @param _entryClass
	 * @param _generator
	 */
	public <EntryType extends Ice.ObjectImpl> void verbaliseOnDeletion(
			final Class<EntryType> _entryClass,
			final TextGenerator<EntryType> _generator) {

		WMEntrySet entrySet = WMEntrySet.create(m_component, _entryClass);
		entrySet.setHandler(new ChangeHandler() {

			@SuppressWarnings("unchecked")
			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, ObjectImpl> _map,
					WorkingMemoryChange _wmc, ObjectImpl _newMotive,
					ObjectImpl _oldMotive) throws CASTException {
				if (_wmc.operation == WorkingMemoryOperation.DELETE) {
					verbaliseCannedText(_generator
							.toText((EntryType) _oldMotive));
				}

			}
		});

		m_deleteBuffers.put(_entryClass, entrySet);

		m_component.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				_entryClass, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
					}
				});
	}

	public <EntryType extends Ice.ObjectImpl> void verbaliseCannedTextOnDeletion(
			final Class<EntryType> _entryClass, final String _text) {

		m_component.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				_entryClass, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_text);
					}
				});
	}

}