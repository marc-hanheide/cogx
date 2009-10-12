package dora;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.castextensions.Accessor;
import motivation.util.castextensions.StateChangeReceiver;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;

import comsys.datastructs.comsysEssentials.ContentPlanningGoal;
import comsys.datastructs.lf.LogicalForm;
import comsys.lf.utils.LFUtils;

/**
 * - found new place N - found new placeholder N - found new room N - seen
 * object O - room N is categorised as a C - found a doorway - going to place N
 * - going room categorise room b - activated goals G - planning for active
 * goals - planning succeeded/failed - executing plan - executing next action -
 * action succeeded/failed - plan execution succeeded/failed
 * 
 * 
 * subarchitectures/comsys/grammars/contentPlanning/doc/cogx-testbed.xml
 * 
 * @author nah
 * 
 */
public class Verbalisation extends ManagedComponent {

	private String m_comsysSA;

	private static Accessor<Motive, MotiveStatus> MOTIVE_STATUS_ACCESSOR = new Accessor<Motive, MotiveStatus>() {
		@Override
		public MotiveStatus access(Motive _entry) {
			return _entry.status;
		}
	};

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
		log(lfGoal);
		return LFUtils.convertFromString(lfGoal);
	}

	public void start() {
		verbaliseCannedTextOnStateTransition(Motive.class,
				MOTIVE_STATUS_ACCESSOR, MotiveStatus.SURFACED,
				MotiveStatus.ACTIVE, "motive activated");
	}

	public void runComponent() {
		verbaliseCannedText("hello world");
	}

	/**
	 * @param greeting
	 */
	private void verbaliseCannedText(String greeting) {
		LogicalForm _greetingLF = lfForCannedText(greeting);
		verbaliseLF(_greetingLF);
	}

	/**
	 * @param _greetingLF
	 */
	private void verbaliseLF(LogicalForm _greetingLF) {
		ContentPlanningGoal _cpGoal = new ContentPlanningGoal(newDataID(),
				_greetingLF);
		try {
			addToWorkingMemory(
					new WorkingMemoryAddress(newDataID(), m_comsysSA), _cpGoal);
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
	public <EntryType extends Ice.Object, MemberType> void verbaliseCannedTextOnStateTransition(
			Class<EntryType> _entryClass,
			Accessor<EntryType, MemberType> _accessor, MemberType _before,
			MemberType _after, final String _cannedText) {

		new StateChangeReceiver<EntryType, MemberType>(this, _entryClass,
				_accessor, _before, _after, false,
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						verbaliseCannedText(_cannedText);
					}
				});

	}

}
