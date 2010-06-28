package spatial.motivation;

import VisionData.PeopleDetectionCommand;
import cast.AlreadyExistsOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;

/**
 * Executor which
 * 
 * @author nah
 * 
 */
public class LookForPeopleExecutor extends TurnAndLookExecutor {

	public LookForPeopleExecutor(ManagedComponent _component, int _detections) {
		super(_component, _detections);
	}

	@Override
	protected void triggerDetection() {
		m_component.log("detection triggered");

		
		// Fire off a detection command
		PeopleDetectionCommand detect = new PeopleDetectionCommand();
		String id = m_component.newDataID();
		try {
			m_component
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
							WorkingMemoryOperation.DELETE),
							getAfterDetectionReceiver());
			m_component.addToWorkingMemory(id, detect);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

}
