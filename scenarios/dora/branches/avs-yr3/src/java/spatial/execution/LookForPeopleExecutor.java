package spatial.execution;

import VisionData.PeopleDetectionCommand;
import cast.AlreadyExistsOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.actions.LookForPeople;

/**
 * Executor which
 * 
 * @author nah
 * 
 */
public class LookForPeopleExecutor extends TurnAndLookExecutor<LookForPeople> {

	public LookForPeopleExecutor(ManagedComponent _component, int _detections) {
		super(_component, LookForPeople.class, _detections);
	}

	@Override
	protected void triggerDetection() {
		getComponent().log("detection triggered");

		// Fire off a detection command
		PeopleDetectionCommand detect = new PeopleDetectionCommand();
		String id = getComponent().newDataID();
		try {
			getComponent()
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
							WorkingMemoryOperation.DELETE),
							getAfterDetectionReceiver());
			getComponent().addToWorkingMemory(id, detect);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

	@Override
	protected boolean acceptAction(LookForPeople _action) {
		return true;
	}

}
