package spatial.execution;

import VisionData.DetectionCommand;
import cast.AlreadyExistsOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.actions.LookForObjects;

public class LookForObjectsExecutor extends TurnAndLookExecutor<LookForObjects> {

	private String[] m_labels;

	public LookForObjectsExecutor(ManagedComponent _component, int _detections) {
		super(_component, LookForObjects.class, _detections);
	}



	@Override
	protected void triggerDetection() {
		getComponent().log("detection triggered");
		// Fire off a detection command
		DetectionCommand detect = new DetectionCommand(m_labels);
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
	protected boolean acceptAction(LookForObjects _action) {
		m_labels = _action.labels;
		return true;
	}


}
