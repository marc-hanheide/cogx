package spatial.motivation;

import VisionData.DetectionCommand;
import cast.AlreadyExistsOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.Action;
import execution.slice.actions.LookForObjects;

public class LookForObjectsExecutor extends TurnAndLookExecutor {

	private String[] m_labels;

	public LookForObjectsExecutor(ManagedComponent _component, int _detections) {
		super(_component, _detections);
	}

	@Override
	public boolean accept(Action _action) {
		m_labels = ((LookForObjects) _action).labels;
		return true;
	}

	@Override
	protected void triggerDetection() {
		m_component.log("detection triggered");
		// Fire off a detection command
		DetectionCommand detect = new DetectionCommand(m_labels);
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
