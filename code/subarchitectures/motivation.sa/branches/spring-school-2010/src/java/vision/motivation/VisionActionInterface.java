/**
 * 
 */
package vision.motivation;

import java.util.Map;

import VisionData.DetectionCommand;
import VisionData.PeopleDetectionCommand;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import execution.slice.Action;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.util.ActionExecutor;
import execution.util.ActionExecutorFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteOnDeleteExecutor;

/**
 * Component to listen to planner actions the trigger the vision sa as
 * appropriate. Must be run from the vision sa.
 * 
 * @author nah
 * 
 */
public class VisionActionInterface extends ManagedComponent {

	/**
	 * An action executor to handle object detection.
	 * 
	 * @author nah
	 */
	private class DetectObjectsActionExecutor extends
			NonBlockingCompleteOnDeleteExecutor implements
			WorkingMemoryChangeReceiver {

		private String[] m_labels;

		public DetectObjectsActionExecutor(ManagedComponent _component) {
			super(_component);
		}

		@Override
		public boolean accept(Action _action) {
			DetectObjects action = (DetectObjects) _action;
			m_labels = action.labels;
			return true;
		}

		@Override
		public void executeAction() {
			DetectionCommand cmd = new DetectionCommand(m_labels);
			addThenCompleteOnDelete(new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID()), cmd);
		}

	}

	/**
	 * An action executor to handle object detection.
	 * 
	 * @author nah
	 */
	private class DetectPeopleActionExecutor extends
			NonBlockingCompleteOnDeleteExecutor implements
			WorkingMemoryChangeReceiver {

		private String[] m_labels;

		public DetectPeopleActionExecutor(ManagedComponent _component) {
			super(_component);
		}

		@Override
		public boolean accept(Action _action) {
			return true;
		}

		@Override
		public void executeAction() {
			PeopleDetectionCommand cmd = new PeopleDetectionCommand();
			addThenCompleteOnDelete(new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID()), cmd);
		}

	}

	private class DetectObjectsActionFactory implements ActionExecutorFactory {

		private final ManagedComponent m_component;

		public DetectObjectsActionFactory(ManagedComponent _component) {
			m_component = _component;
		}

		@Override
		public ActionExecutor getActionExecutor() {
			return new DetectObjectsActionExecutor(m_component);
		}

	}

	private class DetectPeopleActionFactory implements ActionExecutorFactory {

		private final ManagedComponent m_component;

		public DetectPeopleActionFactory(ManagedComponent _component) {
			m_component = _component;
		}

		@Override
		public ActionExecutor getActionExecutor() {
			return new DetectPeopleActionExecutor(m_component);
		}

	}

	private LocalActionStateManager m_actionStateManager;

	@Override
	protected void configure(Map<String, String> _config) {
	}

	public VisionActionInterface() {

	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);
		m_actionStateManager.registerActionType(DetectObjects.class,
				new DetectObjectsActionFactory(this));
		m_actionStateManager.registerActionType(DetectPeople.class,
				new DetectPeopleActionFactory(this));
	}

}
