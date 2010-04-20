/**
 * 
 */
package vision.motivation;

import java.util.Map;

import VisionData.DetectionCommand;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.DetectObjects;
import execution.util.ActionExecutor;
import execution.util.ActionExecutorFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingActionExecutor;

/**
 * Component to listen to planner actions the trigger the vision sa as
 * appropriate. Must be run from the vision sa.
 * 
 * @author nah
 * 
 */
public class VisionActionInterface extends ManagedComponent {
	/**
	 * An action executor to handle object detection. You can construct this
	 * with a default list of object model labels to use, which can be altered
	 * with setModels.
	 * 
	 * @author nah
	 * 
	 */
	public class DetectObjectsActionExecutor extends NonBlockingActionExecutor
			implements WorkingMemoryChangeReceiver {

		private String[] m_labels;
		private String m_cmdID;

		@Override
		public boolean accept(Action _action) {
			DetectObjects action = (DetectObjects) _action;
			m_labels = action.labels;
			return true;
		}

		@Override
		public void executeAction() {
			DetectionCommand cmd = new DetectionCommand(m_labels);
			m_cmdID = newDataID();
			addChangeFilter(ChangeFilterFactory.createIDFilter(m_cmdID,
					WorkingMemoryOperation.DELETE), this);
			try {
				addToWorkingMemory(m_cmdID, cmd);
			} catch (AlreadyExistsOnWMException e) {
				logException(e);
				executionComplete(TriBool.TRIFALSE);
			}
		}

		@Override
		public void stopExecution() {
			//can't stop this one
		}

		@Override
		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {		
			// always succeed, regardless of actual detections.
			executionComplete(TriBool.TRITRUE);
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
				new ActionExecutorFactory() {
					@Override
					public ActionExecutor getActionExecutor() {
						return new DetectObjectsActionExecutor();
					}
				});
	}

}
