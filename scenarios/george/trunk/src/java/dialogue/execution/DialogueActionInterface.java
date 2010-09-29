package dialogue.execution;

import cast.architecture.ManagedComponent;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForShape;
import execution.util.DoNothingActionExecutorFactory;
import execution.util.LocalActionStateManager;


/**
 * Receives actions from the execution system and interfaces with the rest of the dialogue system.
 * 
 * @author nah
 *
 */
public class DialogueActionInterface extends ManagedComponent {
	private LocalActionStateManager m_actionStateManager;

	
	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);
		
		DoNothingActionExecutorFactory derelictFactory = new DoNothingActionExecutorFactory(this);
		
		m_actionStateManager.registerActionType(AskForColour.class, derelictFactory);
		m_actionStateManager.registerActionType(AskForShape.class, derelictFactory);
		m_actionStateManager.registerActionType(AskForIdentity.class, derelictFactory);
		
	}
}
