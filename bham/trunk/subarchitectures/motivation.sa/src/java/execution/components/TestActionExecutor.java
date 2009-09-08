package execution.components;

import cast.architecture.ManagedComponent;
import execution.slice.Action;
import execution.slice.LogMessage;
import execution.slice.PrintMessage;
import execution.slice.TriBool;
import execution.util.ActionExecutor;
import execution.util.ActionExecutorFactory;
import execution.util.LocalActionStateManager;

public class TestActionExecutor extends ManagedComponent {

	private LocalActionStateManager m_actionStateManager;

	private class LogExecutor extends Thread implements ActionExecutor {

		private LogMessage m_lm;
		private ExecutionCompletionCallback m_callback;

		public boolean accept(Action _action) {
			m_lm = (LogMessage) _action;
			return true;
		}

		public TriBool execute() {
			assert true : "non-blocking execution should be used";
			return null;
		}

		public void execute(ExecutionCompletionCallback _callback) {
			//store callback
			m_callback = _callback;
			start();
		}

		@Override
		public void run() {
			try {
				Thread.sleep((long) (Math.random() * 5000));
				println(m_lm.message);
				m_callback.executionComplete(TriBool.TRITRUE);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		public boolean isBlockingAction() {
			return false;
		}

	}

	private class PrintExecutor implements ActionExecutor {

		private PrintMessage m_pm;

		public boolean accept(Action _action) {
			m_pm = (PrintMessage) _action;
			return true;
		}

		public TriBool execute() {
			println(m_pm.message);
			return TriBool.TRITRUE;
		}

		public void execute(ExecutionCompletionCallback _callback) {
			assert true : "blocking execution should be used";
		}

		public boolean isBlockingAction() {
			return true;
		}

	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		m_actionStateManager.registerActionType(PrintMessage.class,
				new ActionExecutorFactory() {
					public ActionExecutor getActionExecutor() {
						return new PrintExecutor();
					}
				});
		m_actionStateManager.registerActionType(LogMessage.class,
				new ActionExecutorFactory() {
					public ActionExecutor getActionExecutor() {
						return new LogExecutor();
					}
				});

	}


}
