package execution.components;

import cast.architecture.ManagedComponent;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.LogMessage;
import execution.slice.actions.PrintMessage;
import execution.slice.actions.Start;
import execution.slice.actions.Report;
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

		@Override
		public void stopExecution() {
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

		@Override
		public void stopExecution() {
		}

	}


        private class StartExecutor implements ActionExecutor {

		public boolean accept(Action _action) {
			return true;
		}

		public TriBool execute() {
		        try { 
			    Runtime rt = Runtime.getRuntime(); 
			    Process p = rt.exec("espeak -s90 'Hi,_my_name_is_barbie,_I_love_you_very_much._Mathematics_is_hard._Lets_go_shopping.'");
			 //   p.waitFor();
			} catch(Exception e) { 
			    System.out.println(e.getMessage()); 
			} 
			return TriBool.TRITRUE;
		}

		public void execute(ExecutionCompletionCallback _callback) {
		    //_callback.executionComplete(TriBool.TRITRUE);
		}

		public boolean isBlockingAction() {
			return true;
		}

		@Override
		public void stopExecution() {
		}

	}

        private class ReportExecutor implements ActionExecutor {

	    String msg;

		public boolean accept(Action _action) {
		    msg = ((Report)_action).message;
			return true;
		}

		public TriBool execute() {
		        try {
			    System.out.println("We are finished");
			    System.out.println(this.msg);
			    Runtime rt = Runtime.getRuntime();
			    Process p = rt.exec(this.msg);
			    p.waitFor();
			    Process p1 = rt.exec("eog /home/cogx/team_green");
			    p1.waitFor();
			} catch(Exception e) { 
			    System.out.println(e.getMessage()); 
			} 
			return TriBool.TRITRUE;
		}

		public void execute(ExecutionCompletionCallback _callback) {
		}

		public boolean isBlockingAction() {
			return true;
		}

		@Override
		public void stopExecution() {
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
		m_actionStateManager.registerActionType(Start.class,
				new ActionExecutorFactory() {
					public ActionExecutor getActionExecutor() {
						return new StartExecutor();
					}
				});
		m_actionStateManager.registerActionType(Report.class,
				new ActionExecutorFactory() {
					public ActionExecutor getActionExecutor() {
						return new ReportExecutor();
					}
				});

	}


}
