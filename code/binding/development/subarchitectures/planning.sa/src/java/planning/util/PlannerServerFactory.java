/**
 * Components for the planning subarchitecture
 */
package planning.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Properties;

import org.omg.CORBA.COMM_FAILURE;
import org.omg.CORBA.ORB;
import org.omg.CosNaming.NameComponent;
import org.omg.CosNaming.NamingContextExt;
import org.omg.CosNaming.NamingContextExtHelper;
import org.omg.CosNaming.NamingContextPackage.CannotProceed;
import org.omg.CosNaming.NamingContextPackage.InvalidName;
import org.omg.CosNaming.NamingContextPackage.NotFound;

import Planner.CCPState;
import Planner.Command;
import Planner.ExecutionState;
import Planner.Fact;
import Planner.PlannerServer;
import Planner.PlannerServerHelper;
import Planner.PlanningState;
import Planner.PlanningTask;
import Planner.PlanningTaskHolder;
import balt.corba.impl.RemoteConnectionManager;

/**
 * Component for interacting with the planner.
 * 
 * @author nah
 */
public class PlannerServerFactory {

	private static final String FF_EXE = "subarchitectures/planning.sa/src/python/mapsim/planning/planner_base/ContinualAxff/ff";
	private static final String VAL_EXE = "subarchitectures/planning.sa/src/python/mapsim/planning/plan_monitor/VAL/validate";

	private static boolean checkExists(String _s) {
		return (new File(_s)).exists();
	}

	private static class ReaderRunnable implements Runnable {

		private final BufferedReader m_reader;
		private final String m_prefix;

		public ReaderRunnable(BufferedReader _reader, String _prefix) {
			m_reader = _reader;
			m_prefix = _prefix;
		}
		public ReaderRunnable(BufferedReader _reader) {
			this(_reader, "");
		}

		public void run() {
			String s;
			// read the output from the command
			try {
				while ((s = m_reader.readLine()) != null) {
					System.out.println(m_prefix + s);
				}
			}
			catch (IOException e) {
				e.printStackTrace();
			}

			System.out.println(m_prefix + "done");

		}
	}

	private static class PlannerServerThread implements Runnable {

		private final String m_executableName;
		private final String m_serverName;
		private Process m_process;

		private PlannerServerThread(String _planner_server_name,
				String _planner_executable) {
			super();
			m_serverName = _planner_server_name;
			m_executableName = _planner_executable;
		}

		public void run() {

			try {
				String plannerCommand = m_executableName
						+ " -ORBInitRef NameService=corbaname::"
						+ RemoteConnectionManager.getNamingHost() + ":"
						+ RemoteConnectionManager.getNamingPort() + " "
						+ m_serverName;

				m_process = Runtime.getRuntime().exec(plannerCommand);
				BufferedReader stdOutput = new BufferedReader(
						new InputStreamReader(m_process.getInputStream()));

				BufferedReader stdError = new BufferedReader(
						new InputStreamReader(m_process.getErrorStream()));

				new Thread(new ReaderRunnable(stdOutput, "planner stdout: "))
						.start();
				new Thread(new ReaderRunnable(stdError, "planner stderr: "))
						.start();
			}
			catch (IOException e) {
				e.printStackTrace();
				System.exit(-1);
			}

			Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
				public void run() {
					PlannerServerFactory.shutdown();
					m_process.destroy();
				}
			}));

		}
	}

	private static String m_plannerExecutable = "./subarchitectures/planning.sa/src/python/mapsim/planning/server.py";

	// private static final String m_plannerExecutable =
	// "/home/brenner/cast/code/subarchitectures/planning.sa/src/python/cast_planner/server.py";
	static public final String PLANNER_SERVER_NAME = "CASTPlannerServer";
	private static PlannerServer m_plannerServer = null;

	private static void connectToPlannerServer() throws PlanningException {

		try {
			// use the orb that is being used by balt for everything
			// else
			ORB orb = RemoteConnectionManager.getORB();

			// get the root naming context
			org.omg.CORBA.Object objRef = orb
					.resolve_initial_references("NameService");
			NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

			// Now find the planner

			NameComponent nc2 = new NameComponent(PLANNER_SERVER_NAME, "Object");
			NameComponent[] name2 = {nc2};

			m_plannerServer = PlannerServerHelper.narrow(ncRef.resolve(name2));
		}
		catch (InvalidName e) {
			throw new PlanningException(
					"Unable to connect to planner server 1.", e);
		}
		catch (org.omg.CORBA.ORBPackage.InvalidName e) {
			throw new PlanningException(
					"Unable to connect to planner server 2.", e);
		}
		catch (NotFound e) {
			throw new PlanningException(
					"Unable to connect to planner server 3.", e);
		}
		catch (CannotProceed e) {
			throw new PlanningException(
					"Unable to connect to planner server 4.", e);
		}
	}

	/**
	 * @throws PlanningException
	 */
	private static void launchPlannerServer() throws PlanningException {

		// create the planner server interaction in a new thread
		new Thread(new PlannerServerFactory.PlannerServerThread(
				PLANNER_SERVER_NAME, m_plannerExecutable)).start();

		try {
			// sleep for a little while to give it time to start up
			Thread.sleep(1000);
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	// private static void testPlannerServer() {
	// m_plannerServer.self_test();
	// }

	/**
	 * @param _plannerExecutable
	 *            the plannerExecutable to set
	 */
	public static void setPlannerExecutablePath(String _plannerExecutable) {
		m_plannerExecutable = _plannerExecutable;
	}

	private static void init() {

		try {

			if (!checkExists(FF_EXE)) {
				throw new RuntimeException(
						"ff executable not found, please \"cd ./subarchitectures/planning.sa/src/python/mapsim/; ./make.sh\"");
			}

			if (!checkExists(VAL_EXE)) {
				throw new RuntimeException(
						"ff executable not found, please \"cd ./subarchitectures/planning.sa/src/python/mapsim/; ./make.sh\"");
			}

			// lauch planner server
			launchPlannerServer();

			boolean connected = false;
			while (!connected) {
				try {

					// first things first, connect to planner server
					connectToPlannerServer();
					connected = true;

				}
				catch (PlanningException e) {
					System.out
							.println("Connection failed, trying again shortly");
					try {
						Thread.sleep(5000);
					}
					catch (InterruptedException e1) {
						e1.printStackTrace();
					}
				}
			}

			// println("established connection with planner server");

			// testPlannerServer();
			// m_plannerServer.kill_me();
		}
		catch (PlanningException e) {
			e.printStackTrace();
		}
		catch (org.omg.CORBA.COMM_FAILURE e) {
			e.printStackTrace();
			System.out
					.println("communication failure: check that the server is live");
		}
	}

	public static PlannerServer getPlannerServer() {

		if (m_plannerServer == null) {
			init();
		}
		return m_plannerServer;

	}

	/**
	 * Shutdown the planner server. Probably won't come back up at the moment!
	 */
	public static void shutdown() {
		if (m_plannerServer != null) {
			try {
				m_plannerServer.kill_me();
			}
			catch (COMM_FAILURE e) {
				// well this probably means it's dead anyway
			}
		}
		m_plannerServer = null;
	}

	public static void main(String[] args) {
		testServer();
	}

	public static boolean selfTestServer() {
		prep();
		PlannerServer planner = PlannerServerFactory.getPlannerServer();
		planner.self_test();
		planner.kill_me();
		return true;
	}

	public static boolean testServer() {
		prep();

		PlannerServer planner = PlannerServerFactory.getPlannerServer();

		try {

			PlanningTask currentTask = planner
					.load_mapl_task(
							"subarchitectures/planning.sa/src/python/mapsim/domains/coffee/scenarios/one_agent/prob001/R2D2.mapl",
							"subarchitectures/planning.sa/src/python/mapsim/domains/coffee/mapl_files/domain_agents.mapl",
							"R2D2");

			System.out.println(PlanningUtils.toString(currentTask));

			PlanningTaskHolder currentTaskHolder = new PlanningTaskHolder(
					currentTask);
			planner.new_task(currentTaskHolder);
			currentTask = currentTaskHolder.value;

			System.out.println("The following test task will be used:\n"
					+ PlanningUtils.toString(currentTask));

			String taskID = currentTask.task_id;
			ArrayList<Command> cmdHistory = new ArrayList<Command>();

			// while True:
			while (true) {

				// if DEBUG: print "Current task:\n", current_task
				System.out.println("Current task:\n"
						+ PlanningUtils.toString(currentTask));

				// monitor the plan and eventually repair it
				assert (taskID != null);
				assert (!taskID.equals(""));

				CCPState ccpState = planner.continual_planning(taskID);

				if (ccpState.planning_state == PlanningState.CHANGED_PLAN) {
					System.out.println("Replanning was triggered.");
				}
				if (ccpState.execution_state == ExecutionState.GOAL_ACHIEVED) {
					// # goal achieved
					break;
				}
				else if (ccpState.execution_state == ExecutionState.GOAL_UNACHIEVABLE) {
					// no way to achieve the goal anymore --> failure
					System.out.println("Test failed. Could not achieve goal.");
					return false;
				}

				// get executable steps from the current plan
				Command[] possibleCommands = planner
						.next_executable_plan_steps(taskID);

				System.out.println("Possible commands:");
				for (Command cmd : possibleCommands) {
					System.out.println(PlanningUtils.toString(cmd));
				}

				// just pick first
				Command command = possibleCommands[0];
				System.out.println("Chosen command:\n"
						+ PlanningUtils.toString(command));

				Fact[] changes = planner.expected_changes(taskID, command);
				System.out.println("Expected changes:");
				for (Fact chg : changes) {
					System.out.println(PlanningUtils.toString(chg));
				}
				// for now simply assume successful execuction
				// inform planner about execuction
				System.out.println("Simulating execution of:\n"
						+ PlanningUtils.toString(command));

				planner.command_was_executed(taskID, command, true);
				cmdHistory.add(command);

				// assume perfect execution and full observability here:
				// add expected_changes as real changes!
				// planner.update_state(task_id, changes)
				// not necessary here because command_was_executed() does it
				// already. do we want that?
				currentTask = planner.current_task_state(taskID);
			}
			System.out
					.println("problem solved by executing the following commands:");

			for (Command cmd : cmdHistory) {
				System.out.println(PlanningUtils.toString(cmd));
			}

		}

		catch (Throwable t) {
			t.printStackTrace();
			planner.kill_me();
			return false;
		}

		planner.kill_me();
		return true;

	}

	private static void prep() {
		Properties props = new Properties();
		props.put("org.omg.CORBA.ORBInitialPort", "1050");
		props.put("org.omg.CORBA.ORBInitialHost", "localhost");
		// initialise orb
		ORB.init(new String[]{}, props);
		// initialise balt
		RemoteConnectionManager.init(new String[]{}, props);

		// tell the planner server where the server executable is
		PlannerServerFactory
				.setPlannerExecutablePath("./subarchitectures/planning.sa/src/python/mapsim/planning/server.py");
	}

}
