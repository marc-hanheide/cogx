/**
 * Components for the planning subarchitecture
 */
package planning.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Properties;

import org.omg.CORBA.BAD_PARAM;
import org.omg.CORBA.ORB;
import org.omg.CORBA.portable.ApplicationException;
import org.omg.CosNaming.NameComponent;
import org.omg.CosNaming.NamingContextExt;
import org.omg.CosNaming.NamingContextExtHelper;
import org.omg.CosNaming.NamingContextPackage.CannotProceed;
import org.omg.CosNaming.NamingContextPackage.InvalidName;
import org.omg.CosNaming.NamingContextPackage.NotFound;

import Planner.PlannerServer;
import Planner.PlannerServerHelper;
import Planner.PlanningTask;
import Planner.PlanningTaskHolder;
import balt.corba.impl.RemoteConnectionManager;

/**
 * Component for interacting with the planner.
 * 
 * @author nah
 */
public class PlannerServerFactory {

	private static class PlannerServerThread implements Runnable {

		private final String m_executableName;
		private final String m_serverName;

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

				Process p = Runtime.getRuntime().exec(plannerCommand);

				BufferedReader stdInput = new BufferedReader(
						new InputStreamReader(p.getInputStream()));

				BufferedReader stdError = new BufferedReader(
						new InputStreamReader(p.getErrorStream()));

				String s;

				// should really launch 2 separate threads for these

				// read the output from the command
				while ((s = stdInput.readLine()) != null) {
					System.out.println("planner output: " + s);
				}

				// read any errors from the attempted command
				while ((s = stdError.readLine()) != null) {
					System.err.println("planner output: " + s);
				}

			}
			catch (IOException e) {
				e.printStackTrace();
				System.exit(-1);
			}

		}
	}

	private static String m_plannerExecutable; // /home/nah/svn.cosy/code/subarchitectures/planning.sa/src/python/cast_planner/server.py
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

			// lauch planner server
			launchPlannerServer();

			// first things first, connect to planner server
			connectToPlannerServer();

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
			m_plannerServer.kill_me();
		}

	}

	public static void main(String[] args) {
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
		PlannerServer server = PlannerServerFactory.getPlannerServer();

		try {

			PlanningTask task = server
					.load_mapl_task(
							"subarchitectures/planning.sa/src/python/mapsim/domains/coffee/scenarios/one_agent/prob001/R2D2.mapl",
							"subarchitectures/planning.sa/src/python/mapsim/domains/coffee/mapl_files/domain_agents.mapl",
							"R2D2");

			System.out.println(PlanningUtils.toString(task));
			PlanningTaskHolder holder = new PlanningTaskHolder(task);
			server.new_task(holder);
			// System.out.println(PlanningUtils.toString(task));
		}
		catch (Throwable t) {
			t.printStackTrace();
		}

		server.kill_me();

	}

}
