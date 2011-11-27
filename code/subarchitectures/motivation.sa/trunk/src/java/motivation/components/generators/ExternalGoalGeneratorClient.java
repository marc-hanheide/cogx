/**
 * 
 */
package motivation.components.generators;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import motivation.slice.ExternalGoalServer;
import motivation.slice.ExternalGoalServerPrx;
import motivation.slice.MotiveStatus;

import org.apache.log4j.Logger;
import org.junit.Test;

import cast.CASTException;
import cast.cdl.ComponentLanguage;
import cast.core.CASTUtils;

/**
 * implements a JUnit4 test where it calls the ExternalGoalGenerator component
 * with a predefined goal string that will then cause a GeneralGoalMotive to be
 * created and monitored in the working memory.
 * 
 * Here is an example how this can be invoked in an ANT file:
 * 
 * <pre>
 * 	<target name="goaltest">
 * 		<junit fork="no">
 * 			<sysproperty key="goal" value="${test.goal}"/>
 * 		  	<classpath>
 * 				<fileset dir="${jar.dir}">
 *   					<include name="*.jar"/>
 * 				</fileset>
 *     			<pathelement location="${output.dir}"/>
 *     			<pathelement location="${cast.jar}"/>
 *     			<pathelement location="${ice.jar}"/>
 *   			</classpath>
 * 
 *     		<formatter type="xml"/> 
 *   			
 * 			<test name="motivation.components.generators.ExternalGoalGeneratorClient" haltonfailure="yes" outfile="result" />
 * 		</junit>
 * 	</target>
 * 
 * </pre>
 * 
 * @author marc
 * 
 */
public class ExternalGoalGeneratorClient {

	/** timeout of 30 minutes */
	private static final long TIMEOUT = 30 * 60 * 1000;
	static Logger logger = Logger.getLogger(ExternalGoalGeneratorClient.class);
	private String serverHost;
	private String serverName;
	private ExternalGoalServerPrx server;

	void init() throws CASTException {
		logger.debug("creating server proxy");
		try {
			server = CASTUtils.getCASTIceServer(serverName, serverHost,
					ComponentLanguage.CPP, ExternalGoalServer.class,
					ExternalGoalServerPrx.class);
		} catch (CASTException e) {
			server = CASTUtils.getCASTIceServer(serverName, serverHost,
					ComponentLanguage.JAVA, ExternalGoalServer.class,
					ExternalGoalServerPrx.class);
		}
	}

	boolean call(String goalString, float importance) {
		logger.info("calling the server with goal " + goalString);
		MotiveStatus result = server.submitGoal(goalString, importance);
		logger.info("server returned with " + result);
		if (result == MotiveStatus.ACTIVE || result == MotiveStatus.COMPLETED)
			return true;
		else
			return false;
	}

	String getGoalFromProperty() {
		String g = System.getProperty("goal");
		if (g == null)
			g = "(and )";
		return g;
	}

	float getImportanceFromProperty() {
		String i = System.getProperty("importance");
		if (i == null)
			i = "-1";
		return Float.parseFloat(i);
	}

	@Test(timeout = TIMEOUT)
	public void testGoal() {
		try {
			init();
		} catch (CASTException e) {
			logger.error("exception in testGoal", e);
			fail("test failed from exception: " + e.message);
		}

		String goal = getGoalFromProperty();
		float importance = getImportanceFromProperty();
		logger.info("got goal " + goal + ", got importance " + importance);
		assertTrue("the goal " + goal
				+ " has not been achieved, but has now been removed.",
				call(goal, importance));
	}

	public ExternalGoalGeneratorClient() {
		super();
		serverHost = System.getProperty("hostname");
		if (serverHost == null)
			serverHost = "localhost";
		serverName = System.getProperty("server-name");
		if (serverName == null)
			serverName = "goal.server";
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		if (args.length < 1) {
			System.err
					.println("command line parameters: <goal> [<importance>]");
			System.exit(1);
		}

		ExternalGoalGeneratorClient client = new ExternalGoalGeneratorClient();
		try {
			client.init();
		} catch (CASTException e) {
			logger.error("exception in testGoal", e);
			System.exit(1);
		}

		String goal = args[0];
		float importance = -1;
		if (args.length > 1)
			importance = Float.parseFloat(args[1]);
		logger.info("got goal " + goal + ", got importance " + importance);
		if (client.call(goal, importance)) {
			logger.info("goal " + goal + " has been achieved.");
			System.exit(0);
		} else {
			logger.warn("goal " + goal + " has failed.");
			System.exit(1);
		}
	}

}
