/**
 * 
 */
package motivation.components.generators;

import static org.junit.Assert.*;
import motivation.slice.ExternalGoalServer;
import motivation.slice.ExternalGoalServerPrx;
import motivation.slice.MotiveStatus;

import org.apache.log4j.Logger;
import org.junit.Test;

import cast.CASTException;
import cast.cdl.ComponentLanguage;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class ExternalGoalGeneratorClient {

	/** timeout of 30 minutes */
	private static final long TIMEOUT = 30 * 60 * 1000;
	Logger logger = Logger.getLogger(ExternalGoalGeneratorClient.class);
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
		if (result == MotiveStatus.ACTIVE)
			return true;
		else
			return false;
	}

	String getGoal() {
		String g = System.getProperty("goal");
		if (g == null)
			g = "(and )";
		return g;
	}

	float getImportance() {
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

		String goal = getGoal();
		float importance = getImportance();
		logger.info("got goal " + goal + ", got importance " + importance);
		assertTrue(call(goal, importance));
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
		// TODO Auto-generated method stub

	}

}
